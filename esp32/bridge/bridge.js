// bridge.js — relays the ESP32-S3 protocol between UART and the browser, and serves esp32/web.
//
//   browser  --ws text frame: one command-->  bridge  --"command\n"-->  ESP32-S3 UART0
//   browser  <--ws text frame: one JSON-----  bridge  <--JSON lines---  ESP32-S3 UART0
//
// Usage: node bridge.js [serialPort] [httpPort]. Without a serial port, the board's CH343 UART
// bridge (USB vendor 1a86) is picked automatically.
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { extname, join, normalize } from 'node:path';
import { fileURLToPath } from 'node:url';
import { SerialPort } from 'serialport';
import { ReadlineParser } from '@serialport/parser-readline';
import { WebSocketServer } from 'ws';

const BAUD = 921600;
const HTTP_PORT = Number(process.argv[3] || 8766);
const WEB_ROOT = fileURLToPath(new URL('../web/', import.meta.url));
const TYPES = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml' };

async function findPort() {
  if (process.argv[2]) return process.argv[2];
  const ports = await SerialPort.list();
  const ch343 = ports.find((p) => (p.vendorId || '').toLowerCase() === '1a86');
  if (!ch343) throw new Error(`no CH343 UART found; pass the port explicitly. Seen: ${ports.map((p) => p.path).join(', ')}`);
  return ch343.path.replace('/dev/tty.', '/dev/cu.');
}

const clients = new Set();
function broadcast(text) { for (const ws of clients) if (ws.readyState === 1) ws.send(text); }

const path = await findPort();
// Opening the port must not reset the board: its auto-reset circuit pulls EN low when RTS is asserted
// without DTR, so both lines are released together right after open.
const serial = new SerialPort({ path, baudRate: BAUD, hupcl: false, autoOpen: false });
let connected = false;
serial.open((err) => {
  if (err) { console.error(`cannot open ${path}: ${err.message}`); process.exit(1); }
  serial.set({ dtr: false, rts: false }, () => {
    connected = true;
    console.log(`serial ${path} @ ${BAUD} open`);
    broadcast(JSON.stringify({ k: 'bridge', port: path, connected }));
  });
});
serial.on('close', () => { connected = false; broadcast(JSON.stringify({ k: 'bridge', port: path, connected })); });
serial.on('error', (e) => console.error('serial error:', e.message));
serial.pipe(new ReadlineParser({ delimiter: '\n' })).on('data', (line) => {
  line = line.trim();
  if (line.startsWith('{')) broadcast(line);  // ROM boot text and logs are not protocol
  else if (line) console.log('[esp32]', line);
});

const http = createServer(async (req, res) => {
  const rel = normalize(decodeURIComponent(new URL(req.url, 'http://x').pathname)).replace(/^[/\\]+/, '') || 'index.html';
  if (rel.startsWith('..')) { res.writeHead(403).end(); return; }
  try {
    const body = await readFile(join(WEB_ROOT, rel));
    res.writeHead(200, { 'content-type': TYPES[extname(rel)] || 'application/octet-stream', 'cache-control': 'no-store' }).end(body);
  } catch {
    res.writeHead(404).end('not found');
  }
});

const wss = new WebSocketServer({ server: http, path: '/ws' });
wss.on('connection', (ws) => {
  clients.add(ws);
  ws.send(JSON.stringify({ k: 'bridge', port: path, connected }));
  ws.on('message', (data) => {
    const cmd = String(data).trim();
    if (!/^[a-z]+( [-0-9. ]+)?$/.test(cmd)) return;  // the firmware only understands "word [numbers]"
    if (connected) serial.write(cmd + '\n');
  });
  ws.on('close', () => clients.delete(ws));
});

http.listen(HTTP_PORT, '127.0.0.1', () => console.log(`open http://127.0.0.1:${HTTP_PORT}/`));
