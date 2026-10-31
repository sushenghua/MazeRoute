# The simulation core is the shared float variant in esp32/sim (header-only).
idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "." "../../sim"
                       PRIV_REQUIRES esp_driver_uart esp_timer esp_system spi_flash)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23 -O2 -Wno-missing-field-initializers)
