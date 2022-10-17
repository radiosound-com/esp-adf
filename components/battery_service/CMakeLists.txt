set(COMPONENT_SRCS "battery_service.c"
                    "./monitors/voltage_monitor.c")

set(COMPONENT_ADD_INCLUDEDIRS "include"
                                "./monitors/include")

set(COMPONENT_REQUIRES esp_adc audio_sal esp_peripherals)

register_component()
