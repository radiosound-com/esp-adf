set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES audio_sal nvs_flash spi_flash)
set(COMPONENT_PRIV_REQUIRES esp_dispatcher display_service esp-adf-libs audio_pipeline audio_sal audio_recorder)

if (CONFIG_SOC_WIFI_SUPPORTED OR CONFIG_ESP_HOSTED_ENABLED)
    list(APPEND COMPONENT_PRIV_REQUIRES wifi_service dueros_service)
endif()

set(COMPONENT_SRCS ./display_action.c
                    ./player_action.c
                    ./recorder_action.c
                    ./nvs_action.c
                    ./partition_action.c)

if (CONFIG_SOC_WIFI_SUPPORTED OR CONFIG_ESP_HOSTED_ENABLED)
    list(APPEND COMPONENT_SRCS  ./dueros_action.c ./wifi_action.c)
endif()

register_component()
