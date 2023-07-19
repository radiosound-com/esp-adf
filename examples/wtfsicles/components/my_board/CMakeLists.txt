# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES)
set(COMPONENT_PRIV_REQUIRES audio_sal audio_hal esp_dispatcher esp_peripherals display_service)

if(CONFIG_AUDIO_BOARD_CUSTOM)
message(STATUS "Current board name is " CONFIG_AUDIO_BOARD_CUSTOM)
list(APPEND COMPONENT_ADD_INCLUDEDIRS ./my_board_v1_0 ./modified_codec_driver)
set(COMPONENT_SRCS
./my_board_v1_0/board.c
./my_board_v1_0/board_pins_config.c
./modified_codec_driver/modified_codec.c
)
endif()

register_component()

# Extend the audio_board component with this component so that its board includes are accessible when compiling audio_board
idf_component_get_property(audio_board_lib audio_board COMPONENT_LIB)
set_property(TARGET ${audio_board_lib} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})
