set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

list(APPEND COMPONENT_ADD_INCLUDEDIRS 
                ./driver/i2c_bus)

list(APPEND COMPONENT_SRCS 
                ./driver/i2c_bus/i2c_bus.c
    )

set(COMPONENT_REQUIRES driver audio_sal)

register_component()
