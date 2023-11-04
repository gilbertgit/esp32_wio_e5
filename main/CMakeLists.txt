# Edit following two lines to set component requirements (see docs)
# set(COMPONENT_REQUIRES )
# set(COMPONENT_PRIV_REQUIRES )

# set(COMPONENT_SRCS "main.c")
# set(COMPONENT_SRCS "bme280.c")
# set(COMPONENT_ADD_INCLUDEDIRS "")
idf_component_register(SRCS "main.c" "bme280.c"
                       INCLUDE_DIRS ".")

# register_component()
