idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES app_event app_network app_protocol app_storage app_espnow app_sensor
)
