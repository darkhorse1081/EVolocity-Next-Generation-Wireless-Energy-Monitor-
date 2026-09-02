idf_component_register(SRCS "wifi_http_connection.c" "adc_reader.c"
    INCLUDE_DIRS "."
    REQUIRES ulp esp_adc esp_wifi esp_netif nvs_flash esp_event esp_http_client freertos soc
    PRIV_REQUIRES esp_timer esp_sntp
)

idf_component_set_property(driver WHOLE_ARCHIVE NO)

target_include_directories(${COMPONENT_LIB} PRIVATE
    $ENV{IDF_PATH}/components/ulp/ulp_fsm/include
    $ENV{IDF_PATH}/components/ulp/ulp_common/include
)

ulp_embed_binary(ulp_main "ulp_adc.S" "adc_reader.c")