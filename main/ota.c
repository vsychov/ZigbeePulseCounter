#include "ota.h"

#include "esp_zigbee_ota.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "app_config.h"

void ota_init(void)
{
    esp_zb_ota_upgrade_client_query_interval_set(APP_ZB_ENDPOINT, ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF);
}
