#pragma once

#include "app_config.h"
#include "esp_zigbee_type.h"

void config_cluster_add(esp_zb_cluster_list_t *cluster_list, app_metering_cfg_t *cfg);
void config_cluster_apply_pending(app_metering_cfg_t *cfg);
void config_cluster_register_callbacks(void);
