#pragma once

#include <esp_matter.h>

void sensor_pwrs_drv_init( void );
void sensor_create_cluster_powersource( esp_matter::node_t *node );