#pragma once

#include <esp_matter.h>


void sensor_create_clusters(esp_matter::node_t *node);
void sensor_drv_init( void );
void batt_adc_init ( void );