#pragma once

#include <esp_matter.h>

void sensor_batt_init( void );
void sensor_batt_create_cluster( esp_matter::node_t *node );