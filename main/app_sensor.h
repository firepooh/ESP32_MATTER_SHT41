#pragma once

#include <esp_matter.h>


void sensor_create_clusters(esp_matter::node_t *node);
void sensor_timer_init( void );