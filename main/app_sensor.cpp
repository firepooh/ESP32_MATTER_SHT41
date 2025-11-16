#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#include <i2cdev.h>
#include <app_sensor.h>
#include <sht4x.h>

/* test purpose config only */
//#define LCFG_VIRTUAL_SENSOR_DATA



#define SENSOR_UPDATE_PERIOD_SEC      60



/* I2C */
#define CONFIG_EXAMPLE_I2C_MASTER_SCL       GPIO_NUM_2
#define CONFIG_EXAMPLE_I2C_MASTER_SDA       GPIO_NUM_3
#define CONFIG_I2C_MASTER_NUM               I2C_NUM_0


static const char *TAG = "sensor";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

typedef struct {

  esp_timer_handle_t sensor_timer;

  uint16_t temp_endpoint_id;
  uint16_t humi_endpoint_id;
  uint16_t batt_endpoint_id;

  float temp;
  float humi;
  float batt_voltage;
  uint8_t batt_percentage;
    
} sensor_app_context_t;

sensor_app_context_t sensor_app_ctx = {
  .sensor_timer = NULL,
  .temp_endpoint_id = 0,
  .humi_endpoint_id = 0,
  .temp = 25.0f,
  .humi = 50.0f,
  .batt_voltage = 4.2f,
  .batt_percentage = 100,
};

sht4x_t sht4x_dev;


void sensor_get( float *temperature, float *humidity );

void sensor_update_temperature( uint16_t ep_id, float temp )
{
  // schedule the attribute update so that we can report it from matter thread
  chip::DeviceLayer::SystemLayer().ScheduleLambda([ep_id, temp]() {
    attribute_t * attribute = attribute::get(ep_id,
                                             TemperatureMeasurement::Id,
                                             TemperatureMeasurement::Attributes::MeasuredValue::Id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.i16 = static_cast<int16_t>(temp * 100);

    attribute::update(ep_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
  });
}


void sensor_update_humidity( uint16_t ep_id, float humi )
{
  // schedule the attribute update so that we can report it from matter thread
  chip::DeviceLayer::SystemLayer().ScheduleLambda([ep_id, humi]() {
    attribute_t * attribute = attribute::get(ep_id,
                                             RelativeHumidityMeasurement::Id,
                                             RelativeHumidityMeasurement::Attributes::MeasuredValue::Id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.u16 = static_cast<uint16_t>(humi * 100);

    attribute::update(ep_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
  });
}


void sensor_create_clusters( node_t *node )
{
  /* Create temperature */    
  temperature_sensor::config_t temp_sensor_config;
  endpoint_t * temp_sensor_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
  ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));
  sensor_app_ctx.temp_endpoint_id = esp_matter::endpoint::get_id(temp_sensor_ep);

  /* Create humidity */
  humidity_sensor::config_t humidity_sensor_config;
  endpoint_t * humidity_sensor_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
  ABORT_APP_ON_FAILURE(humidity_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));
  sensor_app_ctx.humi_endpoint_id = esp_matter::endpoint::get_id(humidity_sensor_ep);    
}


#if defined(LCFG_VIRTUAL_SENSOR_DATA)
static void sensor_test_data( sensor_app_context_t *ctx )
{
  ctx->temp += 0.1f;
  if( ctx->temp > 30 )
    ctx->temp = 25.0f;

  ctx->humi += 0.2f;
  if( ctx->humi > 60 )
    ctx->humi = 50.0f;
}
#endif

void sensor_timer_callback(void *arg)
{
  sensor_app_context_t *ctx = (sensor_app_context_t *)arg;

  if( ctx == NULL )
    return;

  #if defined(LCFG_VIRTUAL_SENSOR_DATA)
  sensor_test_data( &sensor_app_ctx );
  #else
  /* Read sensor data and update the attribute */
  sensor_get( &ctx->temp, &ctx->humi );
  #endif

  sensor_update_temperature( ctx->temp_endpoint_id, ctx->temp );
  sensor_update_humidity( ctx->humi_endpoint_id, ctx->humi );
  ESP_LOGI(TAG, "Sensor data updated: temp: %.2f, humi: %.2f", ctx->temp, ctx->humi );
 
}

void sensor_timer_init( void )
{
  /* esp timer create */
  esp_timer_create_args_t timer_args = {
    .callback = &sensor_timer_callback,
    .arg = &sensor_app_ctx,
    .name = "sensor_timer",
  };

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &sensor_app_ctx.sensor_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_app_ctx.sensor_timer, (SENSOR_UPDATE_PERIOD_SEC*1000*1000)));
}

void sensor_get( float *temperature, float *humidity )
{
  #if !defined(LCFG_VIRTUAL_SENSOR_DATA)
  /* Use High Level Driver */
  ESP_ERROR_CHECK(sht4x_measure(&sht4x_dev, temperature, humidity));
  ESP_LOGI(TAG,"sht4x Sensor: %.2f °C, %.2f %%\n", *temperature, *humidity);
  #endif
}

void sensor_drv_init( void )
{
  #if !defined(LCFG_VIRTUAL_SENSOR_DATA)
  ESP_ERROR_CHECK(i2cdev_init());

  /* SHT4x 초기화 */
  memset(&sht4x_dev, 0, sizeof(sht4x_t));
  ESP_ERROR_CHECK(sht4x_init_desc(&sht4x_dev, CONFIG_I2C_MASTER_NUM, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
  ESP_ERROR_CHECK(sht4x_init(&sht4x_dev));
  #endif

  sensor_timer_init();
}
