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

#include "app_utils.h"

#include <i2cdev.h>
#include <app_sensor.h>
#include <sht4x.h>

/******************************************************************************************* */
/* Block시 i2c초기화 생략하고, 가상 데이터를 생성해서 전송. / Define시 실제 센서 데이터 사용.  */
#define LCFG_REAL_SENSOR_DATA
/* Block시 고정된 주기로 전송. / Define시 입력 센서값 변동에 따른 REPORT 주기를 가변. */
#define LCFG_SENSOR_REPORT_DYNAMIC_INTERVAL          
/******************************************************************************************* */

/******************************************************************************************* */
#define SENSOR_STATE_IDLE_CNTS        10
#define SENSOR_UPDATE_PERIOD_SEC      60
/******************************************************************************************* */
// Threshold 정의
#define TEMP_THRESHOLD 0.5f     // 온도 변화 임계값 (°C)
#define HUMI_THRESHOLD 2.0f     // 습도 변화 임계값 (%)
#define MIN_INTERVAL_SEC 60     // 최소 간격 (1분)
#define MAX_INTERVAL_SEC 300    // 최대 간격 (5분)
#define INTERVAL_STEP_SEC 60    // 간격 증가 단위 (1분)
/******************************************************************************************* */
/* TASK Defines */
#define SENSOR_TEMP_HUMI_TASK_STACK_SIZE    4096
#define SENSOR_TEMP_HUMI_TASK_PRIORITY      5
#define SENSOR_TEMP_HUMI_QUEUE_LENGTH       10
/******************************************************************************************* */
/* I2C */
#define CONFIG_EXAMPLE_I2C_MASTER_SCL       GPIO_NUM_2
#define CONFIG_EXAMPLE_I2C_MASTER_SDA       GPIO_NUM_3
#define CONFIG_I2C_MASTER_NUM               I2C_NUM_0
/******************************************************************************************* */

static const char *TAG = "sensor";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

typedef enum {
  SENSOR_STATE_BOOTUP = 0,
  SENSOR_STATE_IDLE,
  SENSOR_STATE_RUNNING,
} sensor_state_t;

const char * SENSOR_STATE_STRINGS[] = {
  "BOOTUP",
  "IDLE",
  "RUNNING",
};

typedef struct {

  sensor_state_t state[2];
  uint8_t        state_idle_cnts;
  esp_timer_handle_t sensor_timer;

  uint16_t temp_endpoint_id;
  float    temp;

  uint16_t humi_endpoint_id;
  float    humi;
    
} sensor_app_context_t;

sensor_app_context_t sensor_app_ctx;

sht4x_t sht4x_dev;

typedef struct {
  uint32_t event_type;
} sensor_event_t;

static QueueHandle_t sensor_queue = NULL;

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


void sensor_update( sensor_app_context_t *ctx, float temp, float humi )
{
  /* temp, humi 값이 이전 센서값이 소수점 첫째 자리에서 차이가 안나면 무시하고 전송 하지 않는다. */
  if( ctx == NULL )
    return;

  bool temp_changed = ((int)(ctx->temp * 10) != (int)(temp * 10)); /* 소수점 첫째 자리 비교 */
  bool humi_changed = ((int)(ctx->humi * 1) != (int)(humi * 1));   /* 정수부 비교 */

  if (temp_changed) {
    ctx->temp = temp;
    sensor_update_temperature(ctx->temp_endpoint_id, ctx->temp);
  }

  if (humi_changed) {
    ctx->humi = humi;
    sensor_update_humidity(ctx->humi_endpoint_id, ctx->humi);
  }
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


void sensor_timer_callback(void *arg)
{
  sensor_app_context_t *ctx = (sensor_app_context_t *)arg;

  if( ctx == NULL )
    return;

  /* send queue event */
  sensor_event_t evt;
  evt.event_type = 0;
  xQueueSend( sensor_queue, &evt, 0 );

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
  ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_app_ctx.sensor_timer, SEC2USEC(SENSOR_UPDATE_PERIOD_SEC)));
}


void sensor_get( float *temperature, float *humidity )
{
  #if defined(LCFG_REAL_SENSOR_DATA)
  
  /* Use High Level Driver */
  ESP_ERROR_CHECK(sht4x_measure(&sht4x_dev, temperature, humidity));
  
  #else
  /* make virtual sensor data */
  *temperature += 0.1f;
  if( *temperature > 30 )
    *temperature = 25.0f;

  *humidity += 0.2f;
  if( *humidity > 60 )
    *humidity = 50.0f;  
  
  #endif

  ESP_LOGI(TAG,"sht4x Sensor: %.2f °C, %.2f %%\n", *temperature, *humidity);
}


void sensor_drv_init( void )
{
  #if defined(LCFG_REAL_SENSOR_DATA)
  ESP_ERROR_CHECK(i2cdev_init());

  /* SHT4x 초기화 */
  memset(&sht4x_dev, 0, sizeof(sht4x_t));
  ESP_ERROR_CHECK(sht4x_init_desc(&sht4x_dev, CONFIG_I2C_MASTER_NUM, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
  ESP_ERROR_CHECK(sht4x_init(&sht4x_dev));
  #endif

  sensor_timer_init();
}


static void sensor_update_period_cfg( sensor_state_t state, esp_timer_handle_t timer_h, float temp, float humi )
{
  static float prev_temp = 0.0f;
  static float prev_humi = 0.0f;
  static uint32_t current_period_sec = MIN_INTERVAL_SEC;

  #if !defined(LCFG_SENSOR_REPORT_DYNAMIC_INTERVAL)
  /* test purpose only */
  /* 시작 timer 주기로 계속 동작. */
  return;
  #endif

  /* 부팅 후 초기 IDLE State에서는 짧은 고정 주기로 동작 한다. */
  if( state <= SENSOR_STATE_IDLE ) {
    prev_temp = temp;
    prev_humi = humi;
    return;
  }

  // 온도/습도 변화량 계산
  float temp_delta = (temp > prev_temp) ? (temp - prev_temp) : (prev_temp - temp);
  float humi_delta = (humi > prev_humi) ? (humi - prev_humi) : (prev_humi - humi);

  uint32_t new_period_sec = current_period_sec;
  bool period_changed = false;

  // threshold 초과: 1분으로 리셋
  if (temp_delta >= TEMP_THRESHOLD || humi_delta >= HUMI_THRESHOLD) {
    if (current_period_sec != MIN_INTERVAL_SEC) {
      new_period_sec = MIN_INTERVAL_SEC;
      period_changed = true;
      ESP_LOGI(TAG, "Change detected (T:%.2f°C, H:%.2f%%) - Reset to %d sec", temp_delta, humi_delta, (int)(new_period_sec));
    }
  }
  // threshold 이하: 1분씩 증가 (최대 5분)
  else {
    if (current_period_sec < MAX_INTERVAL_SEC) {
      new_period_sec = current_period_sec + INTERVAL_STEP_SEC;
      if (new_period_sec > MAX_INTERVAL_SEC) {
        new_period_sec = MAX_INTERVAL_SEC;
      }
      period_changed = true;
      ESP_LOGI(TAG, "Stable readings (T:%.2f°C, H:%.2f%%) - Increase to %d sec", temp_delta, humi_delta, (int)(new_period_sec));
    }
  }

  // 주기가 변경되면 타이머 재설정
  if (period_changed) {
    esp_timer_stop(timer_h);
    esp_timer_start_periodic(timer_h, SEC2USEC(new_period_sec));
    current_period_sec = new_period_sec;
  }

  // 현재 값 저장
  prev_temp = temp;
  prev_humi = humi;
}

static void sensor_temp_humi_state( sensor_app_context_t *ctx )
{
  float temp,humi;

  if( ctx == NULL )
    return;

  if( ctx->state[0] != ctx->state[1] ) {
    ctx->state[0] = ctx->state[1];
    ESP_LOGI(TAG, "Sensor State changed to %s", SENSOR_STATE_STRINGS[ctx->state[1]] );
  }

  switch( ctx->state[1] ) {

    case SENSOR_STATE_BOOTUP:
      /* 부팅 후 초기 상태 */
      ctx->state[1] = SENSOR_STATE_IDLE;
      break;

    case SENSOR_STATE_IDLE:
      /* get sensor data */
      sensor_get( &temp, &humi );
      /* report network*/
      sensor_update( &sensor_app_ctx, temp, humi );
      /* config update timer */
      sensor_update_period_cfg( ctx->state[1], sensor_app_ctx.sensor_timer, ctx->temp, ctx->humi );
      
      /* check next state */
      ctx->state_idle_cnts++;
      if( ctx->state_idle_cnts >= SENSOR_STATE_IDLE_CNTS ) {
        ctx->state_idle_cnts = 0;
        ctx->state[1] = SENSOR_STATE_RUNNING;
      }
      break;

    case SENSOR_STATE_RUNNING:
      /* 정상 동작 상태 */
      /* get sensor data */
      sensor_get( &temp, &humi );
      /* report network*/
      sensor_update( &sensor_app_ctx, temp, humi );
      /* config update timer */
      sensor_update_period_cfg( ctx->state[1], sensor_app_ctx.sensor_timer, ctx->temp, ctx->humi );

      /* 추후에 exception 상황에 따라 추가 */
      break;

    default:
      break;
  }
}

static void sensor_temp_humi_task(void *pvParameters) {
  sensor_event_t evt;

  ESP_LOGI(TAG, "Sensor task started");

  while (1) {
    if (xQueueReceive(sensor_queue, &evt, portMAX_DELAY) == pdTRUE) {
      /* state machine 처리 */
      sensor_temp_humi_state( &sensor_app_ctx );
#if 0
      // 센서 읽기
      sensor_get( &sensor_app_ctx.temp, &sensor_app_ctx.humi );
           
      // Matter/Thread로 리포팅
      sensor_update( &sensor_app_ctx );
      
      // 간격 조정
      #if defined(LCFG_SENSOR_REPORT_DYNAMIC_INTERVAL)
      sensor_update_period_cfg( sensor_app_ctx.state[1], sensor_app_ctx.sensor_timer, sensor_app_ctx.temp, sensor_app_ctx.humi );
      #else
      /* test purpose : fixed period report */
      ESP_ERROR_CHECK(esp_timer_start_once(sensor_app_ctx.sensor_timer, SEC2USEC(SENSOR_UPDATE_PERIOD_SEC)));
      #endif
#endif      
    }
  }
}


void sensor_temp_humi_task_create( esp_matter::node_t *node  )
{
  /* create clusters */
  sensor_create_clusters( node );
  
  /* Initialize driver */
  sensor_drv_init();

  /* Create sensor queue */
  sensor_queue = xQueueCreate(SENSOR_TEMP_HUMI_QUEUE_LENGTH, sizeof(sensor_event_t));
  if (sensor_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create sensor queue");
    return;
  }

  /* Create sensor task */
  BaseType_t result = xTaskCreate(
    sensor_temp_humi_task,
    "SensorTask",
    SENSOR_TEMP_HUMI_TASK_STACK_SIZE,
    NULL,
    SENSOR_TEMP_HUMI_TASK_PRIORITY,
    NULL
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create sensor task");
    vQueueDelete(sensor_queue);
    return;
  }

  /* debug LCFG Setup information */
  #if defined(LCFG_REAL_SENSOR_DATA)
  ESP_LOGI(TAG, "LCFG_REAL_SENSOR_DATA is defined");
  #else
  ESP_LOGI(TAG, "LCFG_REAL_SENSOR_DATA is NOT defined");
  #endif

  #if defined(LCFG_SENSOR_REPORT_DYNAMIC_INTERVAL)
  ESP_LOGI(TAG, "LCFG_SENSOR_REPORT_DYNAMIC_INTERVAL is defined");
  #else
  ESP_LOGI(TAG, "LCFG_SENSOR_REPORT_DYNAMIC_INTERVAL is NOT defined");
  #endif
  
  ESP_LOGI(TAG, "Sensor task and queue created successfully");
}