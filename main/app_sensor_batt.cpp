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

#include <app_sensor_batt.h>

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#define VBAT_ADC_UNIT         ADC_UNIT_1
#define VBAT_ADC_CHANNEL      ADC_CHANNEL_5   // 예: ESP32-C6 GPIO0. 보드에 맞게 변경
#define VBAT_ADC_ATTEN        ADC_ATTEN_DB_12 //  ~3.3V 대응(분배 후 입력전압 기준)
#define VBAT_DIV_R1           2               /* 2Mohm */
#define VBAT_DIV_R2           2               /* 2Mohm */




/* 부팅 후 아래 조건 동안 UPDATE */
#define BATT_STARTUP_UPDATE_PERIOD_SEC      (60)      /* 1 minute */ 
#define BATT_STARTUP_UPDATE_PERIOD_CNTS     (10)      /* 10회 */ 
/* 부팅 완료 후 아래 조건 동안 UPDATE */
#define BATT_UPDATE_PERIOD_SEC              (60*60*6) /* 6 hours */

static const char *TAG = "pwr";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;


typedef struct {
  esp_timer_handle_t sensor_timer;
  uint8_t startup_update_count;

  uint16_t powers_endpoint_id;

  int     batt_mv;          /* mV 단위 */
  uint8_t batt_percentage;
} sensor_powers_context_t;

sensor_powers_context_t sensor_powers_ctx = {
  .sensor_timer = NULL,
  .startup_update_count = 0,
  .powers_endpoint_id = 0,
  .batt_mv = 4200,
  .batt_percentage = 50,
};

typedef struct {
  bool do_calibration_chan;
  adc_oneshot_unit_handle_t adc_handle;
  adc_cali_handle_t adc_cali_chan_handle;
} adc_context_t;

adc_context_t adc_ctx = {
  .do_calibration_chan = false,
  .adc_handle = NULL,
  .adc_cali_chan_handle = NULL,
};


static void sensor_batt_update(uint16_t endpoint_id, int millivolts, uint8_t percentage, void *user_data);

static void batt_adc_init ( adc_context_t *adc_ctx );
static void batt_adc_deinit( adc_context_t *adc_ctx );
static bool batt_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void batt_adc_calibration_deinit(adc_cali_handle_t handle);
static void batt_timer_init( void );


static uint8_t batt_adc_voltage_to_percentage( int millivolts )
{
  /* 임시 디버깅 */
  /* percentage는 임시로 배터리 전압을 전달 한다. 42% = 4.2V */
  return( (uint8_t)(millivolts/100) );
}

static int batt_adc_read_mv( adc_context_t *adc_ctx )
{
  int adc_raw;
  int adc_millivolts;

  if( adc_ctx == NULL ) {
    ESP_LOGE(TAG, "ADC context is NULL");
    return -1;
  }

  /* light_sleep 활성화 된 상태에서 adc 측정시 매번 init()/deinit() 절차 수행 */
  /* https://github.com/espressif/esp-zigbee-sdk/issues/236 */
  /*****************************************************************************************/
  batt_adc_init( adc_ctx );

  ESP_ERROR_CHECK(adc_oneshot_read(adc_ctx->adc_handle, VBAT_ADC_CHANNEL, &adc_raw));
  ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, VBAT_ADC_CHANNEL, adc_raw);
  if (adc_ctx->do_calibration_chan) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_ctx->adc_cali_chan_handle, adc_raw, &adc_millivolts));
    ESP_LOGD(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, VBAT_ADC_CHANNEL, adc_millivolts);
  }

  batt_adc_deinit( adc_ctx );
  /*****************************************************************************************/

  /* 분배 저항값 적용 */
  adc_millivolts = adc_millivolts * ( (float)(VBAT_DIV_R1 + VBAT_DIV_R2) / (float)VBAT_DIV_R2 );

  return( adc_millivolts ); /* return unit (mV) */
}


static void batt_adc_init ( adc_context_t *adc_ctx )
{
  //-------------ADC1 Init---------------//
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = VBAT_ADC_UNIT,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_ctx->adc_handle));

  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
    .atten = VBAT_ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_ctx->adc_handle, VBAT_ADC_CHANNEL, &config));

  //-------------ADC1 Calibration Init---------------//
  adc_ctx->do_calibration_chan = batt_adc_calibration_init(VBAT_ADC_UNIT, VBAT_ADC_CHANNEL, VBAT_ADC_ATTEN, &adc_ctx->adc_cali_chan_handle);

#if 0
  // adc 초기화 할때 해당 gpio 설정이 이미 수행 됨. 필요 없음.
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << GPIO_NUM_5),  // 또는 사용할 GPIO 번호
    .mode = GPIO_MODE_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);  
#endif  
}


static void batt_adc_deinit( adc_context_t *adc_ctx )
{
  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_ctx->adc_handle));

  if (adc_ctx->do_calibration_chan) {
    batt_adc_calibration_deinit(adc_ctx->adc_cali_chan_handle);
  }
}


static bool batt_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGD(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}


static void batt_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}


void batt_timer_callback(void *arg)
{
  sensor_powers_ctx.batt_mv = batt_adc_read_mv( &adc_ctx );
  /* percentage는 임시로 배터리 전압을 전달 한다. */
  sensor_powers_ctx.batt_percentage = batt_adc_voltage_to_percentage( sensor_powers_ctx.batt_mv );

  ESP_LOGI(TAG, "ADC Timer Callback: Voltage: %d mV, Percentage: %d %%", sensor_powers_ctx.batt_mv, sensor_powers_ctx.batt_percentage/2);
  sensor_batt_update( sensor_powers_ctx.powers_endpoint_id, sensor_powers_ctx.batt_mv, sensor_powers_ctx.batt_percentage, NULL );

  
  if( sensor_powers_ctx.startup_update_count < BATT_STARTUP_UPDATE_PERIOD_CNTS ) {
    /* 부팅 후 초기 업데이트 기간 동안은 짧은 주기로 업데이트 */
    sensor_powers_ctx.startup_update_count++;
    ESP_ERROR_CHECK(esp_timer_start_once(sensor_powers_ctx.sensor_timer, SEC2USEC(BATT_STARTUP_UPDATE_PERIOD_SEC)));
  }else {
    /* 이후에는 긴 주기로 업데이트 */
    ESP_ERROR_CHECK(esp_timer_start_once(sensor_powers_ctx.sensor_timer, SEC2USEC(BATT_UPDATE_PERIOD_SEC)));
  }
}


static void batt_timer_init( void )
{
  /* esp timer create */
  esp_timer_create_args_t timer_args = {
    .callback = &batt_timer_callback,
    .name = "powers_timer",
  };

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &sensor_powers_ctx.sensor_timer));
  ESP_ERROR_CHECK(esp_timer_start_once(sensor_powers_ctx.sensor_timer, SEC2USEC(BATT_STARTUP_UPDATE_PERIOD_SEC)));
}


void sensor_batt_init( void )
{
  int mV = 0;
  int mV_avg = 0;
  int avg_count = 4;

  /***************************************** */
  for( int i=0; i < avg_count; i++ )
  {
    mV = batt_adc_read_mv( &adc_ctx );
    mV_avg += mV;
  }
  mV_avg /= avg_count;
  /***************************************** */
  
  sensor_powers_ctx.batt_mv = mV_avg ;
  /* percentage는 임시로 배터리 전압을 전달 한다. */
  sensor_powers_ctx.batt_percentage = batt_adc_voltage_to_percentage( sensor_powers_ctx.batt_mv );

  /* battery timer start */
  batt_timer_init();

  /* cluster 생성후, 초기값 update */
  sensor_batt_update( sensor_powers_ctx.powers_endpoint_id, sensor_powers_ctx.batt_mv, sensor_powers_ctx.batt_percentage, NULL );

  ESP_LOGI(TAG, "Battery voltage: %d mV", sensor_powers_ctx.batt_mv);
}


static void sensor_batt_update(uint16_t endpoint_id, int millivolts, uint8_t percentage, void *user_data)
{
  //uint32_t voltage_mv = (uint32_t)(voltage);

  chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, millivolts, percentage]() {
    // BatPercentRemaining 업데이트
    esp_matter_attr_val_t percent_val = esp_matter_nullable_uint8(percentage * 2);
    attribute::update(endpoint_id, PowerSource::Id,
                     PowerSource::Attributes::BatPercentRemaining::Id, &percent_val);

    // BatVoltage 업데이트
    esp_matter_attr_val_t voltage_val = esp_matter_nullable_uint32(millivolts);
    attribute::update(endpoint_id, PowerSource::Id,
                     PowerSource::Attributes::BatVoltage::Id, &voltage_val);

    ESP_LOGI(TAG, "Battery updated: %d mV, %d %%", millivolts, percentage);
  });
}

void sensor_batt_create_cluster( esp_matter::node_t *node )
{
  // Endpoint 0 (Root Node) 가져오기
  endpoint_t *root_endpoint = endpoint::get(node, 0);
  if (!root_endpoint) {
    ESP_LOGE(TAG, "Failed to get root endpoint");
    return;
  }

  // Power Source 클러스터 직접 생성
  cluster_t *power_source_cluster = cluster::create(
    root_endpoint,
    PowerSource::Id,
    CLUSTER_FLAG_SERVER
  );
  
  if (!power_source_cluster) {
    ESP_LOGE(TAG, "Failed to create power source cluster");
    return;
  }

  // 필수 속성들 추가
  // Status
  esp_matter_attr_val_t status_val = esp_matter_uint8(1); // Active
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::Status::Id, 
                   ATTRIBUTE_FLAG_NONE, status_val);

  // Order
  esp_matter_attr_val_t order_val = esp_matter_uint8(0);
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::Order::Id,
                   ATTRIBUTE_FLAG_NONE, order_val);

  // Description
  esp_matter_attr_val_t desc_val = esp_matter_char_str("Battery", strlen("Battery"));
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::Description::Id,
                   ATTRIBUTE_FLAG_NONE, desc_val);

  // BatPresent
  esp_matter_attr_val_t bat_present_val = esp_matter_bool(true);
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::BatPresent::Id, 
                   ATTRIBUTE_FLAG_NONE, bat_present_val);
  
  // BatQuantity
  esp_matter_attr_val_t bat_quantity_val = esp_matter_uint8(1);
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::BatQuantity::Id,
                   ATTRIBUTE_FLAG_NONE, bat_quantity_val);

  // BatVoltage
  esp_matter_attr_val_t bat_voltage_val = esp_matter_nullable_uint32((uint32_t)(sensor_powers_ctx.batt_mv));
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::BatVoltage::Id, 
                   ATTRIBUTE_FLAG_NULLABLE, bat_voltage_val);
  
  // BatPercentRemaining
  esp_matter_attr_val_t bat_percent_val = esp_matter_nullable_uint8(sensor_powers_ctx.batt_percentage * 2);
  attribute::create(power_source_cluster, 
                   PowerSource::Attributes::BatPercentRemaining::Id,
                   ATTRIBUTE_FLAG_NULLABLE, bat_percent_val);

  // FeatureMap - Battery feature (0x02)
  esp_matter_attr_val_t feature_map_val = esp_matter_uint32(0x02);
  attribute::create(power_source_cluster,
                   PowerSource::Attributes::FeatureMap::Id,
                   ATTRIBUTE_FLAG_NONE, feature_map_val);

  sensor_powers_ctx.powers_endpoint_id = 0;
  
  ESP_LOGI(TAG, "Power Source cluster added to endpoint 0");
}