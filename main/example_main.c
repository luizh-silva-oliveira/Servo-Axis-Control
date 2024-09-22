#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "servo_tools.h"
#include "servo_hw.h"
#include "imu_tools.h"
#include "sensor_imu.h"
#include <math.h>

#define MPU6050_ADDR 0x68	// Endereço do sensor MPU6050
#define SDA_PIN GPIO_NUM_21 // Pino GPIO para SDA
#define SCL_PIN GPIO_NUM_22 // Pino GPIO para SCL

static const char *TAG_SERVO_TOOLS = "Angulo servo roll";
static const char *TAG_SERVO_TOOLS_2 = "Angulo servo pitch";

IMUData imu;
Quaternion quaternion;
EulerAngle euler;
AccelerationData tempAccelerationMeasures;

ServoConfig servo_config = {
    .max_angle = 180,
    .freq = 200,
    .min_width_us = 500,
    .max_width_us = 2500,
    .timer_number = LEDC_TIMER_0,
    .channel_number = LEDC_CHANNEL_0,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .servo_pin = 19
};

ServoConfig servo_config_2 = {
    .max_angle = 180,
    .freq = 200,
    .min_width_us = 500,
    .max_width_us = 2500,
    .timer_number = LEDC_TIMER_0,
    .channel_number = LEDC_CHANNEL_1,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .servo_pin = 18
};

/*Estados do sistema*/
typedef enum {
    GET_IMU_DATA = 0, CALCULATE_QUATERNIONS, CALCULATE_EULER_ANGLES, MOVE_SERVO, GET_ANGLE, OUTPUT_VALUES
} States;

static float get_imu_roll(float x) {
  if(x < -1) return 0;
  if(x > 1) return 180;
  return ((x + 1)*180) / 2;
}

static float get_imu_pitch(float y) {
  if(y < -1) return 0;
  if(y > 1) return 180;
  return ((y + 1)*180) / 2;
}

void app_main() {
  esp_err_t ret;

  do {
    ret = imu_init(MPU6050_ADDR, SDA_PIN, SCL_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));
    ret |= servo_init(&servo_config);
    ret |= servo_init(&servo_config_2);
  } while (ret != ESP_OK);

  static States state = GET_IMU_DATA;
  static const int quantStates = 6; //variavel que informa a quantidade de estados
  static bool has_imu_data_change = true; //variavel que informa se os dados do sensor alteraram.
  static bool first_measure = true;

  ServoAngle roll;
  ServoAngle pitch;

  while (true){
    for (int i = 0; i < quantStates; i++) {
      switch (state) {

      case GET_IMU_DATA:
        ret = imu_read_data(&imu);
        if(first_measure) {
          ret = imu_get_acceleration_data(&tempAccelerationMeasures); //struct temporario para construir a logica de verificar se o sensor se movimentou
          first_measure = false;
        }
        //logica para verificar se o sensor movimentou 0.03 foi o valor que decidi usar para filtrar o ruido
        if(!first_measure && (fabs((imu.accelData.accel_X - tempAccelerationMeasures.accel_X)) > 0.06 ||
          fabs((imu.accelData.accel_Y - tempAccelerationMeasures.accel_Y)) > 0.06 ||
          fabs((imu.accelData.accel_Z - tempAccelerationMeasures.accel_Z)) > 0.06
        )){
          has_imu_data_change = true;
          tempAccelerationMeasures.accel_X = imu.accelData.accel_X;
          tempAccelerationMeasures.accel_Y = imu.accelData.accel_Y;
          tempAccelerationMeasures.accel_Z = imu.accelData.accel_Z;
        }
        else has_imu_data_change = false;

        if (ret == ESP_OK) state = CALCULATE_QUATERNIONS;
        break;

      case CALCULATE_QUATERNIONS:
        ret = imu_calculate_quaternion(&imu, &quaternion);
        if (ret == ESP_OK) state = CALCULATE_EULER_ANGLES;
        break;

      case CALCULATE_EULER_ANGLES:
        ret = imu_calculate_euler_angles(&quaternion, &euler);
        //logica para solicitar movimentação do servo somente se o MPU se moveu
        if (ret == ESP_OK && has_imu_data_change) state = MOVE_SERVO;
        else if (ret == ESP_OK) state = GET_IMU_DATA;
        break;

      case MOVE_SERVO:
        roll.angle = get_imu_roll(imu.accelData.accel_X);
        pitch.angle = get_imu_pitch(imu.accelData.accel_Y);
        ret = servo_set_angle(&servo_config, roll);
        ret |= servo_set_angle(&servo_config_2, pitch);
        if (ret == ESP_OK) state = GET_ANGLE;
        break;
      
      case GET_ANGLE:
        ret = servo_get_angle(&servo_config, &roll);
        ret |= servo_get_angle(&servo_config_2, &pitch);
        if (ret == ESP_OK) state = OUTPUT_VALUES;
        break;

      case OUTPUT_VALUES:
        ESP_LOGI(TAG_SERVO_TOOLS, "%.2f\n", roll.angle);
        ESP_LOGI(TAG_SERVO_TOOLS_2, "%.2f\n", pitch.angle);
        state = GET_IMU_DATA;
        break;
      }
    }
  }
}