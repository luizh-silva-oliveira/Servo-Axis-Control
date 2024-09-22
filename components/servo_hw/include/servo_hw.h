#ifndef SERVO_HW
#define SERVO_HW

#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

typedef struct {
    uint16_t max_angle;        //Angulo máximo do servo
    uint16_t min_width_us;     //Largura do pulso para angulo minimo, geralmente é 500us
    uint16_t max_width_us;     //Largura do pulso para angulo minimo, geralmente é 2500us
    uint32_t freq;             //Frequencia PWM
    ledc_timer_t timer_number; 
    uint8_t channel_number;    //Canal do ledc
    ledc_mode_t speed_mode;    //Velocidade do canal ledc
    uint32_t duty;             //duty cycle
    uint8_t servo_pin;          //Pino do servo
} ServoConfig;

typedef struct {
    float angle;
} ServoAngle;

extern ServoConfig *global_servo_config[8];
extern bool stop_pwm[8];

esp_err_t hw_servo_init(uint8_t gpio_num);
esp_err_t hw_servo_set_pulse_width(uint8_t gpio_num, uint32_t pulse_width_us);
esp_err_t hw_servo_deinit(uint8_t gpio_num);

#endif // SERVOR_HW