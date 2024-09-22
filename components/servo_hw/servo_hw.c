#include "servo_hw.h"
#include "driver/ledc.h"

ServoConfig *global_servo_config[8] = {NULL};
bool stop_pwm[8];

static uint8_t servo_index; //variavel para indicar qual servo quero configurar

static uint32_t search_servo_index(uint32_t gpio) {
    for (int i = 0; i < 8; i++) {
        if (global_servo_config[i]->servo_pin == gpio) {
            return i;
            break;
        }
    }
    return 9; //Erro caso o gpio nao tenha servo atrelado
}

esp_err_t hw_servo_init(uint8_t gpio_num) {
    servo_index = search_servo_index(gpio_num);
    if (servo_index == 9) return ESP_FAIL;

    stop_pwm[servo_index] = false;

    esp_err_t ret;

    ledc_timer_config_t ledc_timer = {
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = global_servo_config[servo_index]->freq,
        .speed_mode = global_servo_config[servo_index]->speed_mode,
        .timer_num = global_servo_config[servo_index]->timer_number
    };

    ret = (global_servo_config[1] == NULL) ? ledc_timer_config(&ledc_timer) : ESP_OK; //Medida para configurar o timer somente uma unica vez
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;

    ledc_channel_config_t ledc_ch = {
        .intr_type  = LEDC_INTR_DISABLE,
        .channel    = global_servo_config[servo_index]->channel_number,
        .duty       = global_servo_config[servo_index]->duty,
        .gpio_num   = gpio_num,
        .speed_mode = global_servo_config[servo_index]->speed_mode,
        .timer_sel  = global_servo_config[servo_index]->timer_number,
        .hpoint     = 0
    };

    ret = ledc_channel_config(&ledc_ch);
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND ;

    return ESP_OK;
}

esp_err_t hw_servo_set_pulse_width(uint8_t gpio_num, uint32_t pulse_width_us) {
    servo_index = search_servo_index(gpio_num);
    if (servo_index == 9) return ESP_FAIL;
    if (pulse_width_us > 511 || pulse_width_us < 102) return ESP_FAIL; //IMPORTANTE RESPEITAR OS LIMITES DA LARGURA DO PULSO
    global_servo_config[servo_index]->duty = pulse_width_us;

    return ESP_OK;
}

esp_err_t hw_servo_deinit(uint8_t gpio_num) {
    servo_index = search_servo_index(gpio_num);
    if (servo_index == 9) return ESP_FAIL;
    stop_pwm[servo_index] = true;
    esp_err_t ret;
    ret = ledc_set_duty(global_servo_config[servo_index]->speed_mode, (ledc_channel_t) global_servo_config[servo_index]->channel_number, 0);
    ret |= ledc_update_duty(global_servo_config[servo_index]->speed_mode, (ledc_channel_t) global_servo_config[servo_index]->channel_number);
    //ret |= ledc_timer_rst(global_servo_config.speed_mode, global_servo_config.timer_number);
    if (ret != ESP_OK) return ESP_FAIL;

    return ESP_OK;
}