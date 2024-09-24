#include "servo_tools.h"
#include "servo_hw.h"
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

static uint8_t servo_index;

ServoAngle servo_angle = {
    .angle = 0
};

static const uint32_t global_full_duty = (1 << LEDC_TIMER_10_BIT) - 1;

/**
 * @brief Calcula o duty cycle para um ângulo específico.
 *
 * Esta função converte um ângulo em um duty cycle adequado para o servo motor.
 *
 * @param angle O ângulo desejado em graus.
 */
static void calculate_duty(float angle) {
    float angle_us = angle / global_servo_config[servo_index]->max_angle * (global_servo_config[servo_index]->max_width_us - global_servo_config[servo_index]->min_width_us) + global_servo_config[servo_index]->min_width_us;
    uint32_t duty = (uint32_t)((float) global_full_duty * (angle_us) * global_servo_config[servo_index]->freq / (1000000.0f));
    hw_servo_set_pulse_width(global_servo_config[servo_index]->servo_pin, duty);
}

/**
 * @brief Converte um duty cycle em um ângulo.
 *
 * Esta função calcula o ângulo correspondente a um duty cycle fornecido.
 *
 * @param duty O duty cycle atual do servo.
 * @return O ângulo correspondente em graus.
 */
static float calculate_angle(uint32_t duty) {
    float angle_us = (float)duty * 1000000.0f / (float)global_full_duty / (float)global_servo_config[servo_index]->freq;
    angle_us -= global_servo_config[servo_index]->min_width_us;
    angle_us = angle_us < 0.0f ? 0.0f : angle_us;
    float angle = angle_us * global_servo_config[servo_index]->max_angle / (global_servo_config[servo_index]->max_width_us - global_servo_config[servo_index]->min_width_us);
    return angle;
}

/**
 * @brief Pesquisa o índice do servo associado a um pino GPIO.
 *
 * Esta função percorre a configuração global de servos e retorna o índice do servo
 * que está associado ao pino GPIO fornecido.
 *
 * @param gpio Pino GPIO que se deseja pesquisar.
 * @return Índice do servo se encontrado, ou 9 se não houver servo atrelado ao GPIO.
 */
static uint32_t search_servo_index(uint32_t gpio) {
    for (int i = 0; i < 8; i++) {
        if (global_servo_config[i]->servo_pin == gpio) {
            return i;
        }
    }
    return 9; // Erro caso o GPIO não tenha servo atrelado
}

/**
 * @brief Inicializa um servo motor.
 *
 * Esta função configura um servo motor com as configurações fornecidas, 
 * inicializa o PWM e realiza uma movimentação inicial de 0 a 180 graus e volta.
 *
 * @param config Ponteiro para a configuração do servo.
 * @return ESP_OK em caso de sucesso, ou ESP_ERR_NOT_FOUND se não for possível encontrar o servo.
 */
esp_err_t servo_init(ServoConfig *config) {
    esp_err_t ret;
    for (int i = 0; i < 8; i++) {
        if (global_servo_config[i] == NULL) {
            servo_index = i;
            break;
        }
    }
    
    global_servo_config[servo_index] = config;
    calculate_duty(0); // Inicia o servo em 0 graus
    hw_servo_init(config->servo_pin); // Iniciando PWM na GPIO

    // Inicia no ângulo 0
    ret = servo_set_angle(global_servo_config[servo_index], servo_angle);
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Rotaciona até 180 graus
    servo_angle.angle = 180;
    ret = servo_set_angle(global_servo_config[servo_index], servo_angle);
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Rotaciona de volta para 0 graus
    servo_angle.angle = 0;
    ret = servo_set_angle(global_servo_config[servo_index], servo_angle);
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;

    return ESP_OK;
}

/**
 * @brief Define a posição do servo motor.
 *
 * Esta função ajusta o ângulo do servo motor para o valor desejado.
 *
 * @param config Ponteiro para a configuração do servo.
 * @param angle Estrutura contendo o ângulo desejado.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se houver erro ao ajustar o ângulo.
 */
esp_err_t servo_set_angle(ServoConfig *config, ServoAngle angle) {
    servo_index = search_servo_index(config->servo_pin);
    if (servo_index == 9) return ESP_FAIL;
    esp_err_t ret;
    calculate_duty(angle.angle);
    uint32_t duty = config->duty;
    if (stop_pwm[servo_index]) return ESP_OK; // Caso o PWM tenha sido desligado, não atualizar duty cycle
    ret = ledc_set_duty(config->speed_mode, (ledc_channel_t)config->channel_number, duty);
    ret |= ledc_update_duty(config->speed_mode, (ledc_channel_t)config->channel_number);

    if (ret != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

/**
 * @brief Obtém a posição atual do servo motor.
 *
 * Esta função retorna o ângulo atual do servo motor baseado no duty cycle.
 *
 * @param config Ponteiro para a configuração do servo.
 * @param angle Ponteiro para a estrutura onde o ângulo atual será armazenado.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t servo_get_angle(const ServoConfig *config, ServoAngle *angle) {
    uint32_t duty = ledc_get_duty(config->speed_mode, config->channel_number);
    float a = calculate_angle(duty);
    angle->angle = a;
    return ESP_OK;
}
