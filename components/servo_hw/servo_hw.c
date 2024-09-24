#include "servo_hw.h"
#include "driver/ledc.h"

ServoConfig *global_servo_config[8] = {NULL};
bool stop_pwm[8];

static uint8_t servo_index; // Variável para indicar qual servo configurar

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
 * Esta função configura o timer e o canal PWM para um servo motor conectado ao GPIO fornecido.
 *
 * @param gpio_num Pino GPIO ao qual o servo está conectado.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se o GPIO não tiver um servo associado.
 */
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

    ret = (global_servo_config[1] == NULL) ? ledc_timer_config(&ledc_timer) : ESP_OK; // Configura o timer somente uma vez
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
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;

    return ESP_OK;
}

/**
 * @brief Define a largura do pulso para um servo motor.
 *
 * Esta função ajusta a largura do pulso do servo motor conectado ao GPIO fornecido.
 * A largura do pulso deve estar entre 102 e 511 microsegundos.
 *
 * @param gpio_num Pino GPIO ao qual o servo está conectado.
 * @param pulse_width_us Largura do pulso em microsegundos.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se o GPIO não tiver um servo associado
 *         ou se a largura do pulso estiver fora dos limites permitidos.
 */
esp_err_t hw_servo_set_pulse_width(uint8_t gpio_num, uint32_t pulse_width_us) {
    servo_index = search_servo_index(gpio_num);
    if (servo_index == 9) return ESP_FAIL;
    if (pulse_width_us > 511 || pulse_width_us < 102) return ESP_FAIL; // Respeitar os limites da largura do pulso
    global_servo_config[servo_index]->duty = pulse_width_us;

    return ESP_OK;
}

/**
 * @brief Desinstala um servo motor.
 *
 * Esta função desabilita o PWM do servo motor conectado ao GPIO fornecido,
 * parando a emissão de pulsos.
 *
 * @param gpio_num Pino GPIO ao qual o servo está conectado.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se o GPIO não tiver um servo associado.
 */
esp_err_t hw_servo_deinit(uint8_t gpio_num) {
    servo_index = search_servo_index(gpio_num);
    if (servo_index == 9) return ESP_FAIL;
    stop_pwm[servo_index] = true;
    esp_err_t ret;
    ret = ledc_set_duty(global_servo_config[servo_index]->speed_mode, (ledc_channel_t) global_servo_config[servo_index]->channel_number, 0);
    ret |= ledc_update_duty(global_servo_config[servo_index]->speed_mode, (ledc_channel_t) global_servo_config[servo_index]->channel_number);
    if (ret != ESP_OK) return ESP_FAIL;

    return ESP_OK;
}
