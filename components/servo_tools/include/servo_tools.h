#ifndef SERVO_TOOLS_H
#define SERVO_TOOLS_H

#include "esp_err.h"
#include "servo_hw.h"

esp_err_t servo_init(ServoConfig *config);
esp_err_t servo_set_angle(ServoConfig *config, ServoAngle angle);
esp_err_t servo_get_angle(const ServoConfig *config, ServoAngle *angle);

#endif //SERVO_TOOLS_H