#include "imu_tools.h"
#include <math.h>

/**
 * @brief Lê os dados do IMU.
 *
 * Esta função lê os dados de aceleração e giroscópio do IMU e os armazena em uma estrutura `IMUData`.
 *
 * @param data Ponteiro para a estrutura `IMUData` onde os dados lidos serão armazenados.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se o ponteiro `data` for NULL ou se ocorrer um erro ao ler os dados.
 */
esp_err_t imu_read_data(IMUData *data)
{
    if (data == NULL)
    {
        return ESP_FAIL;
    }

    esp_err_t result;

    result = imu_get_acceleration_data(&data->accelData);
    switch (result)
    {
    case ESP_OK:
        break;
    default:
        printf("Failed to get acceleration data: %s\n", esp_err_to_name(result));
        return result;
    }

    result = imu_get_gyroscope_data(&data->gyroData);
    switch (result)
    {
    case ESP_OK:
        break;
    default:
        printf("Failed to get gyroscope data: %s\n", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

/**
 * @brief Calcula o quaternion a partir dos dados do IMU.
 *
 * Esta função usa os dados de aceleração e giroscópio para calcular um quaternion que representa a orientação.
 *
 * @param data Ponteiro para a estrutura `IMUData` contendo os dados do IMU.
 * @param quaternion Ponteiro para a estrutura `Quaternion` onde o resultado será armazenado.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se algum dos ponteiros for NULL.
 */
esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion)
{
    if (data == NULL || quaternion == NULL)
    {
        printf("Invalid input for quaternion calculation\n");
        return ESP_FAIL;
    }

    // Normaliza os dados do acelerômetro
    float ax = data->accelData.accel_X;
    float ay = data->accelData.accel_Y;
    float az = data->accelData.accel_Z;

    // Normaliza os dados do giroscópio
    int gx = data->gyroData.rotation_X;
    int gy = data->gyroData.rotation_Y;
    int gz = data->gyroData.rotation_Z;

    // Intervalo de tempo (defina 'dt' com base na taxa de atualização do sensor)
    float dt = 0.01; // Exemplo: taxa de atualização de 10 ms

    // Calcula os ângulos a partir do acelerômetro
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);
    float yaw = 0; // Suposição inicial, será atualizado integrando os dados do giroscópio

    // Integra os dados do giroscópio
    pitch += gx * dt;
    roll += gy * dt;
    yaw += gz * dt;

    // Converte pitch, roll e yaw em quaternion
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    quaternion->w = cr * cp * cy + sr * sp * sy;
    quaternion->x = sr * cp * cy - cr * sp * sy;
    quaternion->y = cr * sp * cy + sr * cp * sy;
    quaternion->z = cr * cp * sy - sr * sp * cy;

    return ESP_OK;
}

/**
 * @brief Calcula os ângulos de Euler a partir do quaternion.
 *
 * Esta função converte um quaternion em ângulos de Euler (roll, pitch, yaw).
 *
 * @param quaternion Ponteiro para a estrutura `Quaternion` que contém os dados do quaternion.
 * @param euler Ponteiro para a estrutura `EulerAngle` onde os resultados serão armazenados.
 * @return ESP_OK em caso de sucesso, ou ESP_FAIL se algum dos ponteiros for NULL.
 */
esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler)
{
    if (quaternion == NULL || euler == NULL)
    {
        printf("Invalid input for Euler angle calculation\n");
        return ESP_FAIL;
    }

    // Recebe os valores do quaternion
    float qw = quaternion->w;
    float qx = quaternion->x;
    float qy = quaternion->y;
    float qz = quaternion->z;

    // Armazena os ângulos de Euler
    euler->roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    euler->pitch = asin(2 * (qw * qy - qz * qx));
    euler->yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    return ESP_OK;
}
