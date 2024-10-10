#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <Fusion.h>

// Definições de pinos e endereço I2C
#define MPU_ADDRESS      0x68
#define I2C_SDA_GPIO     4
#define I2C_SCL_GPIO     5
#define SAMPLE_PERIOD    (0.012f) // Ajuste leve no período de amostragem

QueueHandle_t dataQueue;

typedef struct {
    int axis;
    int value;
} ImuData;

// Função de inicialização do MPU6050
static void init_mpu6050() {
    i2c_init(i2c_default, 350 * 1000); // Ajuste leve na taxa de I2C
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Reset do MPU6050
    uint8_t buf[] = {0x6B, 0x00}; // Saída do modo de suspensão
    int result = i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
    if (result != 2) {
        printf("Erro ao configurar o MPU6050\n");
    }
}

// Função para leitura de dados brutos do MPU6050
static void read_mpu6050_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg = 0x3B;

    // Lê aceleração
    if (i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true) != 1) {
        printf("Erro na leitura de aceleração\n");
        return;
    }
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Lê giroscópio
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Lê temperatura
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

// Tarefa para enviar dados do MPU6050
void data_transmit_task(void *params) {
    ImuData data;
    while (1) {
        if (xQueueReceive(dataQueue, &data, portMAX_DELAY)) {
            printf("Eixo: %d, Valor: %d\n", data.axis, data.value); // Simples print no lugar do envio
        }
    }
}

// Tarefa de leitura e processamento do MPU6050
void mpu6050_processing_task(void *params) {
    init_mpu6050();

    int16_t accel[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while (1) {
        read_mpu6050_raw(accel, gyro, &temp);

        // Converte valores brutos
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 128.0f, // Ajuste leve no fator de conversão
            .axis.y = gyro[1] / 128.0f,
            .axis.z = gyro[2] / 128.0f,
        };
        FusionVector accelerometer = {
            .axis.x = accel[0] / 16500.0f, // Ajuste leve no fator de conversão
            .axis.y = accel[1] / 16500.0f,
            .axis.z = accel[2] / 16500.0f,
        };

        // Atualiza filtro de fusão
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Cria e envia os dados para a fila
        ImuData data;

        data.axis = 0; // Yaw
        data.value = (int)euler.angle.yaw;
        xQueueSend(dataQueue, &data, portMAX_DELAY);

        data.axis = 1; // Roll
        data.value = (int)euler.angle.roll;
        xQueueSend(dataQueue, &data, portMAX_DELAY);

        // Simulação de clique para aceleração brusca
        if (accelerometer.axis.y > 1.4f) { // Ajuste leve na sensibilidade do clique
            data.axis = 2;
            data.value = 1;
            xQueueSend(dataQueue, &data, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(12)); // Ajuste leve no atraso entre leituras
    }
}

// Função principal
int main() {
    stdio_init_all();
    dataQueue = xQueueCreate(8, sizeof(ImuData)); // Ajuste na quantidade de itens da fila

    xTaskCreate(mpu6050_processing_task, "MPU6050 Process", 4096, NULL, 1, NULL);
    xTaskCreate(data_transmit_task, "Data Transmit", 1024, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true) {
        // Loop principal
    }
}
