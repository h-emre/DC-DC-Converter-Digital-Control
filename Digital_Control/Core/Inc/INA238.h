#ifndef INA238_H
#define INA238_H
#include "main.h"
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr;
    uint16_t average;
    uint16_t busConvTime;
    uint16_t shuntConvTime;
    uint16_t mode;
    float rShuntValue;
    float iMax;
    uint16_t calibrationValue;
    float shuntVoltageLSB;
    float currentLSB;
    float powerLSB;
    float busVoltageLSB;
} INA238_HandleTypeDef;
#define CONFIG_REG           0x00
#define CALIBRATION_REG      0x05
#define SHUNT_VOLTAGE_REG    0x01
#define BUS_VOLTAGE_REG      0x02
#define CURRENT_REG          0x04
#define POWER_REG            0x03
#define INA238_RESET         0x8000
HAL_StatusTypeDef INA238_Init(INA238_HandleTypeDef *hINA238, I2C_HandleTypeDef *hi2c,
		uint16_t addr, uint16_t average, uint16_t busConvTime, uint16_t shuntConvTime,
		uint16_t mode, float rShuntValue, float iMax);
HAL_StatusTypeDef INA238_Reset(INA238_HandleTypeDef *hINA238);
HAL_StatusTypeDef INA238_Config(INA238_HandleTypeDef *hINA238);
HAL_StatusTypeDef INA238_ReadShuntVoltage(INA238_HandleTypeDef *hINA238, float *voltage);
HAL_StatusTypeDef INA238_ReadCurrent(INA238_HandleTypeDef *hINA238, float *current);
HAL_StatusTypeDef INA238_ReadPower(INA238_HandleTypeDef *hINA238, float *power);
HAL_StatusTypeDef INA238_ReadBusVoltage(INA238_HandleTypeDef *hINA238, float *voltage);
#endif


