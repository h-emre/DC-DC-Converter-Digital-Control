#include "INA238.h"
HAL_StatusTypeDef INA238_Init(INA238_HandleTypeDef *hINA238,
		I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t average,
		uint16_t busConvTime, uint16_t shuntConvTime, uint16_t mode,
		float rShuntValue, float iMax){
    hINA238->hi2c = hi2c;
    hINA238->addr = addr;
    hINA238->average = average;
    hINA238->busConvTime = busConvTime;
    hINA238->shuntConvTime = shuntConvTime;
    hINA238->mode = mode;
    hINA238->rShuntValue = rShuntValue;
    hINA238->iMax = iMax;
    // Kalibrasyon değerinin hesaplanması
    hINA238->calibrationValue = (uint16_t)(0.00512 /
    		(hINA238->rShuntValue * (1 << hINA238->average)));
    if (INA238_Reset(hINA238) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef INA238_Reset(INA238_HandleTypeDef *hINA238)
{
    uint16_t reset = INA238_RESET;
    uint8_t data[2];
    data[0] = reset >> 8;
    data[1] = reset & 0xff;
    if (HAL_I2C_Mem_Write(hINA238->hi2c, hINA238->addr,
    		CONFIG_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef INA238_Config(INA238_HandleTypeDef *hINA238)
{
    uint16_t config = (hINA238->average << 9) |
    		(hINA238->busConvTime << 6) |
			(hINA238->shuntConvTime << 3) | hINA238->mode;
    uint8_t data[2];
    data[0] = config >> 8;
    data[1] = config & 0xff;
    // Konfigürasyon kaydını yazma
    if (HAL_I2C_Mem_Write(hINA238->hi2c, hINA238->addr,
    		CONFIG_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Kalibrasyon kaydını yazma
    data[0] = hINA238->calibrationValue >> 8;
    data[1] = hINA238->calibrationValue & 0xff;
    if (HAL_I2C_Mem_Write(hINA238->hi2c, hINA238->addr,
    		CALIBRATION_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef INA238_ReadShuntVoltage(INA238_HandleTypeDef *hINA238,
		float *voltage)
{
    uint16_t shuntVoltageRaw;
    uint8_t data[2];
    // Shunt voltaj kaydını okuma
    if (HAL_I2C_Mem_Read(hINA238->hi2c, hINA238->addr,
    		SHUNT_VOLTAGE_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    shuntVoltageRaw = (data[0] << 8) | data[1];
    // Shunt voltajını volt cinsinden hesaplama
    *voltage = (float)shuntVoltageRaw * hINA238->shuntVoltageLSB;
    return HAL_OK;
}
HAL_StatusTypeDef INA238_ReadCurrent(INA238_HandleTypeDef *hINA238,
		float *current)
{
    uint16_t currentRaw;
    uint8_t data[2];
    // Akım kaydını okuma
    if (HAL_I2C_Mem_Read(hINA238->hi2c, hINA238->addr,
    		CURRENT_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    currentRaw = (data[0] << 8) | data[1];
    // Amper cinsinden akımı hesaplama
    *current = (float)currentRaw * hINA238->currentLSB;

    return HAL_OK;
}
HAL_StatusTypeDef INA238_ReadPower(INA238_HandleTypeDef *hINA238,
		float *power)
{
    uint16_t powerRaw;
    uint8_t data[2];
    // Güç kaydını okuma
    if (HAL_I2C_Mem_Read(hINA238->hi2c, hINA238->addr,
    		POWER_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    powerRaw = (data[0] << 8) | data[1];

    // Watt cinsinden gücü hesaplama
    *power = (float)powerRaw * hINA238->powerLSB;

    return HAL_OK;
}
HAL_StatusTypeDef INA238_ReadBusVoltage(INA238_HandleTypeDef *hINA238,
		float *voltage)
{
    uint16_t busVoltageRaw;
    uint8_t data[2];
    // Bus voltaj kaydını okuma
    if (HAL_I2C_Mem_Read(hINA238->hi2c, hINA238->addr,
    		BUS_VOLTAGE_REG, I2C_MEMADD_SIZE_16BIT, data, 2, 1000) != HAL_OK)
    {
        return HAL_ERROR;
    }
    busVoltageRaw = (data[0] << 8) | data[1];
    // Volt cinsinden bus voltajını hesaplama
    *voltage = (float)busVoltageRaw * hINA238->busVoltageLSB;
    return HAL_OK;
}
