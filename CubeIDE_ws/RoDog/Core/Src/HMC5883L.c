#include "HMC5883L.h"
#include "math.h"


void HMC5883L_readRaw(I2C_HandleTypeDef * hi2c , int16_t magxyz[]){

	  uint8_t dataBuffer[6];

	  // Request data from the HMC5883L
	  uint8_t regAddress = 0x03; // Data register (X MSB)
	  HAL_I2C_Master_Transmit(hi2c, HMC5883L_ADDRESS << 1, &regAddress, sizeof(regAddress), HAL_MAX_DELAY);

	  // Read data from the HMC5883L
	  HAL_I2C_Master_Receive(hi2c , HMC5883L_ADDRESS << 1, dataBuffer, sizeof(dataBuffer), HAL_MAX_DELAY);

	  // Extract the data (assuming data is in two's complement)
	  magxyz[0]= (int16_t)(((dataBuffer[0] << 8) | dataBuffer[1]));
	  magxyz[1]= (int16_t)(((dataBuffer[4] << 8) | dataBuffer[5]));
	  magxyz[2]= (int16_t)(((dataBuffer[2] << 8) | dataBuffer[3]));
}


void HMC5883L_setOffset(int16_t xo , int16_t yo)
{
	magn_xOffset = xo;
	magn_yOffset = yo;
}

void HMC5883L_setRange(hmc5883l_range_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    mgPerDigit = 0.0073;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    mgPerDigit = 0.092f ;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    mgPerDigit = 0.122f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    mgPerDigit = 0.152f;
	    break;

	case HMC5883L_RANGE_4GA:
	    mgPerDigit = 0.227f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    mgPerDigit = 0.256f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    mgPerDigit = 0.303f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    mgPerDigit = 0.435f;
	    break;

	default:
	    break;
    }

    HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t HMC5883L_getRange(void)
{
    return (hmc5883l_range_t)((HMC5883L_readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L_setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    HMC5883L_writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t HMC5883L_getMeasurementMode(void)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void HMC5883L_setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t HMC5883L_getDataRate(void)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void HMC5883L_setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    HMC5883L_writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L_getSamples(void)
{
    uint8_t value;

    value = HMC5883L_readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

// Write byte to register
void HMC5883L_writeRegister8(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&I2C, HMC5883L_DEFAULT_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT  , &value ,1,500);
}

// Read byte to register
uint8_t HMC5883L_fastRegister8(uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&I2C,HMC5883L_DEFAULT_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT , &value, 1, 500);
    return value;
}

// Read byte from register
uint8_t HMC5883L_readRegister8(uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&I2C,HMC5883L_DEFAULT_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 500);
    return value;
}

// Read word from register
int16_t HMC5883L_readRegister16(uint8_t reg)
{
    uint8_t value[2];
    HAL_I2C_Mem_Read(&I2C,HMC5883L_DEFAULT_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, 2, 500);
    return (int16_t)(value[0] << 8) | value[1];
}
