#include "main.h"
#include "new_funcs.h"
#include <stdbool.h>
#include "filter.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern const float rThreshold;

extern const float kLowPassCutoff;   // 5 hz
extern const float kHighPassCutoff;  // 0.5 hz
extern const float kSamplingFrequency;
extern const float kEdgeThreshold;

extern const unsigned long kFingerThreshold;
extern const unsigned int kFingerCooldownMs;




extern const float decayRate;
extern const float thrRate;
extern const int minDiff;
extern const uint32_t fingerThreshold;  // threshold for detecting a finger on the sensor

extern float maxValue;
extern float minValue;
extern float threshold;
extern long lastHeartbeat;
extern float lastValue;
extern int fingerDetected;  // variable to track whether the finger is detected

extern float last_diff;
extern bool crossed;
extern long crossed_time;
/* variables */
extern LowPassFilter low_pass_filter_red;
extern LowPassFilter low_pass_filter_ir;
extern HighPassFilter high_pass_filter;
extern 	LowPassFilter low_pass_filter;
extern 	Differentiator differentiator;
extern 	MovingAverageFilter moving_avg_filter;
extern 	MovingAverageFilter moving_avg_bpm;
extern 	MovingAverageFilter moving_avg_spo2;
/* statistic Variables */
extern 	MinMaxAvgStatistic stat_red;
extern 	MinMaxAvgStatistic stat_ir;


/* finger Detection */
extern 	long finger_timestamp;
extern 	bool finger_detected;

void initialize_filters() {
	HighPassFilter_Init(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
	LowPassFilter_Init(&low_pass_filter_red, kLowPassCutoff, kSamplingFrequency);
	LowPassFilter_Init(&low_pass_filter_ir, kLowPassCutoff, kSamplingFrequency);
	Differentiator_Init(&differentiator, kSamplingFrequency);
	MovingAverageFilter_Init(&moving_avg_filter);
	MovingAverageFilter_Init(&moving_avg_bpm);
	MovingAverageFilter_Init(&moving_avg_spo2);
}


/* SpO2 Calibration Coefficients */
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

/* Pulse Oximetry SpO2 Calculation */
float calculate_SpO2(float rred, float rir) {
	float r = rred / rir;
	float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
	return spo2;
}


void Transmit_UART(const char *msg,UART_HandleTypeDef* huart2) {
			HAL_UART_Transmit(huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}


void MAX30102_Init(I2C_HandleTypeDef* hi2c1) {
	uint8_t configData[2];

	// modes Configuration Register
//		0,1 == adress, value
	configData[0] = 0x09;  // select MODE_CONFIG register
	configData[1] = 0x03;  // set SpO2 mode (0x03== SpO2, 0x02 == heart rate only)
//		hi2cl handler for i2c
//		adress shifterd by 1 only 7 bit sent (required)
//		2 indicate only 2 bytes to send
//		HAL_MAX_DELAY allows operation to block until successful trastmittion
	HAL_I2C_Master_Transmit(hi2c1, MAX30102_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);

	// Set the LED pulse amplitude (this is an example, values need to be set properly)
	configData[0] = 0x0C;  // LED1_PULSE_AMPLITUDE ka register
	configData[1] = 0x1F;  // set the pulse amplitude for LED1 (Red LED we can see naked eyes)
	//		hi2cl handler for i2c
	//		adreess shifterd by 1 only 7 bit sent (required)
	//		2 indicate only 2 bytes to send
	//		HAL_MAX_DELAY allows operation to block until successful trastmittion
	HAL_I2C_Master_Transmit(hi2c1, MAX30102_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);

	// set LED2 (IR LED) pulse Amplitude
	configData[0] = 0x0D;  //register of ir led
	configData[1] = 0x1F;  // infra red LED
	//		hi2cl handler for i2c
	//		adreess shifterd by 1 only 7 bit sent (required)
	//		2 indicate only 2 bytes to send
	//		HAL_MAX_DELAY allows operation to block until successful trastmittion
	HAL_I2C_Master_Transmit(hi2c1, MAX30102_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);

	// Enable the sensor or configure further registers (such as SpO2 config) as needed
}


void MAX30102_ReadRawData(uint32_t* redLED, uint32_t* irLED,I2C_HandleTypeDef* hi2c1,UART_HandleTypeDef* huart2) {
	uint8_t rawData[6];  // 6 byte array as MAX30102 outputs 3 bytes for each LED data
	char uartBuf[50];

	// reading from FIFO register of MAX30102
	if (HAL_I2C_Mem_Read(hi2c1, MAX30102_ADDRESS << 1, 0x07, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY) != HAL_OK) {
		sprintf(uartBuf, "I2C read error\r\n");
		HAL_UART_Transmit(huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);  // sending error message via UART
		Error_Handler();  //  keep printing this message === so  avoid halting the program
	}

	// combine 3 bytes for each LED (Red and IR)
	*redLED = ((uint32_t)rawData[0] << 16) | ((uint32_t)rawData[1] << 8) | rawData[2];
	*irLED = ((uint32_t)rawData[3] << 16) | ((uint32_t)rawData[4] << 8) | rawData[5];
}




//  to enable temperature interrupt on MAX30102
void MAX30102_EnableTemperatureInterrupt(I2C_HandleTypeDef* hi2c1,UART_HandleTypeDef* huart2) {
    uint8_t data = MAX30105_INT_DIE_TEMP_RDY_ENABLE;
    // Write to INT_ENABLE_2 register (0x03)


    HAL_I2C_Mem_Write(hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTENABLE2,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

//    data = MAX30102_INT_A_FULL_ENABLE;
//    HAL_I2C_Mem_Write(hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTENABLE1,
//                          I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    char uartBuf[50];
    sprintf(uartBuf, "Interrupt enabled...\r\n");
    HAL_UART_Transmit(huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

}


// function to start temperature measurement on MAX30102
void MAX30102_StartTemperatureMeasurement(I2C_HandleTypeDef* hi2c1) {
    uint8_t data = 0x01;  // Start temperature measurement

    // Write to TEMP_CONFIG register (0x21)
    HAL_I2C_Mem_Write(hi2c1, MAX30102_ADDRESS << 1, MAX30105_DIETEMPCONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}


// function reading temperature after interrupt signals it's ready
float MAX30102_ReadTemperature(I2C_HandleTypeDef* hi2c1) {
    uint8_t tempInt = 0, tempFrac = 0;

    // Read integer part of temperature
    HAL_I2C_Mem_Read(hi2c1, MAX30102_ADDRESS << 1, MAX30105_TEMP_INTEGER,
                     I2C_MEMADD_SIZE_8BIT, &tempInt, 1, HAL_MAX_DELAY);
    // Read fractional part of temperature
    HAL_I2C_Mem_Read(hi2c1, MAX30102_ADDRESS << 1, MAX30105_TEMP_FRACTION,
                     I2C_MEMADD_SIZE_8BIT, &tempFrac, 1, HAL_MAX_DELAY);

    // Calculate temperature in degrees Celsius
    return (float)tempInt + ((float)tempFrac * 0.0625);
}


int Detect_Finger(uint32_t redLED) {
	return (redLED > fingerThreshold);
}


