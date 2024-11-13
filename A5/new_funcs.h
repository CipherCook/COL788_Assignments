#ifndef __NEW_FUNCS_H
#define __NEW_FUNCS_H

#ifdef __cplusplus
 extern "C" {
#endif



#define MAX30102_ADDRESS  0x57 // I2C address ==== of MAX30102

 // Register Addresses
 #define MAX30105_INTSTAT1      0x00  // Interrupt Status 1
 #define MAX30105_INTSTAT2      0x01  // Interrupt Status 2
 #define MAX30105_INTENABLE1    0x02  // Interrupt Enable 1
 #define MAX30105_INTENABLE2    0x03  // Interrupt Enable 2
 #define MAX30105_DIETEMPCONFIG 0x21  // Die Temperature Config
 #define MAX30105_TEMP_INTEGER  0x1F  // Temperature Integer Part
 #define MAX30105_TEMP_FRACTION 0x20  // Temperature Fractional Part

 // Bit Definitions
 #define MAX30105_INT_DIE_TEMP_RDY_ENABLE 0x02  // Bit 1
#define MAX30102_INT_A_FULL_ENABLE 0x80 //Bit 7
void initialize_filters();
float calculate_SpO2(float rred, float rir);
void Transmit_UART(const char *msg,UART_HandleTypeDef* huart2);
void MAX30102_Init(I2C_HandleTypeDef* hi2c1);
void MAX30102_ReadRawData(uint32_t* redLED, uint32_t* irLED,I2C_HandleTypeDef* hi2c1,UART_HandleTypeDef* huart2);
void MAX30102_EnableTemperatureInterrupt(I2C_HandleTypeDef* hi2c1,UART_HandleTypeDef* huart2);
void MAX30102_StartTemperatureMeasurement(I2C_HandleTypeDef* hi2c1);
float MAX30102_ReadTemperature(I2C_HandleTypeDef* hi2c1);
int Detect_Finger(uint32_t redLED);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
