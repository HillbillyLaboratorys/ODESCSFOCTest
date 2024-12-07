#include <SimpleFOC.h>
#include <Arduino.h>
//#include "SPI.h"

#define HSE_VALUE 8000000
    

// from STM Cube-IDE, Clock config
//extern "C"
void SystemClock_Config(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct = {0};  
 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 /** Configure the main internal regulator output voltage
 */
 __HAL_RCC_PWR_CLK_ENABLE();
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLM = 4;
 RCC_OscInitStruct.PLL.PLLN = 168;
 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
 RCC_OscInitStruct.PLL.PLLQ = 4;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
   Error_Handler();
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
 {
   Error_Handler();
 }
}


//change board config file
//#define PIN_SPI_SS
//#define PIN_SPI_SS1
// #define SPI_MOSI PC12
// #define SPI_MISO PC11
// #define SPI_SCLK PC10
//#define SERIAL_UART_INSTANCE  1
//#define PIN_SERIAL_RX         PA9
//#define PIN_SERIAL_TX         PA10

//SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI_SCLK);

// MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs)
//  config  - SPI config
//  cs      - SPI chip select pin 
// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, PA1);
// alternative constructor (chipselsect, bit_resolution, angle_read_register, )
// MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

void setup() {
  //pinMode(PB5, OUTPUT);
  // monitoring port
  //afio_remap(AFIO_REMAP_SPI3);
  //afio_remap(AFIO_REMAP_USART1);
  //Serial.begin(115200);

  SPI.setMISO(PC11);
  SPI.setMOSI(PC12);
  SPI.setSCLK(PC10);
  //SPI.setSSEL(PB1);

  // initialise magnetic sensor hardware
  sensor.init();
  //sensor.init(&SPI_2);

  //Serial.println("Sensor ready");
  _delay(1000);
  
}
float outAngle = 0;
void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  // display the angle and the angular velocity to the terminal
  outAngle = sensor.getAngle() * 1.0;
  //Serial.print(sensor.getAngle());
  //Serial.print("\t");
  //Serial.println(sensor.getVelocity());
}
