#include "oled.h"
#include "ssd1306.h"
#include "imu.h"

uint16_t* fonts6x8; 
FontDef Font_6x8; 
uint8_t* SSD1306_Buffer;


void oled(void const * argument){
  UNUSED(argument);
  SSD1306_Buffer = (uint8_t*)pvPortMalloc(SSD1306_WIDTH * SSD1306_HEIGHT / 8);
  for(int i = 0; i < SSD1306_WIDTH * SSD1306_HEIGHT / 8; i ++){
    SSD1306_Buffer[i] = 0;
  }
  fonts6x8 = (uint16_t*)pvPortMalloc(sizeof(unsigned char) * 1520); 
  fonts6x8[1] = 0;
  fonts6x8[0] = 0;
  Sensors_I2C_WriteRegister(0x50, 12, 1, (uint8_t*)fonts6x8);
  taskENTER_CRITICAL();
  HAL_I2C_Master_Receive(&hi2c1, 0x50 << 1, (uint8_t*)fonts6x8, 1520, 1000000);
  taskEXIT_CRITICAL();

  Font_6x8.FontWidth = 6;
  Font_6x8.FontHeight = 8;
  Font_6x8.data = fonts6x8;

  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("test imu", Font_6x8, White);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("proyecto de mani", Font_6x8, White);
  ssd1306_UpdateScreen();
  int i = 0;
  for(;;){
    
    vTaskDelay(1000);
  }
}
