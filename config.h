
#include_next "config.h"

#define BOOT_MODE BOOT_MODE_SLOW 1


#if 0
// 14 bit address bus
#define A0_PIN GPIO_PIN(GPIO_PB,0)	// TIM1, TIM3, SPI5
#define A1_PIN GPIO_PIN(GPIO_PB,1)	// TIM1, TIM3, SPI5
#define A2_PIN GPIO_PIN(GPIO_PB,2)	// BOOT1
#define A3_PIN GPIO_PIN(GPIO_PB,3)	// JTDO, TIM2, SPI1, SPI3, USART1, I2C2
#define A4_PIN GPIO_PIN(GPIO_PB,4)	// JTRST, TIM3, SPI1, SPI3, I2S, I2C3, SDIO
#define A5_PIN GPIO_PIN(GPIO_PB,5)	// TIM3, I2C1, SPI1, SPI3, SDIO
#define A6_PIN GPIO_PIN(GPIO_PB,6)	// TIM4, I2C1, USART1
#define A7_PIN GPIO_PIN(GPIO_PB,7)	// TIM4, I2C1, USART1 SDIO
#define A8_PIN GPIO_PIN(GPIO_PB,8)	// TIM4, TIM10, I2C1, SPI5, I2C3, SDIO
#define A9_PIN GPIO_PIN(GPIO_PB,9)	// TIM4, TIM11, I2C1, SPI2, I2S3, I2C2, SDIO
#define A10_PIN GPIO_PIN(GPIO_PB,10)	// TIM2, I2C2, SPI2, I2S3, SDIO
#define A11_PIN GPIO_PIN(GPIO_PB,12) // TIM1, I2C2, SPI2, SPI4, SPI3
#define A12_PIN GPIO_PIN(GPIO_PB,13)	// TIM1, SPI2, SPI4
#define A13_PIN GPIO_PIN(GPIO_PB,14)	// TIM1, SPI2, I2S, SDIO
#define A14_PIN GPIO_PIN(GPIO_PB,15) // RTC, TIM1, SPI2, SDIO

// 8 bit data bus
#define D0_PIN GPIO_PIN(GPIO_PA,1)	// TIM2, TIM5, USART2, ADC1_0, WKUP
#define D1_PIN GPIO_PIN(GPIO_PA,2)	// TIM2, TIM5, SPI4, USART2, ADC1_1
#define D2_PIN GPIO_PIN(GPIO_PA,3)	// TIM2, TIM5, TIM9, I2S2, USART2, ADC1_2
#define D3_PIN GPIO_PIN(GPIO_PA,4)	// TIM2, TIM5, TIM9, I2S2, USART2, ADC1_3
#define D4_PIN GPIO_PIN(GPIO_PA,5)	// SPI1, SPI3, USART2, ADC1_4
#define D5_PIN GPIO_PIN(GPIO_PA,6)	// TIM2, SPI1, ADC1_5
#define D6_PIN GPIO_PIN(GPIO_PA,7)	// TIM1, TIM3, SPI1, I2S2, SDIO, ADC1_6
#define D7_PIN GPIO_PIN(GPIO_PA,8)	// TIM1, TIM3, SPI1, ADC1_7

#define OE GPIO_PIN(GPIO_PA,0)	// MCO_1, TIM1, I2C3, USART1, USB_FS_SOF,SDIO
#define CE GPIO_PIN(GPIO_PC,13)	// RTC, LED
#endif

// Irq pins: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
//           B  A (b)(b) B  A  A  A  A  A   A   A   A   B   C   B
//            free for irq input b2 and b3

//#define FREE_INPUT_P3	GPIO_PIN(GPIO_PC,14)
#define IGNITION	GPIO_PIN(GPIO_PA,9)	// irq 9
#define TUNE_WHEEL_4	GPIO_PIN(GPIO_PA,1)	// irq 1
#define TUNE_WHEEL_2	GPIO_PIN(GPIO_PA,12)	// irq 12
#define TUNE_WHEEL_1	GPIO_PIN(GPIO_PA,11)	// irq 11
#define STOP		GPIO_PIN(GPIO_PA,4)	// no irq, input
#define STEREO		GPIO_PIN(GPIO_PA,5)	// irq 5
#define P4Seek		GPIO_PIN(GPIO_PA,6)	// irq 6
#define P2		GPIO_PIN(GPIO_PA,7)	// irq 7
#define P1P3		GPIO_PIN(GPIO_PB,0)	// irq 0

#define CP2P3P4Set	GPIO_PIN(GPIO_PB,1)	// no irq output
#define CP1ScanSeek	GPIO_PIN(GPIO_PB,7)	// no irq output
#define VF_STROBE_PIN	GPIO_PIN(GPIO_PB,9)	// no irq output
#define SYN_STROBE_PIN  GPIO_PIN(GPIO_PB,8)	// no irq output
#define RECALL		GPIO_PIN(GPIO_PB,15)	// irq 15
#define RSENSE		GPIO_PIN(GPIO_PA,8)	// irq 8 , input
#define RMUTE		GPIO_PIN(GPIO_PB,14)	// no irq, output
#define AM_FM		GPIO_PIN(GPIO_PB,13)	// irq 13
#define TOD		GPIO_PIN(GPIO_PB,4)	// irq 4 input 50 Hz
#define CLK_PIN		GPIO_PIN(GPIO_PB,5)	// no irq output
#define DATA_PIN	GPIO_PIN(GPIO_PA,15)	// no irq output
#define SetScan		GPIO_PIN(GPIO_PA,10)	// irq 10 input

#define BT_MUTE		GPIO_PIN(GPIO_PB,2)	// output
#define RADIO_RUN	GPIO_PIN(GPIO_PB,10)	// output
// PB2, PB3, PC14 free for irq+input or output
// PB6, PB10, PB12, PC15 fro input or output, but not irq
