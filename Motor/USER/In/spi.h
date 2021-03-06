////////////////////////////////////////////////////////////////////////////////
/// @file    spi.h
/// @author  AE TEAM
/// @brief   THIS FILE PROVIDES ALL THE SYSTEM FIRMWARE FUNCTIONS.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////


// Define to prevent recursive inclusion
#ifndef __SPI_H
#define __SPI_H

// Files includes

#include "hal_device.h"
#include "hal_conf.h"


////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Example_Layer
/// @brief MM32 Example Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_RESOURCE
/// @brief MM32 Examples resource modules
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Constants
/// @{



/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Enumeration
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @brief XXXX enumerate definition.
/// @anchor XXXX
////////////////////////////////////////////////////////////////////////////////


/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Variables
/// @{
#ifdef _BKP_C_
#define GLOBAL


#else
#define GLOBAL extern
#define SPI  SPI1

#define SPI_MOSI_BUSCLK                  RCC_AHBPeriph_GPIOA
#define SPI_MOSI_PIN                     GPIO_Pin_7
#define SPI_MOSI_PORT                    GPIOA
#define SPI_MOSI_AFSOURCE                GPIO_PinSource7
#define SPI_MOSI_AFMODE                  GPIO_AF_0

#define SPI_NSS_BUSCLK                  RCC_AHBPeriph_GPIOA
#define SPI_NSS_PIN                     GPIO_Pin_4
#define SPI_NSS_PORT                    GPIOA
#define SPI_NSS_AFSOURCE                GPIO_PinSource4
#define SPI_NSS_AFMODE                  GPIO_AF_0

#define SPI_MISO_BUSCLK                  RCC_AHBPeriph_GPIOA
#define SPI_MISO_PIN                     GPIO_Pin_6
#define SPI_MISO_PORT                    GPIOA
#define SPI_MISO_AFSOURCE                GPIO_PinSource6
#define SPI_MISO_AFMODE                  GPIO_AF_0

#define SPI_SCK_BUSCLK                  RCC_AHBPeriph_GPIOA
#define SPI_SCK_PIN                     GPIO_Pin_5
#define SPI_SCK_PORT                    GPIOA
#define SPI_SCK_AFSOURCE                GPIO_PinSource5
#define SPI_SCK_AFMODE                  GPIO_AF_0






#endif



#undef GLOBAL

/// @}


////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Functions
/// @{
static u32 SPI_WriteRead_1_32bits(SPI_TypeDef* SPIx, u32 gTxData);
static void SPI2_Config(SPI_TypeDef* SPIx, u16 spi_baud_div);
static void SPI2_GPIO_Config(void);
void SPI_1_32bit_Init(SPI_TypeDef* SPIx, u16 spi_baud_div);
bool SPI1_write(u8* ,u8* data,u8 );
bool SPI1_read(u8*,u8* ,u8);

void tmc5160_init(u8* rebuff_data);
void sendData(u8* rebuff_data,unsigned long address,long datagram);

u32 spi_read_write(u8* byte);


/// @}


/// @}

/// @}


////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////
