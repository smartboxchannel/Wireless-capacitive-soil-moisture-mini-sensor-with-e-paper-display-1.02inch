
// ##############    Mini wireless capacitive soil moisture sensor with 1.02 inch e-paper display | nRF52    ############## //
//                                                                                                                          //
//        @filename   :   EFEKTA_PWS102.ino                                                                                 //
//        @brief en   :   Wireless, battery-operated capacitive soil moisture sensor                                        //
//                        with electronic ink display(Good Display GDEW0102T4). Works on nRF52.                             //
//        @brief ru   :   Беcпроводной, батарейный датчик влажности почвы                                                   //
//                        с дисплеем на электронных чернилах(Good Display GDEW0102T4). Работает на nRF52.                   //
//        @author     :   Andrew Lamchenko aka Berk                                                                         //
//                                                                                                                          //
//        Copyright (C) EFEKTALAB 2020                                                                                      //
//        Copyright (c) 2014-2015 Arduino LLC.  All right reserved.                                                         //
//        Copyright (c) 2016 Arduino Srl.  All right reserved.                                                              //
//        Copyright (c) 2017 Sensnology AB. All right reserved.                                                             //
//        Copyright (C) Waveshare     August 10 2017//                                                                      //
//                                                                                                                          //
// ######################################################################################################################## //

#ifndef _EINK_CONFIG_H_
#define _EINK_CONFIG_H_
#include <arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include "MyConfig.h"


#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO config
**/
#ifdef NRF840
#define EPD_RST_PIN         24
#define EPD_DC_PIN          5
#define EPD_CS_PIN          31
#define EPD_BUSY_PIN        13
#else
#define EPD_RST_PIN         12
#define EPD_DC_PIN          17
#define EPD_CS_PIN          29
#define EPD_BUSY_PIN        11
#endif


/**
 * GPIO read and write
**/
#define EINK_Digital_Write(_pin, _value) digitalWrite(_pin, _value == 0? LOW:HIGH)
#define EINK_Digital_Read(_pin) digitalRead(_pin)


/**
 * SPI
**/
#define EINK_SPI_WriteByte(_dat)   SPI.transfer(_dat)

/**
 * delay x ms
**/
#define EINK_Delay_ms(__xms)    delay(__xms)



/*-----------------------------------------------------------------------------*/
 void Config_Init();
#endif
