
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

#ifndef _EINK_1IN02_H_
#define _EINK_1IN02_H_

#include "EINK_Config.h"

// Display resolution
#define EPD_WIDTH   80
#define EPD_HEIGHT  128

UBYTE EPD_Init(void);
void EPD_TurnOnDisplay(void);
void EPD_Clear(void);
void EPD_Clear2(void);
void EPD_Clear3(void);
void EPD_Display_Image(UBYTE *Image);
void EPD_Reset(void);
void EPD_Sleep(void);
void EPD_Part_Init(void);
void EPD_DisplayPartia_Clear(void);
void EPD_SendData(UBYTE Data);
#endif
