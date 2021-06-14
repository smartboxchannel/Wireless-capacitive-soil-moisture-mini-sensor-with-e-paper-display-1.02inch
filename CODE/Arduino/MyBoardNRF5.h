
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

#ifndef _MYBOARDNRF5_H_
#define _MYBOARDNRF5_H_
#include "MyConfig.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus


//#define NRF840

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (8u)


/*
    Buttons
*/
#define PIN_BUTTON             (3)

#define PIN_POWER_PWS         (27)

#define PIN_SENS_PWS          (28)

/*
   Analog ports

   If you change g_APinDescription, replace PIN_AIN0 with
   port numbers mapped by the g_APinDescription Array.
   You can add PIN_AIN0 to the g_APinDescription Array if
   you want provide analog ports MCU independed, you can add
   PIN_AIN0..PIN_AIN7 to your custom g_APinDescription Array
   defined in MyBoardNRF5.cpp
*/
static const uint8_t A0  = ADC_A0;
static const uint8_t A1  = ADC_A1;
static const uint8_t A2  = ADC_A2;
static const uint8_t A3  = ADC_A3;
static const uint8_t A4  = ADC_A4;
static const uint8_t A5  = ADC_A5;
static const uint8_t A6  = ADC_A6;
static const uint8_t A7  = ADC_A7;

/*
   Serial interfaces

   RX and TX are required.
*/

#ifdef NRF840
#define PIN_SERIAL_RX       (10)
#define PIN_SERIAL_TX       (9)
#else
//#define PIN_SERIAL_RX       (9)  //V1
//#define PIN_SERIAL_TX       (10) //V1

#define PIN_SERIAL_RX       (18) //V2
#define PIN_SERIAL_TX       (10) //V2
#endif
/*
   SPI Interfaces

   This is optional

   If SPI is defined MISO, MOSI, SCK are required
   SS is optional and can be used in your sketch.
*/
#define SPI_INTERFACES_COUNT 1
#ifdef NRF840
#define PIN_SPI_MISO         (18)
#define PIN_SPI_MOSI         (2)
#define PIN_SPI_SCK          (30)
#define PIN_SPI_SS           (31)
#else
#define PIN_SPI_MISO         (24)
#define PIN_SPI_MOSI         (4)
#define PIN_SPI_SCK          (30)
#define PIN_SPI_SS           (29)
#endif

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
   Wire Interfaces

   This is optional
*/
#define WIRE_INTERFACES_COUNT 2
#ifdef NRF840
#define PIN_WIRE_SDA         (29u)
#define PIN_WIRE_SCL         (28u)
#else
#define PIN_WIRE_SDA         (28u)
#define PIN_WIRE_SCL         (27u)
#endif

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
