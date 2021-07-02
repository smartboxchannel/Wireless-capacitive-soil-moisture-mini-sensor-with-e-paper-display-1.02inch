
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

#include "MyConfig.h"

#include "EINK_Paint.h"
#include "EINK_Config.h"
#include "EINK_1in02d.h"
#include "EINK_imagedata.h"
unsigned char image_temp[1280];
Paint paith(image_temp, 80, 128);

bool flag_rb;
bool firstLoop;
bool firstLoop2;
bool altfont;
bool colorPrint;
bool opposite_colorPrint;
bool updateink1;
bool updateink2;
bool updateink3;
bool updateink4;
bool updateinkclear;
bool change;
bool chek_h = true;
bool first_send_battery = true;
bool check;
bool tch;
bool hch;
bool bch;
bool vch;
bool configMode;
bool button_flag;
bool nosleep;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool Ack_FP;

uint8_t lang;
uint8_t cpNom;
uint8_t cpCount;
uint8_t timeSend;
uint8_t battSend;
uint8_t battery;
uint8_t old_battery;
uint8_t err_delivery_beat;
uint8_t problem_mode_count;
//uint8_t bindSend;
uint8_t threshholdHum;


float readyCalcSend;
float celsius;
uint16_t m_s_m_calc;
uint16_t readyCalc;
uint16_t lastReadyCalc = 1000;
float lastcelsius;
const float celsiusThreshold = 2.0;
const uint16_t calcThreshold = 1;
int16_t nRFRSSI;
int16_t old_nRFRSSI;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;

uint16_t temp;
uint16_t batteryVoltage;
uint16_t old_batteryVoltage;
const uint16_t minuteT = 60000;
uint16_t BATT_TIME;
uint16_t BATT_COUNT;

uint32_t stopTimer;
uint32_t startTimer;
uint32_t configMillis;
uint32_t previousMillis;
uint32_t SLEEP_TIME;
const uint32_t SLEEP_TIME_WDT = 10000;
uint32_t PRECISION_TIME_WDT;
uint32_t sleepTimeCount;
const uint32_t shortWait = 50;
float differenceAD1;
float differenceAD2;
float batteryVoltageF;
float filtered_m_s_m_calc;


const uint16_t maxi = 3300; //  nom 3900, lmc555 eink on 3v3
const uint16_t nomV = 3300; // nom voltage
const uint16_t mini = 2100; // nom 2000, lmc555 eink on 3v3
uint16_t differenceV;
uint16_t maxiM;
uint16_t miniM;

float filtered_m_s_m = 0;

uint32_t rawTemperature = 0;


#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif
#define MY_RADIO_NRF5_ESB
#ifdef NRF840
#define MY_NRF5_ESB_PA_LEVEL (0x8UL)
#else
#define MY_NRF5_ESB_PA_LEVEL (0x4UL)
#endif
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)



#define HUM_ID 1
#define TEMP_ID 2
#define SIGNAL_Q_ID 100
#define BATTERY_VOLTAGE_ID 101
#define SET_TIME_SEND_ID 102
#define SET_BATT_SEND_ID 103
#define MY_SEND_RESET_REASON 105
#define SET_COLOR_ID 106
#define SET_FONTS_ID 107
//#define SET_BIND_ID 108
#define SET_THRESHOLD_HUM_ID 109

#include <MySensors.h>
MyMessage msgTemp(TEMP_ID, V_TEMP);
MyMessage msgHum(HUM_ID, V_HUM);
MyMessage sqMsg(SIGNAL_Q_ID, V_VAR1);
MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
MyMessage setTimeSendMsg(SET_TIME_SEND_ID, V_VAR1);
MyMessage setBattSendMsg(SET_BATT_SEND_ID, V_VAR1);
MyMessage sendMsg(MY_SEND_RESET_REASON, V_VAR1);
MyMessage setColor(SET_COLOR_ID, V_VAR1);
MyMessage setFonts(SET_FONTS_ID, V_VAR1);
//MyMessage setBind(SET_BIND_ID, V_VAR1);
MyMessage setVar(BATTERY_VOLTAGE_ID, V_VAR1);
MyMessage setThreshHum(SET_THRESHOLD_HUM_ID, V_VAR1);

uint32_t PIN_BUTTON_MASK;
volatile byte buttIntStatus = 0;
#define APP_GPIOTE_MAX_USERS 1
extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
static app_gpiote_user_id_t m_gpiote_user_id;



//################################################### EINK ###################################################

void reseteinkset() {
  updateink1 = false;
  updateink2 = false;
  updateink3 = false;
  updateink4 = false;
  updateinkclear = false;
}


void einkZeropush() {
  wdt_nrfReset();
  //wait(20);
  EPD_Clear2();
  wait(30);
  paith.Clear(opposite_colorPrint);
  if (flag_nogateway_mode == false) {
#ifdef LANG_RU
    DrawImageWH(&paith, 0, 30, PRESENT, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
    DrawImageWH(&paith, 0, 30, PRESENTEN, 80, 35, colorPrint);
#endif
  } else {
#ifdef LANG_RU
    DrawImageWH(&paith, 0, 30, FINDNET, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
    DrawImageWH(&paith, 0, 30, FINDNETEN, 80, 35, colorPrint);
#endif
  }
  EPD_Display_Image((UBYTE *)image_temp);
}


void einkZeroend() {
  wdt_nrfReset();
  if (colorPrint == false) {
    EPD_Clear2();
  }
  paith.Clear(opposite_colorPrint);
  if (flag_nogateway_mode == false) {
#ifdef LANG_RU
    DrawImageWH(&paith, 0, 30, PRESENT2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
    DrawImageWH(&paith, 0, 30, PRESENT2EN, 80, 35, colorPrint);
#endif
  } else {
#ifdef LANG_RU
    DrawImageWH(&paith, 0, 30, FINDNET2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
    DrawImageWH(&paith, 0, 30, FINDNET2EN, 80, 35, colorPrint);
#endif
  }
  EPD_Display_Image((UBYTE *)image_temp);
}


void einkOnepush() {
  wdt_nrfReset();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, CONF, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, CONFEN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
}


void einkOneend() {
  wdt_nrfReset();
  if (colorPrint == false) {
    EPD_Clear2();
  }
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, CONF2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, CONF2EN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
}

/*
void einkOnePluspush() {
  wdt_nrfReset();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, BIND, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, BINDEN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
}


void einkOnePlusend() {
  wdt_nrfReset();
  if (colorPrint == false) {
    EPD_Clear2();
  }
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, BIND2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, BIND2EN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
}
*/

void einkTwopush() {
  wdt_nrfReset();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, RESET, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, RESETEN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
}


void einkTwoend() {
  wdt_nrfReset();
  if (colorPrint == false) {
    EPD_Clear2();
  }
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, RESET2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, RESET2EN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
}


void reportTimeInk() {
  wdt_nrfReset();
  //timeSend = 5; // для теста
  EPD_Clear2();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 20, CHSD, 80, 35, colorPrint);
  DrawImageWH(&paith, 0, 55, CHSD2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 20, CHSDEN, 80, 35, colorPrint);
  DrawImageWH(&paith, 0, 55, CHSD2EN, 80, 35, colorPrint);
#endif

  if (altfont == false) {
    if (timeSend >= 10) {
      byte one_t = timeSend / 10;
      byte two_t = timeSend % 10;
      switch (one_t) {
        case 0:
          DrawImageWH(&paith, 16, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 16, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 16, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 16, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 16, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 16, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 16, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 16, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 16, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 16, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 16, 90, sNumN, 24, 34, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paith, 40, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    } else {
      switch (timeSend) {
        case 0:
          DrawImageWH(&paith, 28, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 28, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 28, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 28, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 28, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 28, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 28, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 28, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 28, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 28, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 28, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    }
  } else {
    if (timeSend >= 10) {
      byte one_t = timeSend / 10;
      byte two_t = timeSend % 10;
      switch (one_t) {
        case 0:
          DrawImageWH(&paith, 16, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 16, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 16, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 16, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 16, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 16, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 16, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 16, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 16, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 16, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 16, 90, sNumN, 24, 34, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paith, 40, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    } else {
      switch (timeSend) {
        case 0:
          DrawImageWH(&paith, 28, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 28, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 28, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 28, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 28, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 28, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 28, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 28, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 28, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 28, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 28, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    }
  }
  EPD_Display_Image((UBYTE *)image_temp);
  wait(shortWait * 40);
}


void reportThreshHumInk() {
  wdt_nrfReset();
  //threshholdHum = 40; // для теста
  EPD_Clear2();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 45, THRESHUM, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 45, THRESHUM, 80, 35, colorPrint);
#endif

  if (altfont == false) {
    if (threshholdHum >= 10) {
      byte one_t = threshholdHum / 10;
      byte two_t = threshholdHum % 10;
      switch (one_t) {
        case 0:
          DrawImageWH(&paith, 16, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 16, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 16, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 16, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 16, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 16, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 16, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 16, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 16, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 16, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 16, 90, sNumN, 24, 34, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paith, 40, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    } else {
      switch (threshholdHum) {
        case 0:
          DrawImageWH(&paith, 28, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 28, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 28, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 28, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 28, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 28, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 28, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 28, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 28, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 28, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 28, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    }
  } else {
    if (threshholdHum >= 10) {
      byte one_t = threshholdHum / 10;
      byte two_t = threshholdHum % 10;
      switch (one_t) {
        case 0:
          DrawImageWH(&paith, 16, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 16, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 16, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 16, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 16, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 16, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 16, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 16, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 16, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 16, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 16, 90, sNumN, 24, 34, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paith, 40, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    } else {
      switch (threshholdHum) {
        case 0:
          DrawImageWH(&paith, 28, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 28, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 28, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 28, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 28, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 28, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 28, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 28, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 28, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 28, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 28, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    }
  }
  EPD_Display_Image((UBYTE *)image_temp);
  wait(shortWait * 40);
}


void reportBattInk() {
  wdt_nrfReset();
  //battSend = 5; // для теста
  EPD_Clear2();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 20, BS, 80, 35, colorPrint);
  DrawImageWH(&paith, 0, 55, BS2, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 20, BSEN, 80, 35, colorPrint);
  DrawImageWH(&paith, 0, 55, BS2EN, 80, 35, colorPrint);
#endif

  if (altfont == false) {
    if (battSend >= 10) {
      byte one_t = battSend / 10;
      byte two_t = battSend % 10;
      switch (one_t) {
        case 0:
          DrawImageWH(&paith, 16, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 16, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 16, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 16, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 16, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 16, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 16, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 16, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 16, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 16, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 16, 90, sNumN, 24, 34, colorPrint);
          break;
      }
      switch (two_t) {
        case 0:
          DrawImageWH(&paith, 40, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    } else {
      switch (battSend) {
        case 0:
          DrawImageWH(&paith, 28, 90, sNum0, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 28, 90, sNum1, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 28, 90, sNum2, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 28, 90, sNum3, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 28, 90, sNum4, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 28, 90, sNum5, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 28, 90, sNum6, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 28, 90, sNum7, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 28, 90, sNum8, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 28, 90, sNum9, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 28, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    }
  } else {
    if (battSend >= 10) {
      byte one_t = battSend / 10;
      byte two_t = battSend % 10;
      switch (one_t) {
        case 0:
          DrawImageWH(&paith, 16, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 16, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 16, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 16, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 16, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 16, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 16, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 16, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 16, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 16, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 16, 90, sNumN, 24, 34, colorPrint);
          break;
      }
      switch (two_t) {
        case 0:
          DrawImageWH(&paith, 40, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    } else {
      switch (battSend) {
        case 0:
          DrawImageWH(&paith, 28, 90, sNum0d, 24, 34, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 28, 90, sNum1d, 24, 34, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 28, 90, sNum2d, 24, 34, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 28, 90, sNum3d, 24, 34, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 28, 90, sNum4d, 24, 34, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 28, 90, sNum5d, 24, 34, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 28, 90, sNum6d, 24, 34, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 28, 90, sNum7d, 24, 34, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 28, 90, sNum8d, 24, 34, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 28, 90, sNum9d, 24, 34, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 28, 90, sNumN, 24, 34, colorPrint);
          break;
      }
    }
  }
  EPD_Display_Image((UBYTE *)image_temp);
  wait(2000);
}

/*
void reportBindInk() {
  wdt_nrfReset();
  EPD_Clear2();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, BINDOK, 80, 35, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 20, BINDOKEN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
  wait(2000);
}
*/

void display_Table()
{
  paith.DrawHorizontalLine(0, 114, 80, colorPrint);

  int8_t s = nRFRSSI;
  if (flag_nogateway_mode == true) {
    s = -1;
    DrawImageWH(&paith, 51, 116, sigNo, 24, 10, colorPrint);
  }
  if (s == 0) {
    DrawImageWH(&paith, 51, 116, sig0, 24, 10, colorPrint);
  }
  if (s > 0 && s < 25) {
    DrawImageWH(&paith, 51, 116, sig25, 24, 10, colorPrint);
  }
  if (s >= 25 && s < 50) {
    DrawImageWH(&paith, 51, 116, sig50, 24, 10, colorPrint);
  }
  if (s >= 50 && s < 75) {
    DrawImageWH(&paith, 51, 116, sig75, 24, 10, colorPrint);
  }
  if (s >= 75 && s <= 100) {
    DrawImageWH(&paith, 51, 116, sig100, 24, 10, colorPrint);
  }


  uint8_t b = battery;
  if (b < 3) {
    DrawImageWH(&paith, 5, 116, batt000, 24, 10, colorPrint);
  }
  if (b >= 3 && b < 17) {
    DrawImageWH(&paith, 5, 116, batt167, 24, 10, colorPrint);
  }
  if (b >= 17 && b < 33) {
    DrawImageWH(&paith, 5, 116, batt333, 24, 10, colorPrint);
  }
  if (b >= 33 && b < 50) {
    DrawImageWH(&paith, 5, 116, batt500, 24, 10, colorPrint);
  }
  if (b >= 50 && b <= 67) {
    DrawImageWH(&paith, 5, 116, batt667, 24, 10, colorPrint);
  }
  if (b >= 67 && b <= 83) {
    DrawImageWH(&paith, 5, 116, batt833, 24, 10, colorPrint);
  }
  if (b >= 83 && b <= 100) {
    DrawImageWH(&paith, 5, 116, batt1000, 24, 10, colorPrint);
  }

  old_battery = b;

#ifdef LANG_RU
  DrawImageWH(&paith, 0, 4, humRU, 80, 10, colorPrint);
#endif
#ifdef LANG_EN
  DrawImageWH(&paith, 0, 4, humEN, 80, 10, colorPrint);
#endif

  if ((readyCalc > threshholdHum) || (readyCalc == 0)) {
#ifdef LANG_RU
    DrawImageWH(&paith, 0, 68, tempRU, 80, 10, colorPrint);
#endif
#ifdef LANG_EN
    DrawImageWH(&paith, 0, 68, tempEN, 80, 10, colorPrint);
#endif
  }
}


void convert(uint16_t  t, uint16_t  h) {
  wdt_nrfReset();
  //wait(20);
  EPD_Clear2();
  wait(30);
  paith.Clear(opposite_colorPrint);
  display_Table();

  //t = 99;  // for test
  byte one_h = h / 10;
  byte two_h = h % 10;

  if (altfont == false) {
    if (one_h == 0) {
      switch (two_h) {
        case 0:
          DrawImageWH(&paith, 24, 14, bNum0, 32, 50, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 24, 14, bNum1, 32, 50, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 24, 14, bNum2, 32, 50, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 24, 14, bNum3, 32, 50, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 24, 14, bNum4, 32, 50, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 24, 14, bNum5, 32, 50, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 24, 14, bNum6, 32, 50, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 24, 14, bNum7, 32, 50, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 24, 14, bNum8, 32, 50, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 24, 14, bNum9, 32, 50, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 24, 14, bNumN, 32, 50, colorPrint);
          break;
      }

      DrawImageWH(&paith, 55, 20, sProc, 8, 11, colorPrint);
    } else {
      switch (one_h) {
        case 0:
          DrawImageWH(&paith, 8, 14, bNum0, 32, 50, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 8, 14, bNum1, 32, 50, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 8, 14, bNum2, 32, 50, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 8, 14, bNum3, 32, 50, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 8, 14, bNum4, 32, 50, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 8, 14, bNum5, 32, 50, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 8, 14, bNum6, 32, 50, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 8, 14, bNum7, 32, 50, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 8, 14, bNum8, 32, 50, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 8, 14, bNum9, 32, 50, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 8, 14, bNumN, 32, 50, colorPrint);
          break;
      }
      switch (two_h) {
        case 0:
          DrawImageWH(&paith, 40, 14, bNum0, 32, 50, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 14, bNum1, 32, 50, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 14, bNum2, 32, 50, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 14, bNum3, 32, 50, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 14, bNum4, 32, 50, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 14, bNum5, 32, 50, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 14, bNum6, 32, 50, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 14, bNum7, 32, 50, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 14, bNum8, 32, 50, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 14, bNum9, 32, 50, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 14, bNumN, 32, 50, colorPrint);
          break;
      }

      DrawImageWH(&paith, 71, 20, sProc, 8, 11, colorPrint);
    }
    if (h <= threshholdHum && h != 0) {
      DrawImageWH(&paith, 13, 70, WW, 54, 41, colorPrint);
    } else {
      byte one_t = t / 10;
      byte two_t = t % 10;
      if (one_t == 0) {
        switch (one_t) {
          case 0:
            DrawImageWH(&paith, 28, 78, sNum0, 24, 34, colorPrint);
            break;
          case 1:
            DrawImageWH(&paith, 28, 78, sNum1, 24, 34, colorPrint);
            break;
          case 2:
            DrawImageWH(&paith, 28, 78, sNum2, 24, 34, colorPrint);
            break;
          case 3:
            DrawImageWH(&paith, 28, 78, sNum3, 24, 34, colorPrint);
            break;
          case 4:
            DrawImageWH(&paith, 28, 78, sNum4, 24, 34, colorPrint);
            break;
          case 5:
            DrawImageWH(&paith, 28, 78, sNum5, 24, 34, colorPrint);
            break;
          case 6:
            DrawImageWH(&paith, 28, 78, sNum6, 24, 34, colorPrint);
            break;
          case 7:
            DrawImageWH(&paith, 28, 78, sNum7, 24, 34, colorPrint);
            break;
          case 8:
            DrawImageWH(&paith, 28, 78, sNum8, 24, 34, colorPrint);
            break;
          case 9:
            DrawImageWH(&paith, 28, 78, sNum9, 24, 34, colorPrint);
            break;
          default:
            DrawImageWH(&paith, 28, 78, sNumN, 24, 34, colorPrint);
            break;
        }

        DrawImageWH(&paith, 52, 81, sCel, 8, 11, colorPrint);
      } else {
        switch (one_t) {
          case 0:
            DrawImageWH(&paith, 16, 78, sNum0, 24, 34, colorPrint);
            break;
          case 1:
            DrawImageWH(&paith, 16, 78, sNum1, 24, 34, colorPrint);
            break;
          case 2:
            DrawImageWH(&paith, 16, 78, sNum2, 24, 34, colorPrint);
            break;
          case 3:
            DrawImageWH(&paith, 16, 78, sNum3, 24, 34, colorPrint);
            break;
          case 4:
            DrawImageWH(&paith, 16, 78, sNum4, 24, 34, colorPrint);
            break;
          case 5:
            DrawImageWH(&paith, 16, 78, sNum5, 24, 34, colorPrint);
            break;
          case 6:
            DrawImageWH(&paith, 16, 78, sNum6, 24, 34, colorPrint);
            break;
          case 7:
            DrawImageWH(&paith, 16, 78, sNum7, 24, 34, colorPrint);
            break;
          case 8:
            DrawImageWH(&paith, 16, 78, sNum8, 24, 34, colorPrint);
            break;
          case 9:
            DrawImageWH(&paith, 16, 78, sNum9, 24, 34, colorPrint);
            break;
          default:
            DrawImageWH(&paith, 16, 78, sNumN, 24, 34, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paith, 40, 78, sNum0, 24, 34, colorPrint);
            break;
          case 1:
            DrawImageWH(&paith, 40, 78, sNum1, 24, 34, colorPrint);
            break;
          case 2:
            DrawImageWH(&paith, 40, 78, sNum2, 24, 34, colorPrint);
            break;
          case 3:
            DrawImageWH(&paith, 40, 78, sNum3, 24, 34, colorPrint);
            break;
          case 4:
            DrawImageWH(&paith, 40, 78, sNum4, 24, 34, colorPrint);
            break;
          case 5:
            DrawImageWH(&paith, 40, 78, sNum5, 24, 34, colorPrint);
            break;
          case 6:
            DrawImageWH(&paith, 40, 78, sNum6, 24, 34, colorPrint);
            break;
          case 7:
            DrawImageWH(&paith, 40, 78, sNum7, 24, 34, colorPrint);
            break;
          case 8:
            DrawImageWH(&paith, 40, 78, sNum8, 24, 34, colorPrint);
            break;
          case 9:
            DrawImageWH(&paith, 40, 78, sNum9, 24, 34, colorPrint);
            break;
          default:
            DrawImageWH(&paith, 40, 78, sNumN, 24, 34, colorPrint);
            break;
        }

        DrawImageWH(&paith, 64, 81, sCel, 8, 11, colorPrint);
      }
    }
  } else {
    if (one_h == 0) {
      switch (two_h) {
        case 0:
          DrawImageWH(&paith, 24, 14, bNum0d, 32, 50, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 24, 14, bNum1d, 32, 50, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 24, 14, bNum2d, 32, 50, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 24, 14, bNum3d, 32, 50, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 24, 14, bNum4d, 32, 50, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 24, 14, bNum5d, 32, 50, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 24, 14, bNum6d, 32, 50, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 24, 14, bNum7d, 32, 50, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 24, 14, bNum8d, 32, 50, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 24, 14, bNum9d, 32, 50, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 24, 14, bNumN, 32, 50, colorPrint);
          break;
      }

      DrawImageWH(&paith, 55, 20, sProcd, 8, 11, colorPrint);
    } else {
      switch (one_h) {
        case 0:
          DrawImageWH(&paith, 8, 14, bNum0d, 32, 50, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 8, 14, bNum1d, 32, 50, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 8, 14, bNum2d, 32, 50, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 8, 14, bNum3d, 32, 50, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 8, 14, bNum4d, 32, 50, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 8, 14, bNum5d, 32, 50, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 8, 14, bNum6d, 32, 50, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 8, 14, bNum7d, 32, 50, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 8, 14, bNum8d, 32, 50, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 8, 14, bNum9d, 32, 50, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 8, 14, bNumN, 32, 50, colorPrint);
          break;
      }
      switch (two_h) {
        case 0:
          DrawImageWH(&paith, 40, 14, bNum0d, 32, 50, colorPrint);
          break;
        case 1:
          DrawImageWH(&paith, 40, 14, bNum1d, 32, 50, colorPrint);
          break;
        case 2:
          DrawImageWH(&paith, 40, 14, bNum2d, 32, 50, colorPrint);
          break;
        case 3:
          DrawImageWH(&paith, 40, 14, bNum3d, 32, 50, colorPrint);
          break;
        case 4:
          DrawImageWH(&paith, 40, 14, bNum4d, 32, 50, colorPrint);
          break;
        case 5:
          DrawImageWH(&paith, 40, 14, bNum5d, 32, 50, colorPrint);
          break;
        case 6:
          DrawImageWH(&paith, 40, 14, bNum6d, 32, 50, colorPrint);
          break;
        case 7:
          DrawImageWH(&paith, 40, 14, bNum7d, 32, 50, colorPrint);
          break;
        case 8:
          DrawImageWH(&paith, 40, 14, bNum8d, 32, 50, colorPrint);
          break;
        case 9:
          DrawImageWH(&paith, 40, 14, bNum9d, 32, 50, colorPrint);
          break;
        default:
          DrawImageWH(&paith, 40, 14, bNumN, 32, 50, colorPrint);
          break;
      }

      DrawImageWH(&paith, 71, 20, sProcd, 8, 11, colorPrint);
    }
    if (h <= threshholdHum && h != 0) {
      DrawImageWH(&paith, 13, 70, WW, 54, 41, colorPrint);
    } else {
      //h = 99;  // for test
      byte one_t = t / 10;
      byte two_t = t % 10;
      if (one_t == 0) {
        switch (one_t) {
          case 0:
            DrawImageWH(&paith, 28, 78, sNum0d, 24, 34, colorPrint);
            break;
          case 1:
            DrawImageWH(&paith, 28, 78, sNum1d, 24, 34, colorPrint);
            break;
          case 2:
            DrawImageWH(&paith, 28, 78, sNum2d, 24, 34, colorPrint);
            break;
          case 3:
            DrawImageWH(&paith, 28, 78, sNum3d, 24, 34, colorPrint);
            break;
          case 4:
            DrawImageWH(&paith, 28, 78, sNum4d, 24, 34, colorPrint);
            break;
          case 5:
            DrawImageWH(&paith, 28, 78, sNum5d, 24, 34, colorPrint);
            break;
          case 6:
            DrawImageWH(&paith, 28, 78, sNum6d, 24, 34, colorPrint);
            break;
          case 7:
            DrawImageWH(&paith, 28, 78, sNum7d, 24, 34, colorPrint);
            break;
          case 8:
            DrawImageWH(&paith, 28, 78, sNum8d, 24, 34, colorPrint);
            break;
          case 9:
            DrawImageWH(&paith, 28, 78, sNum9d, 24, 34, colorPrint);
            break;
          default:
            DrawImageWH(&paith, 28, 78, sNumN, 24, 34, colorPrint);
            break;
        }

        DrawImageWH(&paith, 52, 81, sCeld, 8, 11, colorPrint);
      } else {
        switch (one_t) {
          case 0:
            DrawImageWH(&paith, 16, 78, sNum0d, 24, 34, colorPrint);
            break;
          case 1:
            DrawImageWH(&paith, 16, 78, sNum1d, 24, 34, colorPrint);
            break;
          case 2:
            DrawImageWH(&paith, 16, 78, sNum2d, 24, 34, colorPrint);
            break;
          case 3:
            DrawImageWH(&paith, 16, 78, sNum3d, 24, 34, colorPrint);
            break;
          case 4:
            DrawImageWH(&paith, 16, 78, sNum4d, 24, 34, colorPrint);
            break;
          case 5:
            DrawImageWH(&paith, 16, 78, sNum5d, 24, 34, colorPrint);
            break;
          case 6:
            DrawImageWH(&paith, 16, 78, sNum6d, 24, 34, colorPrint);
            break;
          case 7:
            DrawImageWH(&paith, 16, 78, sNum7d, 24, 34, colorPrint);
            break;
          case 8:
            DrawImageWH(&paith, 16, 78, sNum8d, 24, 34, colorPrint);
            break;
          case 9:
            DrawImageWH(&paith, 16, 78, sNum9d, 24, 34, colorPrint);
            break;
          default:
            DrawImageWH(&paith, 16, 78, sNumN, 24, 34, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paith, 40, 78, sNum0d, 24, 34, colorPrint);
            break;
          case 1:
            DrawImageWH(&paith, 40, 78, sNum1d, 24, 34, colorPrint);
            break;
          case 2:
            DrawImageWH(&paith, 40, 78, sNum2d, 24, 34, colorPrint);
            break;
          case 3:
            DrawImageWH(&paith, 40, 78, sNum3d, 24, 34, colorPrint);
            break;
          case 4:
            DrawImageWH(&paith, 40, 78, sNum4d, 24, 34, colorPrint);
            break;
          case 5:
            DrawImageWH(&paith, 40, 78, sNum5d, 24, 34, colorPrint);
            break;
          case 6:
            DrawImageWH(&paith, 40, 78, sNum6d, 24, 34, colorPrint);
            break;
          case 7:
            DrawImageWH(&paith, 40, 78, sNum7d, 24, 34, colorPrint);
            break;
          case 8:
            DrawImageWH(&paith, 40, 78, sNum8d, 24, 34, colorPrint);
            break;
          case 9:
            DrawImageWH(&paith, 40, 78, sNum9d, 24, 34, colorPrint);
            break;
          default:
            DrawImageWH(&paith, 40, 78, sNumN, 24, 34, colorPrint);
            break;
        }

        DrawImageWH(&paith, 64, 81, sCeld, 8, 11, colorPrint);
      }
    }
  }


  EPD_Display_Image((UBYTE *)image_temp);
  wdt_nrfReset();
}


void DrawImageWH(Paint * paith, int x, int y, const unsigned char* imgData, int Width, int Height, int colored)
{
  int i, j;
  const unsigned char* prt = imgData;
  for (j = 0; j < Height; j++) {
    for (i = 0; i < Width; i++) {
      if (pgm_read_byte(prt) & (0x80 >> (i % 8))) {
        paith->DrawPixel(x + i, y + j, colored);
      }
      if (i % 8 == 7) {
        prt++;
      }
    }
    if (Width % 8 != 0) {
      prt++;
    }
  }
}



//########################################## BATTARY ###################################################


void readBatt() {
  wait(shortWait * 5);
  batteryVoltage = hwCPUVoltage();                            // Read internal voltage from mcu
  if (flag_rb == false) {
    battery = battery_level_in_percent(batteryVoltage);
    if (battery > 100) {
      battery = 100;
    }
    bch = true;
    change = true;
    if (batteryVoltage != old_batteryVoltage) {
      old_batteryVoltage = batteryVoltage;
      batteryVoltageF = (float)batteryVoltage / 1000;  // Сonversion to volts
      vch = true;
    }
    flag_rb = true;
  }

  BATT_COUNT++;
  CORE_DEBUG(PSTR("BATT_COUNT: %d\n"), BATT_COUNT);
  if (BATT_COUNT >= BATT_TIME) {
    battery = battery_level_in_percent(batteryVoltage);
    if (battery > 100) {
      battery = 100;
    }
    bch = true;
    change = true;
    if (batteryVoltage != old_batteryVoltage) {
      old_batteryVoltage = batteryVoltage;
      batteryVoltageF = (float)batteryVoltage / 1000;;          // Сonversion to volts
      vch = true;
    }
    BATT_COUNT = 0;
  }
}


void batLevSend() {
  if (battery > 100) {
    battery = 100;
  }
  sendBatteryLevel(battery, true);
  wait(500, C_INTERNAL, I_BATTERY_LEVEL);
  lqSend();

  wdt_nrfReset();
}


void lqSend() {
  nRFRSSI = transportGetReceivingRSSI();
  nRFRSSI = map(nRFRSSI, -85, -40, 0, 100);
  if (nRFRSSI < 0) {
    nRFRSSI = 0;
  }
  if (nRFRSSI > 100) {
    nRFRSSI = 100;
  }

  if (nRFRSSI != old_nRFRSSI) {
    old_nRFRSSI = nRFRSSI;

    check = send(sqMsg.set(nRFRSSI));
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
    } else {
      CORE_DEBUG(PSTR("MyS: SEND LINK QUALITY\n"));
      CORE_DEBUG(PSTR("MyS: LINK QUALITY %: %d\n"), nRFRSSI);
    }
  }
}


void sendBattVoltage() {

  check = send(bvMsg.set(batteryVoltageF, 3));
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    CORE_DEBUG(PSTR("MyS: SEND BATTERY VOLTAGE\n"));
  }
}


static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 20) / 100;
  }
  else if (mvolts > 2750)
  {
    battery_level = 80 - ((2900 - mvolts) * 30) / 150;
  }
  else if (mvolts > 2550)
  {
    battery_level = 50 - ((2750 - mvolts) * 40) / 200;
  }
  else if (mvolts > 2250)
  {
    battery_level = 10 - ((2550 - mvolts) * 10) / 300;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}



//####################################### SENSOR DATA ##################################################

void readData() {
  CORE_DEBUG(PSTR("MyS: START READ PWSENSOR\n"));
  msm ();
  CORE_DEBUG(PSTR("MyS: STOP READ PWSENSOR\n"));
  CORE_DEBUG(PSTR("MyS: START READ INT TEMP SENSOR\n"));
  itemp();
  CORE_DEBUG(PSTR("MyS: STOP READ INT TEMP SENSOR\n"));
}


void sendData() {

  if (flag_nogateway_mode == false) {
    transportReInitialise();
    wait(30);
    if (tch == true) {
      check = send(msgTemp.setDestination(0).set(celsius, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        change = true;
      } else {
        tch = false;
      }
      if (check == true) {
        err_delivery_beat = 0;
        if (flag_nogateway_mode == true) {
          flag_nogateway_mode = false;
          CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
          err_delivery_beat = 0;
        }
        CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
        CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
      } else {
        _transportSM.failedUplinkTransmissions = 0;
        if (err_delivery_beat < 6) {
          err_delivery_beat++;
        }
        if (err_delivery_beat == 5) {
          if (flag_nogateway_mode == false) {
            happy_node_mode();
            gateway_fail();
            CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
          }
        }
      }
    }


    if (hch == true) {
      check = send(msgHum.setDestination(0).set(readyCalcSend, 0));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        change = true;
      } else {
        hch = false;
      }

      if (check == true) {
        err_delivery_beat = 0;
        if (flag_nogateway_mode == true) {
          flag_nogateway_mode = false;
          CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
          err_delivery_beat = 0;
        }
        CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
        CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
      } else {
        _transportSM.failedUplinkTransmissions = 0;
        if (err_delivery_beat < 6) {
          err_delivery_beat++;
        }
        if (err_delivery_beat == 5) {
          if (flag_nogateway_mode == false) {
            happy_node_mode();
            gateway_fail();
            CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
          }
        }
      }
    }


    if (bch == true) {
      batLevSend();
      bch = false;
    }
    if (vch == true) {
      sendBattVoltage();
      vch = false;
    }

    transportDisable();
  } else {
    tch = false;
    hch = false;
    bch = false;
    vch = false;
  }
  wdt_nrfReset();
}


//############################################## CONFIG #####################################################


void timeConf() {

  SLEEP_TIME = (timeSend * minuteT / SLEEP_TIME_WDT);

  BATT_TIME = battSend * 60 / timeSend;

  cpNom = 60 / timeSend;

  CORE_DEBUG(PSTR("SLEEP_TIME: %d\n"), SLEEP_TIME);
}


void sendConfig() {
  check = send(setTimeSendMsg.set(timeSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setTimeSendMsg.set(timeSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(setBattSendMsg.set(battSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setBattSendMsg.set(battSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(setColor.set(colorPrint));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setColor.set(colorPrint));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(setFonts.set(altfont));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setFonts.set(altfont));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
/*
  check = send(setBind.set(bindSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setBind.set(bindSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
*/
  check = send(setThreshHum.set(threshholdHum));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setThreshHum.set(threshholdHum));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
}


void sendResetReason() {
  String reason;
#ifdef MY_RESET_REASON_TEXT
  if (NRF_POWER->RESETREAS == 0) reason = "POWER_ON";
  else {
    if (NRF_POWER->RESETREAS & (1UL << 0)) reason += "PIN_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 1)) reason += "WDT ";
    if (NRF_POWER->RESETREAS & (1UL << 2)) reason += "SOFT_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 3)) reason += "LOCKUP";
    if (NRF_POWER->RESETREAS & (1UL << 16)) reason += "WAKEUP_GPIO ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "LPCOMP ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "WAKEUP_DEBUG";
  }
#else
  reason = NRF_POWER->RESETREAS;
#endif

  check = send(sendMsg.set(reason.c_str()));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(sendMsg.set(reason.c_str()));
    if (check == false) {
      wait(shortWait * 3);
      _transportSM.failedUplinkTransmissions = 0;
      check = send(sendMsg.set(reason.c_str()));
      wait(shortWait);
    }
  }
  if (check) NRF_POWER->RESETREAS = (0xFFFFFFFF);
}


void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(200, 255);
  hwReboot();
}


//################################################ INTERRUPTS #################################################

//For external interrupts to work, you must add __attribute__ ((weak)) in the Winterrups.c file before the void GPIOTE_IRQHandler (){} function.
//File path - %LOCALAPPDATA%\Arduino15\packages\sandeepmistry\hardware\nRF5\0.6.0\cores\nRF5


void interrupt_Init() {
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  app_gpiote_user_register(&m_gpiote_user_id, PIN_BUTTON_MASK, PIN_BUTTON_MASK, gpiote_event_handler);
  app_gpiote_user_enable(m_gpiote_user_id);
  buttIntStatus = 0;
}


void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2);

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
    if (buttIntStatus == 0) {
      buttIntStatus = PIN_BUTTON;
    }
  }
}


//############################################## RECEIVE CONF ##################################################

void receive(const MyMessage & message)
{
  if (configMode == true) {
    if (message.sensor == SET_TIME_SEND_ID) {
      if (message.type == V_VAR1) {
        timeSend = message.getByte();
        if (timeSend > 240) {
          timeSend = 240;
        }
        if (timeSend < 1) {
          timeSend = 15;
        }
        saveState(102, timeSend);
        wait(shortWait);
        send(setTimeSendMsg.set(timeSend));
        wait(shortWait);
        transportDisable();
        reportTimeInk();
        configMode = false;
        change = true;
        timeConf();
        sleepTimeCount = SLEEP_TIME;
      }
    }

    if (message.sensor == SET_THRESHOLD_HUM_ID) {
      if (message.type == V_VAR1) {
        threshholdHum = message.getByte();
        if (threshholdHum > 99) {
          threshholdHum = 99;
        }
        if (threshholdHum == 0) {
          threshholdHum = 1;
        }

        saveState(109, threshholdHum);
        wait(shortWait);
        send(setThreshHum.set(threshholdHum));
        wait(shortWait);
        transportDisable();
        reportThreshHumInk();
        configMode = false;
        change = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }

    if (message.sensor == SET_BATT_SEND_ID) {
      if (message.type == V_VAR1) {
        battSend = message.getByte();
        if (battSend > 24) {
          battSend = 24;
        }
        if (battSend < 1) {
          battSend = 24;
        }
        saveState(103, battSend);
        wait(shortWait);
        send(setBattSendMsg.set(battSend));
        wait(shortWait);
        transportDisable();
        reportBattInk();
        configMode = false;
        change = true;
        timeConf();
        sleepTimeCount = SLEEP_TIME;
      }
    }
/*
    if (message.sensor == SET_BIND_ID) {
      if (message.type == V_VAR1) {
        bindSend = message.getByte();
        if (bindSend > 254) {
          bindSend = 0;
        }
        saveState(108, bindSend);
        wait(shortWait);
        send(setBind.set(bindSend));
        wait(shortWait);
        transportDisable();
        reportBindInk();
        configMode = false;
        change = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }
*/
    if (message.sensor == SET_COLOR_ID) {
      if (message.type == V_VAR1) {
        colorPrint = message.getBool();
        colorChange(colorPrint);
        wait(shortWait);
        send(setColor.set(colorPrint));
        wait(shortWait);
        transportDisable();
        configMode = false;
        change = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }

    if (message.sensor == SET_FONTS_ID) {
      if (message.type == V_VAR1) {
        altfont = message.getBool();
        if (altfont > 1) {
          altfont = 1;
        }
        saveState(107, altfont);
        wait(shortWait);
        send(setFonts.set(altfont));
        wait(shortWait);
        transportDisable();
        configMode = false;
        change = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }
  }
}


//############################################## HAPPY MODE #####################################################


void happy_init() {

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(200) == 0) {
    saveState(200, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(200));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 25000;
  } else {
    mtwr = 8000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}


void config_Happy_node() {
  if (mtwr == 25000) {
    myid = getNodeId();
    saveState(200, myid);
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      old_mypar = mypar;
      saveState(201, mypar);
      saveState(202, _transportConfig.distanceGW);
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 7;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
  if (mtwr != 25000) {
    myid = getNodeId();
    if (myid != loadState(200)) {
      saveState(200, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      if (mypar != loadState(201)) {
        saveState(201, mypar);
      }
      if (_transportConfig.distanceGW != loadState(202)) {
        saveState(202, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 6;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(900, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == C_INTERNAL) {
      if (_msg.type == 8) {
        Ack_FP = true;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == true) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = false;
    transportSwitchSM(stParent);
    flag_nogateway_mode = false;
    flag_find_parent_process = true;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    nosleep = false;
    transportDisable();
  }
}


void find_parent_process() {
  flag_update_transport_param = true;
  flag_find_parent_process = false;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
}


void gateway_fail() {
  flag_nogateway_mode = true;
  flag_update_transport_param = false;
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0u;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = false;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = true;
    }
  }
}


void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  if (mypar != loadState(201))
  {
    saveState(201, mypar);
  }
  if (_transportConfig.distanceGW != loadState(202))
  {
    saveState(202, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(50);
  flag_update_transport_param = false;
  sleepTimeCount = SLEEP_TIME;
  BATT_COUNT = BATT_TIME;
}


void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}


void preHwInit() {
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_SENS_PWS, INPUT);
  pinMode(PIN_POWER_PWS, OUTPUT);
  digitalWrite(PIN_POWER_PWS, HIGH);
}

void colorChange(bool flag) {
  if (flag == true) {
    colorPrint = false;
    opposite_colorPrint = true;
  } else {
    colorPrint = true;
    opposite_colorPrint = false;
  }
  saveState(106, flag);
}


void before()
{
  //########################################## CONFIG MCU ###############################################

#ifdef DCPOWER
  NRF_POWER->DCDCEN = 1;
#endif
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;
  NRF_TWIM0 ->ENABLE = 0;
  NRF_TWIS0 ->ENABLE = 0;
#ifndef MY_DEBUG
  NRF_UART0->ENABLE = 0;
#endif
#ifdef NRF840
  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;
  NRF_RADIO->TXPOWER = 0x8UL;
#endif

  analogReadResolution(12);
  //analogReference(AR_VDD4);

  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  NRF_TEMP->TASKS_STOP;
  NRF_TEMP->EVENTS_DATARDY = 0;
  NRF_TEMP->INTENSET = 1;


  //########################################## INIT HAPPY ##############################################

  happy_init();

  //########################################## CONFIG PROG ###############################################

  timeSend = loadState(102);  // сохранение в памяти интервала отправки данных с сенсора sht/si, максимано 60 минут, если 0 то отправку не делает, только обновляет инфо на экране
  if (timeSend > 240) {
    timeSend = 15;
    saveState(102, timeSend);
  }

  if (timeSend < 1) {
    timeSend = 15;
    saveState(102, timeSend);
  }

  //timeSend = 1; // для теста, 1 минута

  threshholdHum = loadState(109);  // сохранение в памяти интервала отправки данных с сенсора sht/si, максимано 60 минут, если 0 то отправку не делает, только обновляет инфо на экране
  if (threshholdHum == 255 || threshholdHum == 0) {
    threshholdHum = 40;
    saveState(109, threshholdHum);
  }

  //threshholdHum = 40; // для теста

  battSend = loadState(103);  // сохранение в памяти интервала отправки данных о заряде батареи и качества сигнала, максимано 24 часа, если 0 то отправку не делает, только обновляет инфо на экране
  if (battSend > 24) {
    battSend = 6;
    saveState(103, battSend);
  }
  //battSend = 1; // для теста, 1 час

  if (loadState(106) > 1) {
    saveState(106, 0);
  }
  colorChange(loadState(106));
  //colorChange(false); // для теста, true или false

  if (loadState(107) > 1) {
    saveState(107, 0);
  }
  altfont = loadState(107);
  //altfont = true;// для теста, true или false
/*
  bindSend = loadState(108);
  if (bindSend > 254) {
    bindSend = 0;
    saveState(108, bindSend);
  }
  //bindSend = 100; // id 100
*/
  timeConf();

  //########################################## EINK INIT ###############################################
  Config_Init();
  EPD_Init();
  paith.Clear(opposite_colorPrint);
  DrawImageWH(&paith, 0, 0, LOGO, 80, 128, colorPrint);
  EPD_Display_Image((UBYTE *)image_temp);
  hwSleep(2000);

#ifdef ESPECIALLY
  //EPD_Clear();
  EPD_Clear();
  EPD_Part_Init();
  paith.Clear(opposite_colorPrint);
  DrawImageWH(&paith, 0, 0, Especially, 80, 128, colorPrint);
  EPD_Display_Image((UBYTE *)image_temp);
  hwSleep(2500);
#endif

  EPD_Clear();
  EPD_Part_Init();
  paith.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paith, 0, 30, CONECT, 80, 35, colorPrint);
#endif

#ifdef LANG_EN
  DrawImageWH(&paith, 0, 30, CONECTEN, 80, 35, colorPrint);
#endif
  EPD_Display_Image((UBYTE *)image_temp);
  EPD_Sleep();
}


void presentation()
{
  check = sendSketchInfo(SN, SV);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = sendSketchInfo(SN, SV);
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(TEMP_ID, S_TEMP, "TEMPERATURE");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(TEMP_ID, S_TEMP, "TEMPERATURE");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(HUM_ID, S_HUM, "HUMIDITY");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(HUM_ID, S_HUM, "HUMIDITY");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_TIME_SEND_ID, S_CUSTOM, "H&T SEND INTERVAL | M");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_TIME_SEND_ID, S_CUSTOM, "H&T SEND INTERVAL | M");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_FONTS_ID, S_CUSTOM, "ALTERNATIVE FONT 0/1");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_FONTS_ID, S_CUSTOM, "ALTERNATIVE FONT 0/1");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
/*
  check = present(SET_BIND_ID, S_CUSTOM, "MASTER DEVICE ID");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_BIND_ID, S_CUSTOM, "MASTER DEVICE ID");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
*/
  check = present(SET_THRESHOLD_HUM_ID, S_CUSTOM, "NOTIFICATION OF IRRIGATION");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_THRESHOLD_HUM_ID, S_CUSTOM, "NOTIFICATION OF IRRIGATION");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  wait(shortWait * 2);
  sendConfig();
  wait(shortWait);
}


void setup() {
  CORE_DEBUG(PSTR("MyS: CONFIG HAPPY NODE\n"));
  config_Happy_node();
  //CORE_DEBUG(PSTR("MyS: READ BATTERY VOLTAGE\n"));
  //readBatt();
  CORE_DEBUG(PSTR("MyS: SEND CONFIG PARAMETERS\n"));
  if (flag_nogateway_mode == false) {
    wait(shortWait);
    sendResetReason();
  }
  CORE_DEBUG(PSTR("MyS: INTERRUPT CONFIG\n"));
  interrupt_Init();
  CORE_DEBUG(PSTR("MyS: DISABLE RADIO\n"));
  transportDisable();
  sleepTimeCount = SLEEP_TIME;

  wait(shortWait);
  CORE_DEBUG(PSTR("MyS: WDT ENABLE\n"));
  wdt_init();
}


void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_find_parent_process == true) {
      find_parent_process();
    }
    if (flag_nogateway_mode == false) {
      if (configMode == false) {
        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
            wdt_nrfReset();
            EPD_Part_Init();
            button_flag = true;
            previousMillis = millis();
          }

          if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
            if ((millis() - previousMillis > 1500) && (millis() - previousMillis <= 5500)) {
              if (updateink1 == false) {
                einkZeropush();
                updateink1 = true;
              }

            }
            if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 6500)) {
              if (updateinkclear == false) {
                EPD_Clear3();
                updateinkclear = true;
              }

            }
            if ((millis() - previousMillis > 6500) && (millis() - previousMillis <= 9500)) {
              if (updateink2 == false) {
                einkOnepush();
                updateink2 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 10500)) {
              if (updateinkclear == false) {
                EPD_Clear3();
                updateinkclear = true;
              }

            }
            if ((millis() - previousMillis > 10500) && (millis() - previousMillis <= 13500)) {
              if (updateink3 == false) {
                einkTwopush();
                updateink3 = true;
                updateinkclear = false;
              }
            }
            if (millis() - previousMillis > 13500) {
              if (updateinkclear == false) {
                EPD_Clear3();
                updateinkclear = true;
              }

            }
          }

          if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {

            if ((millis() - previousMillis > 1500 && millis() - previousMillis <= 5500) && button_flag == true)
            {
              wdt_nrfReset();
              einkZeroend();
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              presentation();
              change = true;
              sleepTimeCount = SLEEP_TIME;
            }
            if ((millis() - previousMillis > 6500 && millis() - previousMillis <= 9500) && button_flag == true)
            {
              wdt_nrfReset();
              einkOneend();
              reseteinkset();
              configMode = true;
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              wait(shortWait);
              NRF5_ESB_startListening();
              wait(shortWait);
              configMillis = millis();
            }
            
            if ((millis() - previousMillis > 10500) && (millis() - previousMillis <= 13500) && (button_flag == true))
            {
              einkTwoend();
              new_device();
            }
            if (((millis() - previousMillis < 1500) || ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 6500)) || ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 10500)) || (millis() - previousMillis > 13500) ) && (button_flag == true))
            {
              wdt_nrfReset();
              change = true;
              sleepTimeCount = SLEEP_TIME;
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
            }
          }
          wdt_nrfReset();
        } else {
          sleepTimeCount++;
          nosleep = false;
          wait(10);
          if (!nosleep) {
            nosleep = false;
            wait(10);
          }
          wdt_nrfReset();
          if (sleepTimeCount >= SLEEP_TIME) {
            sleepTimeCount = 0;
            readData();
            if (change == true) {
              change = false;
              sendData();
              wait(shortWait);
              EPD_Part_Init();
              wait(shortWait * 2);
              convert(celsius, readyCalc);
              wait(shortWait);
              EPD_Sleep();
              wait(shortWait * 2);
            }
          }
          nosleep = false;
        }
      } else {
        if (millis() - configMillis > 20000) {
          configMode = false;
          button_flag = false;
          buttIntStatus = 0;
          transportDisable();
          change = true;
          sleepTimeCount = SLEEP_TIME;
        }
        wdt_nrfReset();
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
          button_flag = true;
          previousMillis = millis();
        }

        if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
          if ((millis() - previousMillis > 1500) && (millis() - previousMillis <= 5500)) {
            if (updateink1 == false) {
              EPD_Part_Init();
              einkZeropush();
              updateink1 = true;
            }
          }
          if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 6500)) {
            if (updateinkclear == false) {
              EPD_Clear3();
              updateinkclear = true;
            }
          }
          if ((millis() - previousMillis > 6500) && (millis() - previousMillis <= 9500)) {
            if (updateink2 == false) {
              einkTwopush();
              updateink2 = true;
              updateinkclear = false;
            }
          }
          if (millis() - previousMillis > 9500) {
            if (updateinkclear == false) {
              EPD_Clear3();
              updateinkclear = true;
            }

          }
        }

        if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {
          if ((millis() - previousMillis > 1500 && millis() - previousMillis <= 5500) && button_flag == true)
          {
            einkZeroend();
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
            transportReInitialise();
            check_parent();
            cpCount = 0;
            change = true;
            sleepTimeCount = SLEEP_TIME;
          }
        
          if ((millis() - previousMillis > 6500) && (millis() - previousMillis <= 9500) && (button_flag == true))
          {
            einkTwoend();
            new_device();
          }
          if ((millis() - previousMillis < 1500) ||  ( ( millis() - previousMillis > 5500 && millis() - previousMillis <= 6500 ) || (millis() - previousMillis > 9500)) && button_flag == true)
          {
            change = true;
            sleepTimeCount = SLEEP_TIME;
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
          }
        }
        wdt_nrfReset();
      } else {
        sleepTimeCount++;
        if (sleepTimeCount >= SLEEP_TIME) {
          readData();
          if (change == true) {
            change = false;
            sendData();
            EPD_Part_Init();
            convert(celsius, readyCalc);
            wait(shortWait);
            EPD_Sleep();
            wait(shortWait * 2);
          }
          sleepTimeCount = 0;
          cpCount++;
          if (cpCount >= cpNom) {
            transportReInitialise();
            check_parent();
            cpCount = 0;
          }
        }
        if ((cpCount < cpNom) && (flag_nogateway_mode == true)) {
          nosleep = false;
        }
      }
    }
  }

  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(201);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(202);
    mypar = _transportConfig.parentNodeId;
    nosleep = false;
    err_delivery_beat = 6;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == false) {
    wdt_nrfReset();
    uint32_t periodTimer;
    uint32_t quotientTimer;
    stopTimer = millis();
    if (stopTimer < startTimer) {
      periodTimer = (4294967295 - startTimer) + stopTimer;
    } else {
      periodTimer = stopTimer - startTimer;
    }
    if (periodTimer >= SLEEP_TIME_WDT) {
      quotientTimer = periodTimer / SLEEP_TIME_WDT;
      if (quotientTimer == 0) {
        PRECISION_TIME_WDT = periodTimer - SLEEP_TIME_WDT;
        sleepTimeCount++;
      } else {
        PRECISION_TIME_WDT = periodTimer - SLEEP_TIME_WDT * quotientTimer;
        sleepTimeCount = sleepTimeCount + quotientTimer;
      }
    } else {
      PRECISION_TIME_WDT = SLEEP_TIME_WDT - periodTimer;
    }

    hwSleep(PRECISION_TIME_WDT);
    startTimer = millis();
    nosleep = true;
  }
  wdt_nrfReset();
}


static __INLINE void wdt_init(void)
{
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = 30 * 32768;
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}

static __INLINE void wdt_nrfReset() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}


void itemp() {
  for (byte i = 0; i < 9; i++) {
    NRF_TEMP->TASKS_START = 1;
    while (!(NRF_TEMP->EVENTS_DATARDY)) {}
    rawTemperature = NRF_TEMP->TEMP;
    float celsius2 = (((float)rawTemperature) / 4.0);
    celsius = celsius + celsius2;
    wait(5);
  }
  celsius = celsius / 9;

  if (abs(celsius - lastcelsius) >= celsiusThreshold) {
    change = true;
    tch = true;
    lastcelsius = celsius;
  }

}

void msm () {
  digitalWrite(PIN_POWER_PWS, LOW);                           // Turning on the power of the soil moisture meter
  hwSleep(1000);  // Wait
  wait(100);
  readBatt();
  CORE_DEBUG(PSTR("MyS: SENSOR DEBAG START\n"));
  CORE_DEBUG(PSTR("MyS: BatteryVoltage 1 = %d\n"), batteryVoltage);
  uint16_t m_s_m = 0;
  uint16_t m_s_m2 = 0;
  for (byte i = 0; i < 5; i++) {
    m_s_m = analogRead(PIN_SENS_PWS);
    CORE_DEBUG(PSTR("MyS: m_s_m = %d\n"), m_s_m);
    m_s_m2 = m_s_m2 + m_s_m;
    CORE_DEBUG(PSTR("MyS: m_s_m2 = %d\n"), m_s_m2);
    wait(30);
  }
  m_s_m = m_s_m2 / 5;
  digitalWrite(PIN_POWER_PWS, HIGH);                          // Turning off the power of the soil moisture meter

  uint16_t ready_m_s_m = m_s_m;

  if (batteryVoltage > nomV) {
    batteryVoltage = nomV;
  }
  differenceV = nomV - batteryVoltage;
  differenceAD1 = (float)differenceV * 1.3;
  differenceAD2 = (float)differenceV * 0.8;
  maxiM = maxi - round(differenceAD1);
  miniM = mini - round(differenceAD2);

  if (ready_m_s_m <= maxiM) {
    m_s_m_calc = map(ready_m_s_m, miniM, maxiM, 99, 0);
    if (m_s_m_calc > 99) {
      m_s_m_calc = 99;
    }
  } else {
    m_s_m_calc = 0;
  }

  readyCalc = m_s_m_calc;
  if ((readyCalc + 3 > lastReadyCalc) || (readyCalc < lastReadyCalc)) {
    if ((readyCalc - lastReadyCalc) != calcThreshold) {
      if (abs(readyCalc - lastReadyCalc) >= calcThreshold) {
        change = true;
        hch = true;
        lastReadyCalc = readyCalc;
        readyCalcSend = (float)readyCalc;
      }
    }
  }

  CORE_DEBUG(PSTR("MyS: maxiM = %d\n"), maxiM);
  CORE_DEBUG(PSTR("MyS: miniM = %d\n"), miniM);
  CORE_DEBUG(PSTR("MyS: m_s_m = %d\n"), m_s_m);
  CORE_DEBUG(PSTR("MyS: ready_m_s_m = %d\n"), ready_m_s_m);
  CORE_DEBUG(PSTR("MyS: m_s_m_calc = %d\n"), m_s_m_calc);
  CORE_DEBUG(PSTR("MyS: readyCalc = %d\n"), readyCalc);
  CORE_DEBUG(PSTR("MyS: SENSOR DEBAG END\n"));
}
