
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

//#define NRF840
#define DCPOWER
#define LANG_EN
//#define MY_DEBUG
//#define MY_PASSIVE_NODE
//#define MY_NODE_ID 101
//#define MY_NRF5_ESB_MODE (NRF5_1MBPS)
#define MY_NRF5_ESB_MODE (NRF5_250KBPS)
#define MY_RESET_REASON_TEXT
#define SN "EFEKTA MINI PWS E-Ink"
#define SV "0.15"
#define MY_RADIO_NRF5_ESB
