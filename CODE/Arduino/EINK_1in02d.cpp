
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

#include "EINK_1in02d.h"
/**
   full screen update LUT
**/
const unsigned char lut_w1[] =
{
  0x60	, 0x5A	, 0x5A	, 0x00	, 0x00	, 0x01	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,

};
const unsigned char lut_b1[] =
{
  0x90	, 0x5A	, 0x5A	, 0x00	, 0x00	, 0x01	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,


};
/**
   partial screen update LUT
**/
const unsigned char lut_w[] =
{
  0x60	, 0x01	, 0x01	, 0x00	, 0x00	, 0x01	,
  0x80	, 0x0f	, 0x00	, 0x00	, 0x00	, 0x01	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,

};
const unsigned char lut_b[] =
{
  0x90	, 0x01	, 0x01	, 0x00	, 0x00	, 0x01	,
  0x40	, 0x0f	, 0x00	, 0x00	, 0x00	, 0x01	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,
  0x00	, 0x00	, 0x00	, 0x00	, 0x00	, 0x00	,

};




/******************************************************************************
  function :	Software reset
  parameter:
******************************************************************************/
void EPD_Reset(void)
{
  EINK_Digital_Write(EPD_RST_PIN, 0);// Module reset
  EINK_Delay_ms(20);
  EINK_Digital_Write(EPD_RST_PIN, 1);
  EINK_Delay_ms(20);

}

/******************************************************************************
  function :	send command
  parameter:
     command : Command register
******************************************************************************/
static void EPD_SendCommand(UBYTE command)
{
  EINK_Digital_Write(EPD_CS_PIN, 0);
  EINK_Digital_Write(EPD_DC_PIN, 0);// command write

  EINK_SPI_WriteByte(command);
  EINK_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
  function :	send data
  parameter:
    Data : Write data
******************************************************************************/
void EPD_SendData(UBYTE Data)
{
  EINK_Digital_Write(EPD_CS_PIN, 0);
  EINK_Digital_Write(EPD_DC_PIN, 1);	// command write
  EINK_SPI_WriteByte(Data);
  EINK_Digital_Write(EPD_CS_PIN, 1);
}


/******************************************************************************
  function :	LUT download
******************************************************************************/
void EPD_SetFulltReg(void)
{
  unsigned int count;
  EPD_SendCommand(0x23);
  for (count = 0; count < 42; count++)
  {
    EPD_SendData(lut_w1[count]);
  }

  EPD_SendCommand(0x24);
  for (count = 0; count < 42; count++)
  {
    EPD_SendData(lut_b1[count]);
  }
}

/******************************************************************************
  function :	LUT download
******************************************************************************/
void EPD_SetPartReg(void)
{
  unsigned int count;
  EPD_SendCommand(0x23);
  for (count = 0; count < 42; count++) {
    EPD_SendData(lut_w[count]);
  }

  EPD_SendCommand(0x24);
  for (count = 0; count < 42; count++) {
    EPD_SendData(lut_b[count]);
  }
}

/******************************************************************************
  function :	Wait until the busy_pin goes LOW
******************************************************************************/
void EPD_WaitUntilIdle(void)
{
  unsigned char busy;
  do
  {
    EPD_SendCommand(0x71);
    busy = EINK_Digital_Read(EPD_BUSY_PIN);
    busy = !(busy & 0x01);
  }
  while (busy);
  EINK_Delay_ms(50);
}

void EPD_WaitUntilIdle2(void)
{
  unsigned char busy;
  do
  {
    EPD_SendCommand(0x71);
    busy = EINK_Digital_Read(EPD_BUSY_PIN);
    busy = !(busy & 0x01);
  }
  while (busy);
  EINK_Delay_ms(100);
}


/******************************************************************************
  function :	Turn On Display
******************************************************************************/
void EPD_TurnOnDisplay(void)
{
  EPD_SendCommand(0x04);  //power on
  EPD_WaitUntilIdle();
  EPD_SendCommand(0x12);  //Start refreshing the screen
  EPD_WaitUntilIdle();
  EPD_SendCommand(0x02);
  EPD_WaitUntilIdle();    //power off
}

void EPD_TurnOnDisplay2(void)
{
  EPD_SendCommand(0x04);  //power on
  EPD_WaitUntilIdle2();
  EPD_SendCommand(0x12);  //Start refreshing the screen
  EPD_WaitUntilIdle2();
  EPD_SendCommand(0x02);
  EPD_WaitUntilIdle2();    //power off
}

/******************************************************************************
  function :Initialize the e-Paper register
******************************************************************************/
UBYTE EPD_Init(void)
{
  EPD_Reset();

  EPD_SendCommand(0xD2);
  EPD_SendData(0x3F);

  EPD_SendCommand(0x00);
  EPD_SendData (0x6F);  //from outside

  EPD_SendCommand(0x01);  //power setting
  EPD_SendData (0x03);
  EPD_SendData (0x00);
  EPD_SendData (0x2b);
  EPD_SendData (0x2b);

  EPD_SendCommand(0x06);  //Configuring the charge pump
  EPD_SendData(0x3f);

  EPD_SendCommand(0x2A);  //Setting XON and the options of LUT
  EPD_SendData(0x00);
  EPD_SendData(0x00);

  EPD_SendCommand(0x30);  //Set the clock frequency
  EPD_SendData(0x13); //50Hz

  EPD_SendCommand(0x50);  //Set VCOM and data output interval
  EPD_SendData(0x57);

  EPD_SendCommand(0x60);  //Set The non-overlapping period of Gate and Source.
  EPD_SendData(0x22);

  EPD_SendCommand(0x61);  //resolution setting
  EPD_SendData (0x50);    //source 128
  EPD_SendData (0x80);

  EPD_SendCommand(0x82);  //sets VCOM_DC value
  EPD_SendData(0x12);  //-1v

  EPD_SendCommand(0xe3);//Set POWER SAVING
  EPD_SendData(0x33);
  EPD_SetFulltReg();
  EPD_SendCommand(0x04);//Set POWER SAVING

  EPD_WaitUntilIdle();
  return 0;
}

/******************************************************************************
  function :Partial refresh initialization e-paper
******************************************************************************/
void EPD_Part_Init(void)
{
  //EPD_Reset();
  //EPD_Reset();
  EPD_Reset();
  EPD_SendCommand(0xD2);
  EPD_SendData(0x3F);

  EPD_SendCommand(0x00);
  EPD_SendData (0x6F);  //from outside

  EPD_SendCommand(0x01);  //power setting
  EPD_SendData (0x03);
  EPD_SendData (0x00);
  EPD_SendData (0x2b);
  EPD_SendData (0x2b);

  EPD_SendCommand(0x06);  //Configuring the charge pump
  EPD_SendData(0x3f);

  EPD_SendCommand(0x2A);  //Setting XON and the options of LUT
  EPD_SendData(0x00);
  EPD_SendData(0x00);

  EPD_SendCommand(0x30);  //Set the clock frequency
  EPD_SendData(0x17);   // old 0x5

  EPD_SendCommand(0x50);  //Set VCOM and data output interval
  EPD_SendData(0xf2);

  EPD_SendCommand(0x60);  //Set The non-overlapping period of Gate and Source.
  EPD_SendData(0x22);

  EPD_SendCommand(0x82);  //Set VCOM_DC value
  EPD_SendData(0x12);//old -0.1v (0x00)

  EPD_SendCommand(0xe3);//Set POWER SAVING
  EPD_SendData(0x33);

  EPD_SetPartReg();

  EPD_SendCommand(0x04);//Set POWER SAVING

  EPD_WaitUntilIdle();
}

/******************************************************************************
  function :	Clear screen
******************************************************************************/
void EPD_Clear(void)
{
  unsigned int i;
  EPD_SendCommand(0x10);
  for (i = 0; i < 1280; i++) {
    EPD_SendData(0X00);
  }
  EPD_SendCommand(0x13);	       //Transfer new data
  for (i = 0; i < 1280; i++) {
    EPD_SendData(0xff);
  }
  EPD_TurnOnDisplay2();
}

void EPD_Clear2(void)
{
  UWORD Width, Height;
  Width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  Height = EPD_HEIGHT;
  EPD_SendCommand(0x13);
  for (UWORD j = 0; j < Height; j++) {
    for (UWORD i = 0; i < Width; i++) {
      EPD_SendData(0X00);
    }
  }
  EPD_TurnOnDisplay2();


  Width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  Height = EPD_HEIGHT;
  EPD_SendCommand(0x13);
  for (UWORD j = 0; j < Height; j++) {
    for (UWORD i = 0; i < Width; i++) {
      EPD_SendData(0XFF);
    }
  }
  EPD_TurnOnDisplay2();
}

void EPD_Clear3(void)
{
  UWORD Width, Height;

  Width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  Height = EPD_HEIGHT;
  EPD_SendCommand(0x13);
  for (UWORD j = 0; j < Height; j++) {
    for (UWORD i = 0; i < Width; i++) {
      EPD_SendData(0XFF);
    }
  }
  EPD_TurnOnDisplay2();
}



/******************************************************************************
  function :	Sends the image buffer in RAM to e-Paper and displays
  parameter:
    Image :Displayed data
******************************************************************************/
void EPD_Display_Image(UBYTE *Image)
{
  UWORD Width;
  Width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  //EPD_Init();
  EPD_SendCommand(0x10);
  for (UWORD j = 0; j < EPD_HEIGHT; j++) {
    for (UWORD i = 0; i < Width; i++) {
      EPD_SendData(0xff);
    }
  }

  EPD_SendCommand(0x13);
  for (UWORD j = 0; j < EPD_HEIGHT; j++) {
    for (UWORD i = 0; i < Width; i++) {
      EPD_SendData(~pgm_read_byte(&Image[i + j * Width]));
    }
  }
  EPD_TurnOnDisplay();
}



/******************************************************************************
  function :	Enter sleep mode
******************************************************************************/


void EPD_Sleep(void)
{
  EPD_SendCommand(0X50);   //VCOM AND DATA INTERVAL SETTING
  EPD_WaitUntilIdle();
  EPD_SendData(0xf7);  //WBmode:VBDF 17|D7 VBDW 97 VBDB 57    WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
  //EINK_Delay_ms(10);
  EPD_SendCommand(0X02);  	//power off
  EPD_WaitUntilIdle();
  EPD_SendCommand(0X07);  	//deep sleep
  //EINK_Delay_ms(10);
  EPD_SendData(0xA5);
  //EINK_Delay_ms(50);
}

/*
void EPD_Sleep(void)
{
  //EPD_Reset();
    EPD_SendCommand(0X02);    //power off
    EPD_WaitUntilIdle();
    EPD_SendCommand(0X07);    //deep sleep
    EPD_WaitUntilIdle();
    EPD_SendData(0xA5);
    //EINK_Digital_Write(EPD_RST_PIN, 0);// Module reset
}
*/
