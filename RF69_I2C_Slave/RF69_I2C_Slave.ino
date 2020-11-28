/**
 ******************************************************************************
 * @file  https://github.com/infrapale/RetroKeyPad_I2C_Slave  
 * @author  Tom Hoglund  infrapale@gmail.com
 *
 * @brief  Arduino C code for the I2C RFM69 Adapter 2020 
 ******************************************************************************
 * @attention
 * Requires TaHa task handler (scheduler)
 * 
 * reference code:
 * https://github.com/adafruit/Adafruit_CircuitPython_TLV493D/blob/master/adafruit_tlv493d.py
 * https://github.com/adafruit/Adafruit_CircuitPython_PCF8591/blob/master/adafruit_pcf8591/pcf8591.py
 *
 ******************************************************************************
 */


#include <Wire.h>
#include "main.h"
#include <rfm69_support.h>
#include "ArrayRingBuf.h"
#include <TaHa.h>

#define KEY_BUF_LEN     8
#define RFM_I2C_ADDR    0x20
#define NBR_BUFFERS     4
#define BUF_LEN         

//#define FREQUENCY     RF69_434MHZ
#define RFM69_CS      10
#define RFM69_INT     2
#define RFM69_RST     9
#define RFM69_FREQ    434.0 
#define RFM69_TX_IVAL_100ms  20;

#define RFM69_RESET    0x01
#define RFM69_CLR_RX   0x02
#define RFM69_CLR_TX   0x03
#define RFM69_SEND_MSG 0x10
#define RFM69_RX_AVAIL 0x40
#define RFM69_TX_FREE  0x50


#define I2C_EVENT_BUFF_LEN RFM69_BUF_LEN


ArrayRingBuf RxData;
ArrayRingBuf TxData;

uint8_t test1[RFM69_BUF_LEN];
uint8_t test2[RFM69_BUF_LEN];


enum key_states {
  KEY_STATE_IDLE,
  KEY_STATE_PRESSED,
};

TaHa scan_keypad_handle;
TaHa print_key_handle;

boolean Debug = true;
static uint8_t reg_addr;
static uint8_t i2c_event_buf[I2C_EVENT_BUFF_LEN];

/**
 * @brief  Scan Keypad,run every 10ms by scheduler  
 * @param  -
 * @retval -
 */

void setup() {
    // wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
    delay(2000);
    while (!Serial); // wait until serial console is open, remove if not tethered to computer
    Serial.begin(9600);
    Serial.println("RF69 I2C_Slave Tom Höglund 2020");

    
    Wire.begin(RFM_I2C_ADDR);      // join i2c bus with address 
    Wire.onRequest(RequestEvent);  // register event
    Wire.onReceive(ReceiveEvent);  // register event    

    strcpy(test1,"ABCDEFGHIJKLMNOPQRSTUVXYZ");
    for (uint8_t i = 0; i < 10; i++) {
        Serial.print((char)test1[i]);
    }
     Serial.println(" !!"); 
    RxData.Initialize(); 
    RxData.AddArray(&test1[0], strlen(test1));
    Serial.println(RxData.Available());
    RxData.GetArray(test2,RFM69_BUF_LEN);
    Serial.print(RxData.Available());
    
    for (uint8_t i = 0; i < 10; i++) {
        Serial.print((char)test2[i]);
    }
    Serial.println(" <<");
     
    radio_init(RFM69_CS,RFM69_INT,RFM69_RST, RFM69_FREQ);
    radio_send_msg("RFM69 I2C Slave");

 
}

void loop() {
  // put your main code here, to run repeatedly:
  scan_keypad_handle.run();
  //print_key_handle.run();
  // RawScan();
  //digitalWrite(ROW_PIN[0], HIGH);
}

/**
 * @brief  I2C Recive Event
 * @param  howMany
 * @param  I2C from master:key value vector 25 char values
 * @retval 
 */

void ReceiveEvent(int howMany)
{ 
    uint8_t idx;
    uint8_t buf_len = 0;
 
    Serial.println("receive event");
    idx = 0;
    memset(i2c_event_buf,0x00,sizeof(i2c_event_buf));
    while(1 < Wire.available()) // loop through all but the last
    {
        if (idx < I2C_EVENT_BUFF_LEN ) {
            i2c_event_buf[idx++] = Wire.read();  
        } else {
          break;
        }
    }
    buf_len = idx - 1;
    switch (i2c_event_buf[0]) {
    case RFM69_RESET:
        radio_init(RFM69_CS,RFM69_INT,RFM69_RST, RFM69_FREQ);
        RxData.Initialize(); 
        TxData.Initialize();     
        break;
    case RFM69_CLR_RX:
        Serial.println("RFM69_CLR_RX");
        RxData.Initialize(); 
        break;
    case RFM69_CLR_TX:
        Serial.println("RFM69_CLR_TX");
        break;
    case RFM69_SEND_MSG:
        if (TxData.IsFull()){
             Serial.println("Tx buffer is full");           
        }
        else
        {
            TxData.AddArray(i2c_event_buf[1],buf_len-1);
            Serial.println("Added to Tx buffer");
            Serial.print("Availablein Tx buffer: "); Serial.println(TxData.Available());
        }
        break;
    }
}

#define RFM69_SEND_MSG 0x10
#define RFM69_RX_AVAIL 0x40


/**
 * @brief  I2C Request Event
 * @param  I2C Cmd: RKP_REQ_GET_KEY 
 * @retval via I2C: [key_value, duration_index]
 */
void RequestEvent()
{
   static char c = '0';
   uint8_t cmd; 
   uint8_t buf[2];
   //Serial.println("request event");
   cmd = Wire.read();  
   switch (cmd){
   case RFM69_RX_AVAIL:
       buf[0] = RxData.Available();
       Wire.write(buf,1);
       break;
   case RFM69_TX_FREE:
       buf[0] = TxData.Free();
       Wire.write(buf,1);
       break;
    default:
       buf[0] = 0xAA;
       buf[1] = 0x55;    
    }
}
