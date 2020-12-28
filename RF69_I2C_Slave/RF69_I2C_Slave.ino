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
#define RFM69_TX_DATA  0x11
#define RFM69_RX_AVAIL 0x40
#define RFM69_TX_FREE  0x50


#define LED   13

#define I2C_EVENT_BUFF_LEN 32

ArrayRingBuf RxData(Serial);
ArrayRingBuf TxData(Serial);

uint8_t tx_i2c_buf[RFM69_BUF_LEN];
uint8_t rx_i2c_buf[RFM69_BUF_LEN];

uint8_t tx_buf[RFM69_BUF_LEN];

uint8_t tx_sema;
uint8_t rx_sema;



enum key_states {
  KEY_STATE_IDLE,
  KEY_STATE_PRESSED,
};

TaHa radio_send_handle;

boolean Debug = true;
static uint8_t reg_addr;
static uint8_t i2c_event_buf[I2C_EVENT_BUFF_LEN+2];

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
   
   
    radio_init(RFM69_CS,RFM69_INT,RFM69_RST, RFM69_FREQ);
    radio_send_msg("RFM69 I2C Slave");
    
    radio_send_handle.set_interval(2000,RUN_RECURRING, radio_tx_handler);
}

void loop() {
    radio_send_handle.run();
  
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
    uint8_t b;
 
    
    idx = 0;
    memset(i2c_event_buf,0x00,sizeof(i2c_event_buf));
    while(1 < Wire.available()) // loop through all but the last
    {
        if (idx < I2C_EVENT_BUFF_LEN ) {
            b = Wire.read();
            // Serial.print(b);
            i2c_event_buf[idx++] = b;  
        } else {
          break;
        }
    }
    if (idx > 1) buf_len = idx; 
    
    Serial.print("receive event  buf[0]=");
    Serial.print(i2c_event_buf[0]);
    Serial.print("  buf_len=");
    Serial.println(buf_len);

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
        TxData.Initialize();
        break;
    case RFM69_TX_DATA:
        Serial.println(" RFM69_TX_DATA");
        if (i2c_event_buf[1] + buf_len < RFM69_BUF_LEN){
            memcpy( &tx_i2c_buf[i2c_event_buf[1]] , &i2c_event_buf[2], buf_len-2);
        }
        else {
            Serial.println("too long");
        }
    case RFM69_SEND_MSG:
    
        while (!TxData.SemaAvail()){
            Serial.println("Tx sema wait");
        }
        if (TxData.ReserveSema()){
            if (TxData.IsFull()){
                 Serial.println("Tx buffer is full");           
            }
            else
            {
                TxData.AddArray(tx_i2c_buf,RFM69_BUF_LEN);
                
                TxData.PrintBuffers();
                for (uint8_t i=0;i<RFM69_BUF_LEN;i++) {
                    Serial.print(char(tx_i2c_buf[i]));
                }
                Serial.print(", Added to Tx buffer, buf_len="); Serial.print(buf_len);
                Serial.print(", Free in  Tx buffer: "); Serial.println(TxData.Free());
            }
        } else {
            Serial.println("Failed when reserving TX buffer semaphore");
        }
        TxData.ReleaseSema();
        memset(tx_i2c_buf,0x00,RFM69_BUF_LEN);
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
   
   Serial.print("RequestEvent  cmd = ");
   cmd = Wire.read();  
   Serial.println(cmd);
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


void radio_tx_handler(void){
  
    if (TxData.Available()){
        if (TxData.SemaAvail()){
            if (TxData.ReserveSema()){
                digitalWrite(LED, HIGH); 
                Serial.println("Sending data");
                TxData.PrintBuffers();
                TxData.GetArray(tx_buf,RFM69_BUF_LEN);
                TxData.PrintBuffers();
                TxData.ReleaseSema();
                radio_send_msg(tx_buf);
                digitalWrite(LED, LOW);
                radio_send_handle.delay_task(2000);
            }
        }    
    } 
}
