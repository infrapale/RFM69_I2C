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

#define RFM69_RESET       0x01
#define RFM69_CLR_RX      0x02
#define RFM69_CLR_TX      0x03
#define RFM69_SEND_MSG    0x10
#define RFM69_TX_DATA     0x11
#define RFM69_RX_AVAIL    0x40
#define RFM69_RX_LOAD_MSG 0x41
#define RFM69_RX_RD_MSG1  0x42
#define RFM69_RX_RD_MSG2  0x43
#define RFM69_RX_RD_LEN   0x44
#define RFM69_TX_FREE     0x50


#define LED   13

#define I2C_EVENT_BUFF_LEN 32

ArrayRingBuf RxData(Serial);
ArrayRingBuf TxData(Serial);

uint8_t tx_i2c_buf[RFM69_BUF_LEN];
uint8_t rx_i2c_buf[RFM69_BUF_LEN];

uint8_t tx_buf[RFM69_BUF_LEN];

uint8_t tx_sema;
uint8_t rx_sema;

struct i2c_mgmt_struct{
    uint8_t request_command;
    uint8_t rd_len;
    uint8_t rx_bytes;
    uint8_t offset;
    
};

i2c_mgmt_struct i2c_mgmt;


enum key_states {
  KEY_STATE_IDLE,
  KEY_STATE_PRESSED,
};

TaHa radio_send_handle;
TaHa radio_receive_handle;

boolean Debug = true;
static uint8_t reg_addr;
static uint8_t i2c_event_buf[I2C_EVENT_BUFF_LEN+2];
static uint8_t i2c_load_buf[I2C_EVENT_BUFF_LEN+2];

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

    i2c_mgmt.request_command = 0x00;
    i2c_mgmt.rd_len = 0x00;
    i2c_mgmt.offset = 0x00;
    
    Wire.begin(RFM_I2C_ADDR);      // join i2c bus with address 
    Wire.onRequest(RequestEvent);  // register event
    Wire.onReceive(ReceiveEvent);  // register event    
   
   
    radio_init(RFM69_CS,RFM69_INT,RFM69_RST, RFM69_FREQ);
    radio_send_msg("RFM69 I2C Slave");
    
    radio_send_handle.set_interval(2000,RUN_RECURRING, radio_tx_handler);
    radio_receive_handle.set_interval(500,RUN_RECURRING, radio_rx_handler);
}

void loop() {
    radio_send_handle.run();
    radio_receive_handle.run();
  
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
    uint8_t buf_pos;
    uint8_t i;
 
    
    idx = 0;
    memset(i2c_event_buf,0x00,sizeof(i2c_event_buf));
    //Serial.print("howMany= "); Serial.println(howMany);
    while(0 < Wire.available()) 
    {
        if (idx < I2C_EVENT_BUFF_LEN ) {
            b = Wire.read();
            // Serial.print(b);
            i2c_event_buf[idx++] = b;  
        } else {
          break;
        }
    }
    buf_len = idx;
    memset(&i2c_mgmt,0x00,sizeof(i2c_mgmt)); 
    if (buf_len > 0) i2c_mgmt.request_command = i2c_event_buf[0];
    if (buf_len > 1) i2c_mgmt.offset          = i2c_event_buf[1];
    if (buf_len > 2) i2c_mgmt.rd_len          = i2c_event_buf[2];
    
    /*
    Serial.print("request_command="); Serial.print(i2c_mgmt.request_command, HEX);
    Serial.print(", offset =");  Serial.print(i2c_mgmt.offset);
    Serial.print(", rd_len =");  Serial.print(i2c_mgmt.rd_len);
    Serial.print(",  buf_len="); Serial.println(buf_len);
    */
    
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
        //Serial.print(" RFM69_TX_DATA at pos: "); Serial.println(i2c_event_buf[1]);
        buf_pos = i2c_event_buf[1];
        if (buf_pos + buf_len - 2 < RFM69_BUF_LEN){
            memcpy( &tx_i2c_buf[buf_pos] , &i2c_event_buf[2], buf_len-2);
        }
        else {
            Serial.println("too long");
        }
        /*
        for (i=0; i< RFM69_BUF_LEN; i++){
            Serial.print(tx_i2c_buf[i]); Serial.print(", ");
        }
        Serial.println();
        */
        break;
    case RFM69_SEND_MSG:
        Serial.println("RFM69_SEND_MSG:");
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
                
                // TxData.PrintBuffers();
                /*
                for (uint8_t i=0;i<RFM69_BUF_LEN;i++) {
                    Serial.print(char(tx_i2c_buf[i]));
                }
                Serial.print(", Added to Tx buffer, buf_len="); Serial.print(buf_len);
                Serial.print(", Free in  Tx buffer: "); Serial.println(TxData.Free());
                */
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
   uint8_t buf[2];
   uint8_t n_bytes;
   uint8_t indx;
   
   Serial.print("RequestEvent  cmd = ");  Serial.println(i2c_mgmt.request_command,HEX);
   switch (i2c_mgmt.request_command){
   case RFM69_RX_AVAIL:
       buf[0] = RxData.Available();
       Wire.write(buf,1);
       break;
   case RFM69_TX_FREE:
       buf[0] = TxData.Free();
       Wire.write(buf,1);
       break;
    case RFM69_RX_LOAD_MSG:
        memset(i2c_load_buf,0x00,RFM69_BUF_LEN);
        if (RxData.Available() > 0){
            i2c_mgmt.rx_bytes =  RxData.GetArray(i2c_load_buf, RFM69_BUF_LEN);  
        }
        else {
            i2c_mgmt.rx_bytes = 0;
        }
        buf[0] = i2c_mgmt.rx_bytes;
        Wire.write(buf,1);
        break;
   case RFM69_RX_RD_MSG1:
        i2c_mgmt.offset = 0;
        i2c_mgmt.rd_len = 32;
        Wire.write(&i2c_load_buf[i2c_mgmt.offset],i2c_mgmt.rd_len);        
        break;  
   case RFM69_RX_RD_MSG2:
        i2c_mgmt.offset = 32;
        i2c_mgmt.rd_len = RFM69_BUF_LEN - i2c_mgmt.offset ;
        Wire.write(&i2c_load_buf[i2c_mgmt.offset],i2c_mgmt.rd_len);        
        break;         
    case  RFM69_RX_RD_LEN:
        buf[0] = i2c_mgmt.rx_bytes;
        Wire.write(buf,1);
        break;      
    default:
       buf[0] = 0xAA;
       buf[1] = 0x55;
       Wire.write(buf,1);
       break;    
    }
}


void radio_tx_handler(void){
  
    if (TxData.Available()){
        if (TxData.SemaAvail()){
            if (TxData.ReserveSema()){
                digitalWrite(LED, HIGH); 
                Serial.println("Sending data");
                //TxData.PrintBuffers();
                TxData.GetArray(tx_buf,RFM69_BUF_LEN);
                TxData.ReleaseSema();
                radio_send_msg(tx_buf);
                digitalWrite(LED, LOW);
                radio_send_handle.delay_task(2000);
            }
        }    
    } 
}

void radio_rx_handler(void){
    uint8_t i;
    if (radio_check_available_msg()) {
        // Serial.print("! available!");
        // Should be a message for us now   
        char json_msg[RH_RF69_MAX_MESSAGE_LEN];
        uint8_t len;
        len = radio_read_msg(json_msg, RFM69_BUF_LEN);
        for (i=0;i<len;i++) Serial.print(json_msg[i]);
        Serial.println();  
        if (RxData.Free() > 0){
            RxData.AddArray(json_msg,RFM69_BUF_LEN);             
        } else {
            Serial.println("Rx buffers are full, message not added");
        }
        RxData.PrintBuffers();
    } 
}
