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
#include <rfm69_support.h>
#include <TaHa.h>

#define KEY_BUF_LEN     8
#define RFM_I2C_ADDR    0x20
#define NBR_RX_BUFFERS  4
#define NBR_TX_BUFFERS  4

#define FREQUENCY     RF69_434MHZ
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define RFM69_FREQ    434.0 
#define RFM69_TX_IVAL_100ms  20;


uint8_t rx_buf[NBR_RX_BUFFERS][RH_RF69_MAX_MESSAGE_LEN];
uint8_t tx_buf[NBR_TX_BUFFERS][RH_RF69_MAX_MESSAGE_LEN];

const uint8_t ROW_PIN[NBR_ROWS] = {ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };  
const uint8_t COL_PIN[NBR_COLS] = {COL0_PIN, COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN };

enum key_states {
  KEY_STATE_IDLE,
  KEY_STATE_PRESSED,
  KEY_STATE_PRESSED_DEBOUNCED,
  KEY_STATE_RELEASED,
  KEY_STATE_READY
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

    radio_init(RFM69_CS,RFM69_INT,RFM69_RST, RFM69_FREQ);
    radio_send_msg("Telmac Wall Terminal 14-segment");

    for (row = 0; row < NBR_ROWS; row++) {
        pinMode(ROW_PIN[row], OUTPUT);
    }
    for (col = 0; col < NBR_COLS; col++) {
        pinMode(COL_PIN[col], INPUT);    
    }
    key_char = '0';
    for (row = 0; row < NBR_ROWS; row++) {      
        for (col = 0; col < NBR_COLS; col++) {
            key_matrix[row][col] = key_char;
            key_char++;
            if (key_char == ':') key_char = 'A';
        }
    }  

    for (key_wr_indx = 0;key_wr_indx <  KEY_BUF_LEN; key_wr_indx++){
        key_buf[key_wr_indx] = 0x00;
    }
    key_wr_indx = 0;
    key_rd_indx = 0;

    key_vector = &key_matrix[0][0];
    scan_keypad_handle.set_interval(10,RUN_RECURRING, ScanKeypad);
    print_key_handle.set_interval(50,RUN_RECURRING, PrintKey);

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
    switch (i2c_event_buf[0]) {
    case RKP_EVENT_SET_KEY_VECTOR:
        if (idx > NBR_KEYS) idx = NBR_KEYS;
        memcpy(key_matrix, &i2c_event_buf[1],idx);
        break;
    }
}

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
   if (key_buf[key_rd_indx] != 0x00) {
       Serial.print (" Key from buffer: "); Serial.print(key_buf[key_rd_indx]);
       Serial.print(" "); Serial.println(key_func_buf[key_rd_indx]);
       cmd = Wire.read();  
       switch (cmd){
       case RKP_REQ_GET_KEY:
           buf[0] = key_buf[key_rd_indx];
           buf[1] = i2c_event_buf[0];
           break;
       default:
           buf[0] = 0xAA;
           buf[1] = 0x55;    
       }
       Wire.write(buf,2);
       key_buf[key_rd_indx] = 0x00;
       key_func_buf[key_rd_indx] = 0x00;
       
       key_rd_indx = ++key_rd_indx & KEY_BUF_MASK;
    } 
    else{
        buf[0] = 0x00; buf[1] = 0x00;
        Wire.write(buf,2); 
    } 
}
  
