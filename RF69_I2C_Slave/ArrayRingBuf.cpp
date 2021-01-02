#include "ArrayRingBuf.h"
#include "HardwareSerial.h"
    
ArrayRingBuf::ArrayRingBuf(HardwareSerial &print){
    printer = &print; //operate on the adress of print
    printer->begin(9600);
    Initialize();
}
void ArrayRingBuf::Initialize(void){
    head = 0;
    tail = 0;
    counter = 0;
    sema = SEMA_FREE;
    for (uint8_t i = 0; i < NBR_BUFFERS; i++) {
        buf[i][0] = 0;
    }
}
void ArrayRingBuf::AddArray(uint8_t *b_array, uint16_t b_len)
{
    if ( ! IsFull() ){
        b_len = (b_len < RING_BUF_LEN) ? b_len :  RING_BUF_LEN;
        memcpy(buf[head],b_array, b_len);
        //for (uint8_t i=0;i<b_len;i++) printer->print(b_array[i]);
        //printer->println("");
        if(++head >= NBR_BUFFERS) head = 0;
        counter++;
    } else printer->println("Buffer is full");
}


uint16_t ArrayRingBuf::GetArray(uint8_t *b_array, uint16_t b_len){
    if (counter > 0 ){
        memcpy(b_array,buf[tail], b_len);
        if(++tail >= NBR_BUFFERS) tail = 0;
        counter--;          
        return b_len;
    } else
    {
        return 0;
    }    
}

uint8_t ArrayRingBuf::Available(){
    return counter;  
}

uint8_t ArrayRingBuf::Free(){
    return NBR_BUFFERS - counter;  
}

boolean ArrayRingBuf::IsFull(void){
  return (counter < NBR_BUFFERS) ? false : true; 
}

boolean ArrayRingBuf::ReserveSema(void){
    if (sema == SEMA_FREE) {
        sema = SEMA_RESERVED;
        return true; 
    } else {
        return false;
    }
}
void ArrayRingBuf::ReleaseSema(void){
    sema = SEMA_FREE;
}
boolean ArrayRingBuf::SemaAvail(void){
    return (sema == SEMA_FREE) ? true : false;
}


void ArrayRingBuf::PrintBuffers(void){

    printer->print("head = "); printer->print(head);
    printer->print(" tail = "); printer->print(tail);
    printer->print(" counter = "); printer->println(counter);
    for (uint8_t i = 0; i < NBR_BUFFERS; i++){
        printer->print(i); printer->print(": ");
        printer->println((char*)buf[i]);
    }
  
}
