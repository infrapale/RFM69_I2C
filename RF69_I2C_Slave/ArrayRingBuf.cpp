#include "ArrayRingBuf.h"
    
ArrayRingBuf::ArrayRingBuf(){
   Initialize();
}
void ArrayRingBuf::Initialize(void){
    head = 0;
    tail = 0;
    counter = 0;
    for (uint8_t i = 0; i < NBR_BUFFERS; i++) {
        buf[i][0] = 0;
    }
}
void ArrayRingBuf::AddArray(uint8_t *b_array, uint16_t b_len)
{
    if ( ! IsFull() ){
        memcpy(buf[head],b_array, b_len);
        if(++head >= NBR_BUFFERS) head = 0;
        counter++;
    }
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
  if (counter < NBR_BUFFERS) 
  {
      return false;
  } 
  else
  {
      return true;  
  }
}
