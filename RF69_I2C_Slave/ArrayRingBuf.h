#include "main.h"
#include <stddef.h>
/*
 * https://www.embedded.com/ring-buffer-basics/
 */
#define NBR_BUFFERS  4
#define RING_BUF_LEN RFM69_BUF_LEN

class ArrayRingBuf
{
  public:
    ArrayRingBuf(void);
    void Initialize(void);
    void AddArray(uint8_t *b_array, uint16_t b_len);
    uint16_t GetArray(uint8_t *b_array, uint16_t b_len);
    uint8_t *GetArray(void);
    uint8_t Available(void);
    uint8_t Free(void);
    boolean IsFull(void);
private:
    uint8_t head;
    uint8_t tail;
    uint8_t counter;
    uint8_t buf[NBR_BUFFERS][RING_BUF_LEN];   
};
