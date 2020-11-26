#include "main.h"
/*
 * https://www.embedded.com/ring-buffer-basics/
 */
#define NBR_BUFFERS  4
#define RING_BUF_LEN RFM69_BUF_LEN


struct ring_control_struct {
    uint8_t head;
    uint8_t tail;
    boolean full;
    uint8_t buf[NBR_BUFFERS][RH_RF69_MAX_MESSAGE_LEN];
};

class ArrayRingBuf{
public:
    ArrayRingBuf;
    void Initialize(void);
    void AddArray(uint8_t *b_array);
    uint8_t *GetArray();
    uint8_t Available;
private:
    uint8_t head;
    uint8_t tail;
    uint8_t avail;
    buf[NBR_BUFFERS][RING_BUF_LEN];   
}
