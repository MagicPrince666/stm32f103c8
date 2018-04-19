#ifndef __RINGBUFFER_H_
#define __RINGBUFFER_H_

#include <stdlib.h>

#define DEFAULT_BUF_SIZE 64

typedef struct cycle_buffer 
{  
    unsigned char buf[DEFAULT_BUF_SIZE];
    unsigned int   size;
    unsigned int   in;
    unsigned int   out;
}RingBuf;  


class RingBuffer {
public:
    void create();

    void destroy(RingBuf *buffer);

    int read(char *target,unsigned int amount);
    int write(char *data,unsigned int length);

protected:
    RingBuf* buffer;
    int empty(RingBuf *buffer);
    int Reset(RingBuf *buffer);
};

#endif