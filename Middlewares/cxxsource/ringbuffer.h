#ifndef __RINGBUFFER_H_
#define __RINGBUFFER_H_

#include <stdlib.h>

#define DEFAULT_BUF_SIZE 64

typedef struct 
{  
    unsigned char buf[DEFAULT_BUF_SIZE];
    unsigned int   size;
    unsigned int   in;
    unsigned int   out;
}cycle_buffer;  


class RingBuffer {
public:
    //cycle_buffer* buffer;
    RingBuffer(cycle_buffer *buffer);
    ~RingBuffer();

    static int read(cycle_buffer *buffer,char *target,unsigned int amount);
    static int write(cycle_buffer *buffer,char *data,unsigned int length);

protected:
    int empty(cycle_buffer *buffer);
    int Reset(cycle_buffer *buffer);
};

#endif