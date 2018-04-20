#include "ringbuffer.h"
#include <string.h>
#include <stdio.h>


#define Min(x, y) ((x) < (y) ? (x) : (y))

//cycle_buffer* buffer;

RingBuffer::RingBuffer(cycle_buffer *buffer)
{
    //memset(buffer, 0, sizeof(RingBuf)); 
    printf("init ring buffer start\n");

    buffer->size = DEFAULT_BUF_SIZE;  
    buffer->in   = 0;
    buffer->out  = 0;  

    printf("init ring buffer end\n");
    //memset(buffer->buf, 0, DEFAULT_BUF_SIZE);
}

RingBuffer::~RingBuffer()
{
    printf("quit ring buffer\n");
}


int RingBuffer::Reset(cycle_buffer *buffer)
{
    if (buffer == NULL)
    {
        return -1;
    }
     
    buffer->in   = 0;
    buffer->out  = 0;
    memset(buffer->buf, 0, buffer->size);

    return 0;
}

int RingBuffer::empty(cycle_buffer *buffer)
{
    return buffer->in == buffer->out;
}

int RingBuffer::write(cycle_buffer *buffer,char *data,unsigned int length)
{
    unsigned int len = 0;

    length = Min(length, buffer->size - buffer->in + buffer->out);  
    len    = Min(length, buffer->size - (buffer->in & (buffer->size - 1)));

 
    memcpy(buffer->buf + (buffer->in & (buffer->size - 1)), data, len);
    memcpy(buffer->buf, data + len, length - len);
 
    buffer->in += length;
 
    return length;
}

int RingBuffer::read(cycle_buffer *buffer,char *target,unsigned int amount)
{
    unsigned int len = 0;  

    amount = Min(amount, buffer->in - buffer->out);
    len    = Min(amount, buffer->size - (buffer->out & (buffer->size - 1)));
 
    memcpy(target, buffer->buf + (buffer->out & (buffer->size - 1)), len);
    memcpy(target + len, buffer->buf, amount - len);
 
    buffer->out += amount;
 
    return amount;
}

