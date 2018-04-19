#include "ringbuffer.h"
#include <string.h>


#define Min(x, y) ((x) < (y) ? (x) : (y))

void RingBuffer::create()
{
    memset(buffer, 0, sizeof(RingBuf)); 
 
    buffer->size = DEFAULT_BUF_SIZE;  
    buffer->in   = 0;
    buffer->out  = 0;  

    memset(buffer->buf, 0, DEFAULT_BUF_SIZE);
}


void RingBuffer::destroy(RingBuf *buffer)
{

}

int RingBuffer::Reset(RingBuf *buffer)
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

int RingBuffer::empty(RingBuf *buffer)
{
    return buffer->in == buffer->out;
}

int RingBuffer::write(char *data,unsigned int length)
{
    unsigned int len = 0;

    length = Min(length, buffer->size - buffer->in + buffer->out);  
    len    = Min(length, buffer->size - (buffer->in & (buffer->size - 1)));

 
    memcpy(buffer->buf + (buffer->in & (buffer->size - 1)), data, len);
    memcpy(buffer->buf, data + len, length - len);
 
    buffer->in += length;
 
    return length;
}

int RingBuffer::read(char *target,unsigned int amount)
{
    unsigned int len = 0;  

    amount = Min(amount, buffer->in - buffer->out);
    len    = Min(amount, buffer->size - (buffer->out & (buffer->size - 1)));
 
    memcpy(target, buffer->buf + (buffer->out & (buffer->size - 1)), len);
    memcpy(target + len, buffer->buf, amount - len);
 
    buffer->out += amount;
 
    return amount;
}

