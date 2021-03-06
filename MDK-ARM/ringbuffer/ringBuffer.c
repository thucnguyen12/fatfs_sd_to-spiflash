#include "ringBuffer.h"

// var 
uint8_t sizeRead;
uint8_t sizeToWrite;
ringBuffer_t ringBuff;
uint8_t temp [sizeofBuff];
// function

uint8_t getChar(USART_TypeDef * uart)
{
    uint8_t t;
    while (!(uart->SR & (1 << 5))){}  //wait until rxen set
    t = uart ->DR;
    return t;        
}

void putChar(USART_TypeDef * uart , uint8_t c)
{
    uart ->DR = c;
    while (!(uart ->SR & (uint32_t )(1 << 6)));  // wait until tc set
}

void uart_tx(USART_TypeDef * uart,uint8_t *data, uint8_t size)
{

    for (uint8_t i = 0; i < size; i++)
    {
        HAL_Delay (1);
        putChar (uart, *(data+i));
        HAL_Delay (1);
    }
}
void ringBufferInit(ringBuffer_t *ringbuff, uint8_t *tempbuff, uint32_t size)
{
    USART1 ->CR1 |=   1 << 5;               /// idle ENABLE
    ringbuff ->buffer = tempbuff;
    ringbuff ->sizeBuff = size;
    ringbuff ->head = 0;
    ringbuff ->tail = 0;
 
}     
uint8_t  getByteToWriteToRingBuffer (ringBuffer_t* ringBuf)
{
    
    if (ringBuf->head > ringBuf->tail)
    {
        sizeToWrite = ringBuf ->sizeBuff -  (ringBuf ->head - ringBuf->tail);            
    }
    else if (ringBuf->head == ringBuf->tail)
    {
        sizeToWrite = ringBuf ->sizeBuff;
    }
    else
    {
        sizeToWrite = ringBuf->tail - ringBuf->head ;
    }
    return sizeToWrite - 1;
}

uint8_t  getByteFromRingBufferAvailableToRead (ringBuffer_t* ringBuf)
{
    if(ringBuf ->head > ringBuf ->tail)
    {
        sizeRead = ringBuf ->head - ringBuf ->tail;        
    
    }
    else if (ringBuf ->head == ringBuf ->tail)
    {
        sizeRead = 0;
    }
    else
    {
        sizeRead = ringBuf ->sizeBuff - ringBuf ->tail + ringBuf ->head ;
    }
    return sizeRead;
}
uint8_t readFromRingBuffer(ringBuffer_t *ringbuff)
{
    uint8_t data;
    data = ringbuff ->buffer[ringbuff ->tail];
    ringbuff ->tail++;
    if (ringbuff ->tail == ringbuff ->sizeBuff)
    {
        ringbuff ->tail = 0;
    }
    return data;
}

void WriteToRingBuffer(ringBuffer_t *ringbuff, USART_TypeDef * uart)
{
    if (getByteToWriteToRingBuffer(ringbuff))
    {

        ringbuff->buffer [ringbuff ->head] = getChar (uart);
        ringbuff->head++;
        if (ringbuff ->head == ringbuff ->sizeBuff)
        {
            ringbuff ->head = 0;
        }
    }
}
void Get_string (ringBuffer_t *ringbuff, char *buffer)
{
	int index=0;

	while (ringbuff->tail>ringbuff->head)
	{
		if ((ringbuff->buffer[ringbuff->head-1] == '\n')||((ringbuff->head == 0) && (ringbuff->buffer[sizeofBuff-1] == '\n')))
			{
				buffer[index] = readFromRingBuffer(ringbuff);
				index++;
			}
	}
	unsigned int start = ringbuff->tail;
	unsigned int end = (ringbuff->head);
	if (ringbuff->buffer[end-1] == '\n')
	{

		for (unsigned int i=start; i<end; i++)
		{
			buffer[index] = readFromRingBuffer(ringbuff);
			index++;
		}
	}
}
int wait_until (ringBuffer_t *ringbuff, char *string, char*buffertostore)
{
	while (!(getByteFromRingBufferAvailableToRead(ringbuff)));
	int index=0;

	while (ringbuff->tail>ringbuff->head)
	{
		if ((ringbuff->buffer[ringbuff->head-1] == '\n')||((ringbuff->head == 0) && (ringbuff->buffer[sizeofBuff-1] == '\n')))
			{
				buffertostore[index] = readFromRingBuffer(ringbuff);
				index++;
			}
	}

	unsigned int start = ringbuff->tail;
	unsigned int end = (ringbuff->head);
	if (ringbuff->buffer[end-1] == '\n')
	{
		for (unsigned int i=start; i<end; i++)
		{
			buffertostore[index] = readFromRingBuffer(ringbuff);
			index++;
		}
		return 1;
	}
	return 0;
}
