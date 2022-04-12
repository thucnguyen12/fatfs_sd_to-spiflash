#include "app_debug.h"
#include <stdarg.h>
#include "stdio.h"
#include <string.h>
#include <ctype.h>

#if DEBUG_ISR_ENABLE
#include "lwrb/lwrb.h"
#endif

#define NUMBER_OF_DEBUG_PORT    4

static app_debug_output_cb_t m_write_cb[NUMBER_OF_DEBUG_PORT];
static uint8_t number_of_callback = 0;
static app_debug_get_timestamp_ms_cb_t m_get_ms;
static app_debug_lock_cb_t m_lock_cb;

#if DEBUG_ISR_ENABLE
static lwrb_t m_ringbuffer_debug_isr;
static uint8_t m_isr_buffer_size[DEBUG_ISR_RINGBUFFER_SIZE];
#endif


void app_debug_init(app_debug_get_timestamp_ms_cb_t get_ms, app_debug_lock_cb_t lock_cb)
{
    m_get_ms = get_ms;
    m_lock_cb = lock_cb;
#if DEBUG_ISR_ENABLE
    lwrb_init(&m_ringbuffer_debug_isr, &m_isr_buffer_size, DEBUG_ISR_RINGBUFFER_SIZE);
#endif
}

uint32_t app_debug_get_ms(void)
{
    if (m_get_ms)
    {
        return m_get_ms();
    }
    return 0;
}

void app_debug_print_nothing(const char *fmt, ...)
{

}

void app_debug_put_byte(uint8_t data)
{
    for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT; index++)
    {
        if (m_write_cb[index])
        {
            m_write_cb[index](&data, 1);
        }
    }
}

void put_string(char* s)
{
    if (s)
    {
        while (*s != '\0')
        {
            app_debug_put_byte((*s++));
        }	
    }
    else
    {    
        app_debug_put_byte('N');
        app_debug_put_byte('U');
        app_debug_put_byte('L');
        app_debug_put_byte('L');
    }
}

#if DEBUG_ISR_ENABLE
static inline void app_debug_put_byte_in_isr(uint8_t data)
{
    lwrb_write(&m_ringbuffer_debug_isr, &data, 1);
}

static inline void put_string_in_isr(char* s)
{
    if (s)
    {
        while (*s != '\0')
        {
            app_debug_put_byte_in_isr((*s++));
        }	
    }
    else
    {    
        char *null_ptr = "NULL";
        lwrb_write(&m_ringbuffer_debug_isr, null_ptr, 4);
    }
}


static void custom_itoa_isr(int32_t val, int32_t radix, int32_t len)
{
    uint8_t c, r, sgn = 0, pad = ' ';    
    uint8_t s[20], i = 0;    
    uint32_t v;

    if (radix == 0)
    {
        return;
    }
    if (radix < 0)
    {    
        radix = -radix;    
        if (val < 0) 
        {
    
            val = -val;    
            sgn = '-';    
        }    
    }    
    v = val;    
    r = radix;    
    if (len < 0) 
    {    
        len = -len;    
        pad = '0';    
    }    

    if (len > 20) 
    {
        return;    
    }

    do 
    {    
        c = (uint8_t)(v % r);    
        if (c >= 10) c += 7;    
        c += '0';    
        s[i++] = c;    
        v /= r;    
    } while (v);   

    if (sgn) 
    {
        s[i++] = sgn;   
    } 

    while (i < len)   
    { 
        s[i++] = pad;   
    }

    do
    {    
        app_debug_put_byte_in_isr(s[--i]);   
    } while (i);
}

#endif

static void custom_itoa(int32_t val, int32_t radix, int32_t len)
{
    uint8_t c, r, sgn = 0, pad = ' ';    
    uint8_t s[20], i = 0;    
    uint32_t v;

    if (radix == 0)
    {
        return;
    }
    if (radix < 0)
    {    
        radix = -radix;    
        if (val < 0) 
        {
    
            val = -val;    
            sgn = '-';    
        }    
    }    
    v = val;    
    r = radix;    
    if (len < 0) 
    {    
        len = -len;    
        pad = '0';    
    }    

    if (len > 20) 
    {
        return;    
    }

    do 
    {    
        c = (uint8_t)(v % r);    
        if (c >= 10) c += 7;    
        c += '0';    
        s[i++] = c;    
        v /= r;    
    } while (v);   

    if (sgn) 
    {
        s[i++] = sgn;   
    } 

    while (i < len)   
    { 
        s[i++] = pad;   
    }

    do
    {    
        app_debug_put_byte(s[--i]);   
    } while (i);
}


void app_debug_print_raw(const char *fmt, ...)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
#if 0
    int32_t n;
    char *p = &m_debug_buffer[0];
    int size = SEGGER_RTT_PRINTF_BUFFER_SIZE;
    int time_stamp_size;

    p += sprintf(m_debug_buffer, "<%u>: ", app_debug_get_ms());
    time_stamp_size = (p-m_debug_buffer);
    size -= time_stamp_size;
    va_list args;

    va_start (args, fmt);
    n = vsnprintf(p, size, fmt, args);
    if (n > (int)size) 
    {
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT; index++)
        {
            if (m_write_cb[index])
            {
                m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }
        
    } 
    else if (n > 0) 
    {
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT; index++)
        {
            if (m_write_cb[index] != NULL)
            {
                m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }

    }
    va_end(args);
#else

    va_list arp;
    int32_t d, r, w, s, l;
    va_start(arp, fmt);

    while ((d = *fmt++) != 0) 
    {
        if (d != '%')
        {    
            app_debug_put_byte(d); 
            continue;    
        }
        const char *next = fmt;
        if (*next == '%')
        {
            fmt++;
            app_debug_put_byte('%'); 
            continue;
        }
            

        d = *fmt++; 
        w = r = s = l = 0;    
        
        if (d == '0') 
        {    
            d = *fmt++; s = 1;    
        }    

        while ((d >= '0') && (d <= '9')) 
        {    
            w += w * 10 + (d - '0');    
            d = *fmt++;    
        }    

        if (s) 
        {
            w = -w;    
        }

        if (d == 'l') 
        {    
            l = 1;    
            d = *fmt++;    
        }    

        if (!d) 
        {
            break;
        }

        if (d == 's') 
        {    
            put_string(va_arg(arp, char*));    
            continue;    
        }    

        if (d == 'c') 
        {    
            app_debug_put_byte((char)va_arg(arp, int32_t));    
            continue;    
        }    

        if (d == 'u') r = 10;    
        if (d == 'd') r = -10;    
        if (d == 'X' || d == 'x') r = 16; // 'x' added by mthomas in increase compatibility    
        if (d == 'b') r = 2;    

        if (!r) 
        {   
            break;    
        }

        if (l) 
        {    
            custom_itoa((int32_t)va_arg(arp, int32_t), r, w);    
        } 
        else 
        {    
            if (r > 0)
            {        
                custom_itoa((uint32_t)va_arg(arp, int32_t), r, w); 
            }       
            else
            {        
                custom_itoa((int32_t)va_arg(arp, int32_t), r, w);
            }
        }    
    }  
    va_end(arp);
#endif
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

static void simple_print_hex(uint8_t hex_value)
{
    const char *hex_str = "0123456789ABCDEF";
    app_debug_put_byte(hex_str[(hex_value>>4)&0x0F]);
    app_debug_put_byte(hex_str[(hex_value)&0x0F]);
}

#if DEBUG_ISR_ENABLE
void app_debug_print_isr(const char *fmt, ...)
{
#if 0
    int32_t n;
    char *p = &m_debug_buffer[0];
    int size = SEGGER_RTT_PRINTF_BUFFER_SIZE;
    int time_stamp_size;

    p += sprintf(m_debug_buffer, "<%u>: ", app_debug_get_ms());
    time_stamp_size = (p-m_debug_buffer);
    size -= time_stamp_size;
    va_list args;

    va_start (args, fmt);
    n = vsnprintf(p, size, fmt, args);
    if (n > (int)size) 
    {
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT; index++)
        {
            if (m_write_cb[index])
            {
                m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }
        
    } 
    else if (n > 0) 
    {
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT; index++)
        {
            if (m_write_cb[index] != NULL)
            {
                m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }

    }
    va_end(args);
#else

    va_list arp;
    int32_t d, r, w, s, l;
    va_start(arp, fmt);

    while ((d = *fmt++) != 0) 
    {
        if (d != '%')
        {    
            app_debug_put_byte_in_isr(d); 
            continue;    
        }
        const char *next = fmt;
        if (*next == '%')
        {
            fmt++;
            app_debug_put_byte_in_isr('%'); 
            continue;
        }
            

        d = *fmt++; 
        w = r = s = l = 0;    
        
        if (d == '0') 
        {    
            d = *fmt++; s = 1;    
        }    

        while ((d >= '0') && (d <= '9')) 
        {    
            w += w * 10 + (d - '0');    
            d = *fmt++;    
        }    

        if (s) 
        {
            w = -w;    
        }

        if (d == 'l') 
        {    
            l = 1;    
            d = *fmt++;    
        }    

        if (!d) 
        {
            break;
        }

        if (d == 's') 
        {    
            put_string_in_isr(va_arg(arp, char*));    
            continue;    
        }    

        if (d == 'c') 
        {    
            app_debug_put_byte_in_isr((char)va_arg(arp, int32_t));    
            continue;    
        }    

        if (d == 'u') r = 10;    
        if (d == 'd') r = -10;    
        if (d == 'X' || d == 'x') r = 16; // 'x' added by mthomas in increase compatibility    
        if (d == 'b') r = 2;    

        if (!r) 
        {   
            break;    
        }

        if (l) 
        {    
            custom_itoa_isr((int32_t)va_arg(arp, int32_t), r, w);    
        } 
        else 
        {    
            if (r > 0)
            {        
                custom_itoa_isr((uint32_t)va_arg(arp, int32_t), r, w); 
            }       
            else
            {        
                custom_itoa_isr((int32_t)va_arg(arp, int32_t), r, w);
            }
        }    
    }  
    va_end(arp);
#endif
}

/**
 * @brief           Flush all data in ringbuffer of ISR
 */
void app_debug_isr_ringbuffer_flush(void)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    
    uint8_t tmp;
    while (lwrb_read(&m_ringbuffer_debug_isr, &tmp, 1))
    {
        app_debug_put_byte(tmp);
    }
    
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

#endif /* DEBUG_ISR_ENABLE */

void app_debug_dump(const void* data, int len, const char* message)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    
    uint8_t *p = (uint8_t*)data;
    uint8_t  buffer[16];
    int32_t i_len;
    int32_t i;

    DEBUG_RAW("%s : %u bytes\n", message, len);

    while (len > 0)
    {
        i_len = (len > 16) ? 16 : len;
        memset(buffer, 0, 16);
        memcpy(buffer, p, i_len);
        for (i = 0; i < 16; i++)
        {
            if (i < i_len)
            {
                simple_print_hex(buffer[i]);
                app_debug_put_byte(' ');
            }
            else
            {
                put_string("   ");
            }
        }
        put_string("\t");
        for (i = 0; i < 16; i++)
        {
            if (i < i_len)
            {
                if (isprint(buffer[i]))
                {
                    DEBUG_RAW("%c", (char)buffer[i]);
                }
                else
                {
                    put_string(".");
                }
            }
            else
            {
                put_string(" ");
            }
        }
        put_string("\r\n");
        len -= i_len;
        p += i_len;
    }
    
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}



void app_debug_register_callback_print(app_debug_output_cb_t callback)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    uint8_t callback_exist = 0;                   // Check for existion function pointer in function pointer arry
    if (callback)
    {
        for (uint8_t func_count = 0; func_count < NUMBER_OF_DEBUG_PORT; func_count++)
        {
            if (callback == m_write_cb[func_count])
            {
                // Callback already existed in array
                callback_exist = 1;           
            }
        }

        if (!callback_exist)                    
        {
            m_write_cb[number_of_callback] = callback;
            number_of_callback++;
        }
    }
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

void app_debug_unregister_callback_print(app_debug_output_cb_t callback)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    for (uint8_t func_count = 0; func_count < NUMBER_OF_DEBUG_PORT; func_count++)
    {
        if (callback == m_write_cb[func_count])
        {
            number_of_callback--;            
        }
    }
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

uint32_t sys_get_ms(void)
{
   return uwTick; 
}

uint32_t usart_logger_put(const void *buffer, uint32_t size)
{
    uint8_t *data = (uint8_t*)buffer;
    for (uint32_t i = 0; i < size; i++)
    {
        putChar(USART1, data[i]);
    //    while (0 == LL_USART_IsActiveFlag_TXE(USART1));
    }
    return size;
}

