#include "serial3.h"
#include "stm32h7xx_hal.h"
#include "app_uart_fifo.h"

app_uart_fifo_ctx_t	*p_instance3;

#define UART0_TX_BUF_SIZE 256   /**< UART TX buffer size is a power of two */
#define UART0_RX_BUF_SIZE 256   /**< UART RX buffer size is a power of two */

UART_Status_t uart_instance3_Init(app_uart_fifo_ctx_t *p_uart_cxt)
{
	uint32_t err_code;
	p_instance3 = p_uart_cxt;

	APP_UART_FIFO_INIT(p_instance3,
                         UART0_RX_BUF_SIZE,
                         UART0_TX_BUF_SIZE,
						 err_code);
    return (err_code == APP_UART_FIFO_OK) ? UART_OK : UART_ERROR;
}

UART_Status_t uart_instance3_deinit(void)
{
  return UART_OK;
}

int putchar_instance3(uint8_t c)
{
	return (app_uart_put(p_instance3, c) == APP_UART_FIFO_OK) ? UART_OK : UART_ERROR;
}

void puts_uart_instance3(char *s)
{
  while(*s)
  {
    app_uart_put(p_instance3, (uint8_t)*s++);
  }
}

uint32_t write_uart_instance3(uint8_t *s, uint32_t len)
{
  uint32_t i;
  for(i = 0; i < len; i++)
  {
    if(app_uart_put(p_instance3, (uint8_t)(*s)) != APP_UART_FIFO_OK)
    {
      break;
    }
    s++;
  }
  return i;
}

/**
  * @brief get one character from Rx buff usart0
  * @param  none
  * @retval character
  */
int getchar_instance3(uint8_t *c)
{
    return (app_uart_get(p_instance3, c) == APP_UART_FIFO_OK)? UART_OK : UART_ERROR;
}

/**
  * @brief check rx buff usart0
  * @param  none
  * @retval size real buff available
  */
uint32_t uart_instance3_available(void)
{
  return app_uartRx_is_available(p_instance3);
}

uint32_t uart_instance3_can_send(void)
{
  return app_uartTx_can_send(p_instance3);
}

void uart_instance3_flush(void)
{
  app_uart_flush(p_instance3);
}

/* Test uart echo */
void uart_instance3_echo(void)
{
	if (uart_instance3_available())
	{
		uint8_t c;
		if (UART_OK == getchar_instance3(&c))
		{
			putchar_instance3(c);
		}
	}
}

