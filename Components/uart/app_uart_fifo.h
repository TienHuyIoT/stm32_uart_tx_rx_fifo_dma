/**@file
 *
 * @defgroup app_uart UART module
 * @{
 * @ingroup app_common
 *
 * @brief UART module interface.
 */

#ifndef APP_UART_FIFO_H__
#define APP_UART_FIFO_H__

/* Macro for live debug */
#define APP_UART_FIFO_LIVE_DBG	0

#define APP_UART_FIFO_RX_DMA	1

#define APP_UART_FIFO_RX_ECHO	0

#define IRQ_RX_BUF_SIZE         64
#define IRQ_TX_BUF_SIZE         256

#include <stdint.h>
#include <stdbool.h>
#include "app_fifo_extra.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	APP_UART_FIFO_OK = 0,
	APP_UART_FIFO_ERROR
} APP_UART_Status_t;

typedef enum
{
    EVT_APP_UART_TX_READY,             /*!< UART TX ready to send */
	EVT_APP_UART_RX_READY,             /*!< UART RX ready to recieve */
	EVT_APP_UART_TX_EMPTY,             /*!< UART TX fifo empty */
	EVT_APP_UART_RX_EMPTY,             /*!< UART RX fifo empty */
	EVT_APP_UART_TX_OVER,              /*!< UART TX fifo full */
	EVT_APP_UART_RX_OVER,              /*!< UART RX fifo full */
	EVT_APP_UART_DATA_SENDING,         /*!< UART TX data sending */
	EVT_APP_UART_DATA_READY,           /*!< UART RX data available */
	EVT_APP_UART_TX_FIFO_ERROR,        /*!< UART TX fifo error */
	EVT_APP_UART_RX_FIFO_ERROR,        /*!< UART RX fifo error */
	EVT_APP_UART_COMMUNICATION_ERROR   /*!< UART transfer data error */
} app_uart_evt_type_t;

/**@brief Struct containing events from the UART module.
 *
 * @details The app_uart_evt_t is used to notify the application of asynchronous events when data
 * are received on the UART peripheral or in case an error occured during data reception.
 */
typedef struct
{
    volatile app_uart_evt_type_t evt_type; /**< Type of event. */
    union
    {
        uint32_t error_communication;
        uint32_t error_code;
        uint8_t  value;
    } data;
} app_uart_evt_t;

typedef enum
{
    APP_UART_IRQ_TX,
	APP_UART_IRQ_RX,
	APP_UART_IRQ_IDLE,
	APP_UART_IRQ_ERROR
} app_uart_irq_t;

typedef struct {
	uint32_t size;
	uint8_t *buff;
	uint32_t head;
	uint32_t tail;
	uint32_t xfer_size;
} irq_buff_t;

/**@brief UART buffer for transmitting/receiving data.
 */
typedef struct
{
    struct
	{
		uint8_t * rx_buf;      /**< Pointer to the RX fifo buffer. */
		uint32_t  rx_buf_size; /**< Size of the RX fifo buffer. */
		uint8_t * tx_buf;      /**< Pointer to the TX fifo buffer. */
		uint32_t  tx_buf_size; /**< Size of the TX fifo buffer. */
	} fifo;

	struct
	{
		uint8_t * rx_buf;      /**< Pointer to the RX irq buffer. */
		uint32_t  rx_buf_size; /**< Size of the RX irq buffer. */
		uint8_t * tx_buf;      /**< Pointer to the TX irq buffer. */
		uint32_t  tx_buf_size; /**< Size of the TX irq buffer. */
	} irq;
} app_uart_buffers_t;

struct app_uart_fifo_ctx;
typedef void (*app_uart_irq_callback)(struct app_uart_fifo_ctx*, app_uart_irq_t irq_event);
typedef void (*app_uart_transmit_ptr) (uint8_t *, uint32_t lenght);
typedef void (*app_uart_receive_ptr) (uint8_t *, uint32_t lenght);
typedef void (*app_uart_event_ptr) (struct app_uart_fifo_ctx*, app_uart_evt_t *);
typedef struct app_uart_fifo_ctx
{
	/** Component mandatory fields **/
	app_uart_irq_callback   irq_callback;
	app_uart_transmit_ptr   fp_transmit;
	app_uart_receive_ptr    fp_receive;
	app_uart_event_ptr		fp_event;
	app_uart_evt_type_t		tx_status;
	app_fifo_t				tx_fifo;
	app_fifo_t				rx_fifo;
	irq_buff_t				tx_irq;
	irq_buff_t				rx_irq;
	uint8_t					tx_over;
	uint8_t					rx_over;
	uint8_t 				rx_disable;
	/** Customizable optional pointer **/
	void *handle;
} app_uart_fifo_ctx_t;

uint32_t app_uart_fifo_init(app_uart_fifo_ctx_t *app_cxt, app_uart_buffers_t*);

#define APP_UART_FIFO_INIT(P_COMM_PARAMS, RX_BUF_SIZE, TX_BUF_SIZE, ERR_CODE) \
    do                                                                        \
    {                                                                         \
    	app_uart_buffers_t buffers;											  \
		static uint8_t     rx_fifo_buf[RX_BUF_SIZE];                          \
        static uint8_t     tx_fifo_buf[TX_BUF_SIZE];                          \
        static uint8_t     rx_irq_buf[IRQ_RX_BUF_SIZE];                       \
        static uint8_t     tx_irq_buf[IRQ_TX_BUF_SIZE];                       \
                                                                              \
        buffers.fifo.rx_buf      = rx_fifo_buf;                               \
        buffers.fifo.rx_buf_size = sizeof (rx_fifo_buf);                      \
        buffers.fifo.tx_buf      = tx_fifo_buf;                               \
        buffers.fifo.tx_buf_size = sizeof (tx_fifo_buf);                      \
        buffers.irq.rx_buf      = rx_irq_buf;                                 \
		buffers.irq.rx_buf_size = sizeof (rx_irq_buf);                        \
		buffers.irq.tx_buf      = tx_irq_buf;                                 \
		buffers.irq.tx_buf_size = sizeof (tx_irq_buf);                        \
        ERR_CODE = app_uart_fifo_init(P_COMM_PARAMS, &buffers);    			  \
    } while (0)

uint32_t app_uart_get(app_uart_fifo_ctx_t *app_cxt, uint8_t *p_byte);

uint32_t app_uart_put(app_uart_fifo_ctx_t *app_cxt, uint8_t byte);

uint32_t app_uart_flush(app_uart_fifo_ctx_t *app_cxt);

uint32_t app_uartRx_is_available(app_uart_fifo_ctx_t *app_cxt);

uint32_t app_uartTx_can_send(app_uart_fifo_ctx_t *app_cxt);

#ifdef __cplusplus
}
#endif

#endif //APP_UART_FIFO_H__

/** @} */
