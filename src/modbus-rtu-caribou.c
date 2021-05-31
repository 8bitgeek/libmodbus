/*****************************************************************************
* Copyright (c) 2013 by Accutron Instruments                                 *
* All Rights Reserved                                                        *
*****************************************************************************/
#include <caribou.h>
#include <caribou/lib/stdio.h>
#include <modbus-private.h>
#include <modbus-rtu.h>
#include <modbus-rtu-private.h>
#include <syslog_printf.h>

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

#define rs485_tx(ctx_rtu) {                                                 \
    caribou_thread_lock();                                                  \
    caribou_gpio_set((ctx_rtu)->rs485_dir);                                 \
    if((ctx_rtu)->nrs485_dir) caribou_gpio_set((ctx_rtu)->nrs485_dir);      \
    caribou_thread_unlock(); }
    
#define rs485_rx(ctx_rtu) {                                                 \
    caribou_thread_lock();                                                  \
    caribou_gpio_reset((ctx_rtu)->rs485_dir);                               \
    if((ctx_rtu)->nrs485_dir) caribou_gpio_reset((ctx_rtu)->nrs485_dir);    \
    caribou_thread_unlock(); }


/* Define the slave ID of the remote device to talk in master mode or set the
 * internal slave ID in slave mode */
static int _modbus_set_slave(modbus_t *ctx, int slave)
{
    /* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
    if (slave >= 0 && slave <= 247) 
	{
        ctx->slave = slave;
    } 
	else 
	{
        errno = EINVAL;
        return -1;
    }
    return 0;
}

/* Builds a RTU request header */
static int _modbus_rtu_build_request_basis(modbus_t *ctx, int function,
                                           int addr, int nb,
                                           uint8_t *req)
{
    req[0] = ctx->slave;
    req[1] = function;
    req[2] = addr >> 8;
    req[3] = addr & 0x00ff;
    req[4] = nb >> 8;
    req[5] = nb & 0x00ff;
    return _MODBUS_RTU_PRESET_REQ_LENGTH;
}

/* Builds a RTU response header */
static int _modbus_rtu_build_response_basis(sft_t *sft, uint8_t *rsp)
{
    /* In this case, the slave is certainly valid because a check is already
     * done in _modbus_rtu_listen */
    rsp[0] = sft->slave;
    rsp[1] = sft->function;
    return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) 
	{
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }
    return (crc_hi << 8 | crc_lo);
}

static int _modbus_rtu_prepare_response_tid(const uint8_t *req, int *req_length)
{
    (*req_length) -= _MODBUS_RTU_CHECKSUM_LENGTH;
    /* No TID */
    return 0;
}

static int _modbus_rtu_send_msg_pre(uint8_t *req, int req_length)
{
    uint16_t crc = crc16(req, req_length);
    req[req_length++] = crc >> 8;
    req[req_length++] = crc & 0x00FF;
    return req_length;
}

static int _modbus_rtu_flush(modbus_t *);

static ssize_t _modbus_rtu_send(modbus_t *ctx, const uint8_t *req, int req_length)
{
	ssize_t size;
	modbus_rtu_t *ctx_rtu = ctx->backend_data;
    rs485_tx(ctx_rtu);
    size = fwrite( (uint8_t *)req, req_length, 1, ctx_rtu->fp);
	/* flush the transmit queue */
	while( caribou_uart_tx_busy(ctx_rtu->fd) || !caribou_bytequeue_empty(caribou_uart_tx_queue(ctx_rtu->fd)) )
	{
		caribou_thread_yield();
	}
    rs485_rx(ctx_rtu);
	return size;
}

static int _modbus_rtu_receive(modbus_t *ctx, uint8_t *req)
{
    int rc;
    modbus_rtu_t *ctx_rtu = ctx->backend_data;

    if (ctx_rtu->confirmation_to_ignore) 
	{
        _modbus_receive_msg(ctx, req, MSG_CONFIRMATION);
        /* Ignore errors and reset the flag */
        ctx_rtu->confirmation_to_ignore = FALSE;
        rc = 0;
        if (ctx->debug) 
		{
            SYSLOG_PRINTF( NULL, SYSLOG_DEBUG, "Confirmation to ignore");
        }
    } 
	else 
	{
        rc = _modbus_receive_msg(ctx, req, MSG_INDICATION);
        if (rc == 0) 
		{
            /* The next expected message is a confirmation to ignore */
            ctx_rtu->confirmation_to_ignore = TRUE;
        }
    }
    return rc;
}

static ssize_t _modbus_rtu_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length)
{
	ssize_t size=0;
	modbus_rtu_t *ctx_rtu = ctx->backend_data;
	int c;
	while( size < rsp_length && (c = caribou_bytequeue_get(caribou_uart_rx_queue(ctx_rtu->fd))) >= 0 )
	{
		rsp[size++]=(c&0xFF);
	}
	return size;
}

static int _modbus_rtu_pre_check_confirmation(modbus_t *ctx, const uint8_t *req,
                                              const uint8_t *rsp, int rsp_length)
{
    /* Check responding slave is the slave we requested (except for broacast
     * request) */
    if (req[0] != rsp[0] && req[0] != MODBUS_BROADCAST_ADDRESS) {
        if (ctx->debug) {
            MODBUS_FPRINTF(stderr,
                    "The responding slave %d isn't the requested slave %d",
                    rsp[0], req[0]);
        }
        errno = EMBBADSLAVE;
        return -1;
    } else {
        return 0;
    }
}

/* The check_crc16 function shall return 0 is the message is ignored and the
   message length if the CRC is valid. Otherwise it shall return -1 and set
   errno to EMBADCRC. */
static int _modbus_rtu_check_integrity(modbus_t *ctx, uint8_t *msg,
                                       const int msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;
    int slave = msg[0];

    /* Filter on the Modbus unit identifier (slave) in RTU mode to avoid useless
     * CRC computing. */
    if (slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS) {
        if (ctx->debug) {
            SYSLOG_PRINTF( NULL, SYSLOG_DEBUG, "Request for slave %d ignored (not %d)", slave, ctx->slave);
        }
        /* Following call to check_confirmation handles this error */
        return 0;
    }

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received) {
        return msg_length;
    } else {
        if (ctx->debug) {
            MODBUS_FPRINTF(stderr, "ERROR CRC received %0X != CRC calculated %0X",
                    crc_received, crc_calculated);
        }

        if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
            _modbus_rtu_flush(ctx);
        }
        errno = EMBBADCRC;
        return -1;
    }
}

/* Sets up a serial port for RTU communications */
static int _modbus_rtu_connect(modbus_t *ctx)
{
	caribou_uart_config_t uart_config;
    modbus_rtu_t *ctx_rtu = ctx->backend_data;

    if (ctx->debug) {
        SYSLOG_PRINTF( NULL, SYSLOG_DEBUG, "Opening %s at %d bauds (%c, %d, %d)",
               ctx_rtu->device, ctx_rtu->baud, ctx_rtu->parity,
               ctx_rtu->data_bit, ctx_rtu->stop_bit);
    }
	
	caribou_uart_init_config(&uart_config);

    /* Speed setting */
    switch (ctx_rtu->baud) 
	{
		case 110:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_110;
			break;
		case 300:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_300;
			break;
		case 600:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_600;
			break;
		case 1200:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_1200;
			break;
		case 2400:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_2400;
			break;
		case 4800:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_4800;
			break;
		case 9600:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_9600;
			break;
		case 19200:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_19200;
			break;
		case 38400:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_38400;
			break;
		case 57600:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_57600;
			break;
		case 115200:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_115200;
			break;
		case 230400:
			/* CBR_230400 - not defined */
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_230400;
			break;
		default:
			uart_config.baud_rate = CARIBOU_UART_BAUD_RATE_9600;
			SYSLOG_PRINTF( NULL, SYSLOG_DEBUG, "WARNING Unknown baud rate %d for %s (B9600 used)",
				   ctx_rtu->baud, ctx_rtu->device);
    }

    /* Data bits */
    switch (ctx_rtu->data_bit) 
	{
		case 5:
			uart_config.word_size = CARIBOU_UART_WORDSIZE_5;
			break;
		case 6:
			uart_config.word_size = CARIBOU_UART_WORDSIZE_6;
			break;
		case 7:
			uart_config.word_size = CARIBOU_UART_WORDSIZE_7;
			break;
		case 8:
		default:
			uart_config.word_size = CARIBOU_UART_WORDSIZE_8;
			break;
    }

    /* Stop bits */
	switch( ctx_rtu->stop_bit )
	{
		case 1:
			uart_config.stop_bits = CARIBOU_UART_STOPBITS_1;
			break;
		case 2: /* 2 */
			uart_config.stop_bits = CARIBOU_UART_STOPBITS_2;
			break;
	}

    /* Parity */
	switch( ctx_rtu->parity )
	{
		default:
		case 'N':
			uart_config.parity_bits = CARIBOU_UART_PARITY_NONE;
			break;
		case 'E':
			uart_config.parity_bits = CARIBOU_UART_PARITY_EVEN;
			break;
		case 'O':
			uart_config.parity_bits = CARIBOU_UART_PARITY_ODD;
			break;
	}

	uart_config.dma_mode		= CARIBOU_UART_DMA_RX /* | CARIBOU_UART_DMA_TX */;
	uart_config.dma_prio		= CARIBOU_UART_DMA_PRIO_HIGH;

	caribou_uart_set_config(ctx_rtu->fd,&uart_config);		/* FIXME - use get correct port number */

    return 0;
}

int modbus_rtu_set_serial_mode(modbus_t *ctx, int mode)
{
	int rc=0;
	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
		ctx_rtu->serial_mode = mode;
	}
	else
	{
        errno = EINVAL;
        rc = (-1);		
	}
	return rc;
}

int modbus_rtu_get_serial_mode(modbus_t *ctx)
{
	int rc=0;
	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
		rc = ctx_rtu->serial_mode;
	}
	else
	{
        errno = EINVAL;
        rc = (-1);		
	}
	return rc;
}

int modbus_rtu_set_rts(modbus_t *ctx, int mode)
{
	int rc=0;
	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
		if ( mode == MODBUS_RTU_RTS_UP )
            rs485_tx(ctx_rtu)
		else
			rs485_rx(ctx_rtu);
	}
	else
	{
        errno = EINVAL;
        rc = (-1);		
	}
	return rc;
}

int modbus_rtu_get_rts(modbus_t *ctx)
{
	int rc=0;
	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
		if ( caribou_gpio_pinstate(ctx_rtu->rs485_dir) )
			rc=1;
		else
			rc=0;
	}
	else
	{
        errno = EINVAL;
        rc = (-1);		
	}
	return rc;
}

static void _modbus_rtu_close(modbus_t *ctx)
{
    /* Restore line settings and close file descriptor in RTU mode */
	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
		fclose(ctx_rtu->fp);
	}
}

static int _modbus_rtu_flush(modbus_t *ctx)
{
	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
		/* flush the receiver queue */
		while( !caribou_bytequeue_empty(caribou_uart_rx_queue(ctx_rtu->fd)) )
		{
			caribou_bytequeue_get(caribou_uart_rx_queue(ctx_rtu->fd));
		}
	}
    return 0;
}

static int _modbus_rtu_select(modbus_t *ctx, 
							caribou_tick_t tv, 
							int length_to_read)
{
    int s_rc=0;

	if ( ctx != NULL && ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU )
	{
		caribou_tick_t start = caribou_timer_ticks();
		caribou_tick_t timeout = tv;
		while( caribou_bytequeue_size(caribou_uart_rx_queue(ctx->s)) < length_to_read && caribou_timer_ticks() - start <= timeout )
		{
			caribou_thread_yield();
		}
        if( caribou_bytequeue_size(caribou_uart_rx_queue(ctx->s)) >= length_to_read )
		{
			s_rc=1;
		}
		else
		{
			errno = ETIMEDOUT;
			s_rc=-1;
		}
	}
    return s_rc;
}

static void _modbus_rtu_free(modbus_t *ctx) {
    free(((modbus_rtu_t*)ctx->backend_data)->device);
    free(ctx->backend_data);
    free(ctx);
}

const modbus_backend_t _modbus_rtu_backend = {
    _MODBUS_BACKEND_TYPE_RTU,
    _MODBUS_RTU_HEADER_LENGTH,
    _MODBUS_RTU_CHECKSUM_LENGTH,
    MODBUS_RTU_MAX_ADU_LENGTH,
    _modbus_set_slave,
    _modbus_rtu_build_request_basis,
    _modbus_rtu_build_response_basis,
    _modbus_rtu_prepare_response_tid,
    _modbus_rtu_send_msg_pre,
    _modbus_rtu_send,
    _modbus_rtu_receive,
    _modbus_rtu_recv,
    _modbus_rtu_check_integrity,
    _modbus_rtu_pre_check_confirmation,
    _modbus_rtu_connect,
    _modbus_rtu_close,
    _modbus_rtu_flush,
    _modbus_rtu_select,
    _modbus_rtu_free
};

modbus_t* modbus_new_rtu(FILE* fp,
						 caribou_gpio_t* rs485_dir,
						 caribou_gpio_t* nrs485_dir,
                         int baud,
						 char parity, 
						 int data_bit,
                         int stop_bit)
{
    modbus_t *ctx;
    modbus_rtu_t *ctx_rtu;

    ctx = (modbus_t *) malloc(sizeof(modbus_t));
    _modbus_init_common(ctx);
    ctx->backend = &_modbus_rtu_backend;
    ctx->backend_data = (modbus_rtu_t *) malloc(sizeof(modbus_rtu_t));
    ctx_rtu = (modbus_rtu_t *)ctx->backend_data;

    /* Device name and \0 */
    ctx_rtu->device = NULL;
	ctx_rtu->fp=fp;
	ctx_rtu->fd=_fd(fp);
	ctx_rtu->rs485_dir=rs485_dir;
	ctx_rtu->nrs485_dir=nrs485_dir;

    ctx_rtu->baud = baud;
    if (parity == 'N' || parity == 'E' || parity == 'O') {
        ctx_rtu->parity = parity;
    } else {
        modbus_free(ctx);
        errno = EINVAL;
        return NULL;
    }
    ctx_rtu->data_bit = data_bit;
    ctx_rtu->stop_bit = stop_bit;

#if HAVE_DECL_TIOCSRS485
    /* The RS232 mode has been set by default */
    ctx_rtu->serial_mode = MODBUS_RTU_RS232;
#endif

#if HAVE_DECL_TIOCM_RTS
    /* The RTS use has been set by default */
    ctx_rtu->rts = MODBUS_RTU_RTS_NONE;

    /* Calculate estimated time in micro second to send one byte */
    ctx_rtu->onebyte_time = (1000 * 1000) * (1 + data_bit + (parity == 'N' ? 0 : 1) + stop_bit) / baud;
#endif

    ctx_rtu->confirmation_to_ignore = FALSE;

    return ctx;
}
