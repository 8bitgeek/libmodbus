/*****************************************************************************
* Copyright (c) 2013 by Accutron Instruments                                 *
* All Rights Reserved                                                        *
*****************************************************************************/
#include <caribou.h>
#include <modbus-private.h>
#include <modbus-tcp.h>
#include <modbus-tcp-private.h>
#include <syslog_printf.h>

#if !defined(MSG_NOSIGNAL)
#define MSG_NOSIGNAL 0
#endif

static int _modbus_tcp_select(modbus_t *ctx, caribou_tick_t ms, int length_to_read);
static int _modbus_tcp_bytes_available(int s);


static int _modbus_set_slave(modbus_t *ctx, int slave)
{
    /* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
    if (slave >= 0 && slave <= 247) 
	{
        ctx->slave = slave;
    } 
	else if (slave == MODBUS_TCP_SLAVE) 
	{
        /* The special value MODBUS_TCP_SLAVE (0xFF) can be used in TCP mode to
         * restore the default value. */
        ctx->slave = slave;
    } 
	else 
	{
        errno = EINVAL;
        return -1;
    }

    return 0;
}

/* Builds a TCP request header */
static int _modbus_tcp_build_request_basis(modbus_t *ctx, int function, int addr, int nb, uint8_t *req)
{
    modbus_tcp_t *ctx_tcp = (modbus_tcp_t *)ctx->backend_data;

    /* Increase transaction ID */
    if (ctx_tcp->t_id < UINT16_MAX)
        ctx_tcp->t_id++;
    else
        ctx_tcp->t_id = 0;
    req[0] = ctx_tcp->t_id >> 8;
    req[1] = ctx_tcp->t_id & 0x00ff;

    /* Protocol Modbus */
    req[2] = 0;
    req[3] = 0;

    /* Length will be defined later by set_req_length_tcp at offsets 4
       and 5 */

    req[6] = ctx->slave;
    req[7] = function;
    req[8] = addr >> 8;
    req[9] = addr & 0x00ff;
    req[10] = nb >> 8;
    req[11] = nb & 0x00ff;

    return _MODBUS_TCP_PRESET_REQ_LENGTH;
}

/* Builds a TCP response header */
static int _modbus_tcp_build_response_basis(sft_t *sft, uint8_t *rsp)
{
    /* Extract from MODBUS Messaging on TCP/IP Implementation
       Guide V1.0b (page 23/46):
       The transaction identifier is used to associate the future
       response with the request. */
    rsp[0] = sft->t_id >> 8;
    rsp[1] = sft->t_id & 0x00ff;

    /* Protocol Modbus */
    rsp[2] = 0;
    rsp[3] = 0;

    /* Length will be set later by send_msg (4 and 5) */

    /* The slave ID is copied from the indication */
    rsp[6] = sft->slave;
    rsp[7] = sft->function;

    return _MODBUS_TCP_PRESET_RSP_LENGTH;
}


static int _modbus_tcp_prepare_response_tid(const uint8_t *req, int *req_length)
{
    return (req[0] << 8) + req[1];
}

static int _modbus_tcp_send_msg_pre(uint8_t *req, int req_length)
{
    /* Substract the header length to the message length */
    int mbap_length = req_length - 6;

    req[4] = mbap_length >> 8;
    req[5] = mbap_length & 0x00FF;

    return req_length;
}

static ssize_t _modbus_tcp_send(modbus_t *ctx, const uint8_t *req, int req_length)
{
    /* MSG_NOSIGNAL
       Requests not to send SIGPIPE on errors on stream oriented
       sockets when the other end breaks the connection.  The EPIPE
       error is still returned. */
    return lwip_send(ctx->s, (const char*)req, req_length, MSG_NOSIGNAL);
}

static int _modbus_tcp_receive(modbus_t *ctx, uint8_t *req) 
{
    return _modbus_receive_msg(ctx, req, MSG_INDICATION);
}

static ssize_t _modbus_tcp_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length) 
{
    return lwip_recv(ctx->s, (char *)rsp, rsp_length, 0);
}

static int _modbus_tcp_check_integrity(modbus_t *ctx, uint8_t *msg, const int msg_length)
{
    return msg_length;
}

static int _modbus_tcp_pre_check_confirmation(modbus_t *ctx, const uint8_t *req, const uint8_t *rsp, int rsp_length)
{
    /* Check TID */
    if (req[0] != rsp[0] || req[1] != rsp[1]) {
        if (ctx->debug) {
            MODBUS_FPRINTF(stderr, "Invalid TID received 0x%X (not 0x%X)",
                    (rsp[0] << 8) + rsp[1], (req[0] << 8) + req[1]);
        }
        errno = EMBBADDATA;
        return -1;
    } else {
        return 0;
    }
}

static int _modbus_tcp_set_ipv4_options(int s)
{
    int rc;
    int option;

    /* Set the TCP no delay flag */
    /* SOL_TCP = IPPROTO_TCP */
    option = 1;
    rc = lwip_setsockopt(s, IPPROTO_TCP, TCP_NODELAY,
                    (const void *)&option, sizeof(int));
    if (rc == -1) {
        return -1;
    }

    /* If the OS does not offer SOCK_NONBLOCK, fall back to setting FIONBIO to
     * make sockets non-blocking */
    /* Do not care about the return value, this is optional */
    option = 1;
	#if !defined(SOCK_NONBLOCK) && defined(FIONBIO)
		lwip_ioctl(s, FIONBIO, &option);
	#endif

    return 0;
}

static int _connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen, caribou_tick_t tv)
{
    int rc;

    rc = lwip_connect(sockfd, addr, addrlen);

    if (rc == -1 && errno == EINPROGRESS) 
    {
        int optval;
        socklen_t optlen = sizeof(optval);
        caribou_tick_t start = caribou_timer_ticks();
        while( !caribou_timer_ticks_timeout(start,tv) )
        {
            /* The connection is established if SO_ERROR and optval are set to 0 */
            rc = lwip_getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void *)&optval, &optlen);
            if (rc == 0 && optval == 0) {
                rc = 0;
                break;
            } else {
                errno = ECONNREFUSED;
                rc = -1;
            }
        }
    }
    return rc;
}

/* Establishes a modbus TCP connection with a Modbus server. */
static int _modbus_tcp_connect(modbus_t *ctx)
{
    int rc;
    /* Specialized version of sockaddr for Internet socket address (same size) */
    struct sockaddr_in addr;
    modbus_tcp_t *ctx_tcp = ctx->backend_data;
    int flags = SOCK_STREAM;

	#ifdef SOCK_CLOEXEC
		flags |= SOCK_CLOEXEC;
	#endif

	#ifdef SOCK_NONBLOCK
		flags |= SOCK_NONBLOCK;
	#endif

    ctx->s = lwip_socket(PF_INET, flags, 0);
    if (ctx->s == -1) 
	{
        return -1;
    }

    rc = _modbus_tcp_set_ipv4_options(ctx->s);
    if (rc == -1) 
	{
        lwip_close(ctx->s);
        ctx->s = -1;
        return -1;
    }

    if (ctx->debug) 
	{
        SYSLOG_PRINTF( NULL, SYSLOG_DEBUG, "Connecting to %s:%d", ctx_tcp->ip, ctx_tcp->port);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(ctx_tcp->port);
    addr.sin_addr.s_addr = inet_addr(ctx_tcp->ip);
    rc = _connect(ctx->s, (struct sockaddr *)&addr, sizeof(addr), ctx->response_timeout);
    if (rc == -1) 
	{
        lwip_close(ctx->s);
        ctx->s = -1;
        return -1;
    }

    return 0;
}

/* Establishes a modbus TCP PI connection with a Modbus server. */
/* 
From http://libmodbus.org/docs/v3.0.6/ re: Modbus TCP PI...
The TCP PI (Protocol Indepedent) backend implements a Modbus variant used for communications over TCP IPv4 and IPv6 networks. It does not require a checksum calculation as lower layer takes care of the same.
Contrary to the TCP IPv4 only backend, the TCP PI backend offers hostname resolution but it consumes about 1Kb of additional memory.
*/

static int _modbus_tcp_pi_connect(modbus_t *ctx)
{
    errno = ECONNREFUSED;
	return -1;
}

/* Closes the network connection and socket in TCP mode */
static void _modbus_tcp_close(modbus_t *ctx)
{
    if (ctx->s != -1) {
        lwip_shutdown(ctx->s, SHUT_RDWR);
        lwip_close(ctx->s);
        ctx->s = -1;
    }
}

static int _modbus_tcp_flush(modbus_t *ctx)
{
    int rc;
    int rc_sum = 0;

    do {
        /* Extract the garbage from the socket */
        char devnull[MODBUS_TCP_MAX_ADU_LENGTH];

        rc = lwip_recv(ctx->s, devnull, MODBUS_TCP_MAX_ADU_LENGTH, MSG_DONTWAIT);

        if (rc > 0) 
		{
            rc_sum += rc;
        }
    } while (rc == MODBUS_TCP_MAX_ADU_LENGTH);

    return rc_sum;
}

/* Listens for any request from one or many modbus masters in TCP */
int modbus_tcp_listen(modbus_t *ctx, int nb_connection)
{
    int new_s;
    int yes;
    struct sockaddr_in addr;
    modbus_tcp_t *ctx_tcp;

    if (ctx == NULL) 
	{
        errno = EINVAL;
        return -1;
    }

    ctx_tcp = ctx->backend_data;

    new_s = lwip_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (new_s == -1) 
	{
        return -1;
    }

	//set to non-blocking mode
	int i = 1;
	lwip_ioctl(new_s, FIONBIO, &i);

    yes = 1;
	#if 0 // [RMS] LwIP not supported?
		if (lwip_setsockopt(new_s, SOL_SOCKET, SO_REUSEADDR, (char *) &yes, sizeof(yes)) == -1) 
		{
			lwip_close(new_s);
			return -1;
		}
	#endif

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    /* If the modbus port is < to 1024, we need the setuid root. */
    addr.sin_port = lwip_htons(ctx_tcp->port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (lwip_bind(new_s, (struct sockaddr *)&addr, sizeof(addr)) == -1) 
	{
        lwip_close(new_s);
        return -1;
    }

    if (lwip_listen(new_s, nb_connection) == -1) 
	{
        lwip_close(new_s);
        return -1;
    }

    return new_s;
}

int modbus_tcp_pi_listen(modbus_t *ctx, int nb_connection)
{
    errno = EINVAL;
    return -1;
}

/* On success, the function return a non-negative integer that is a descriptor
for the accepted socket. On error, socket is set to -1, -1 is returned and errno
is set appropriately. */
int modbus_tcp_accept(modbus_t *ctx, int *s)
{
    struct sockaddr_in addr;
    socklen_t addrlen;

    if (ctx == NULL) 
	{
        errno = EINVAL;
        return -1;
    }

    addrlen = sizeof(addr);
    /* Inherit socket flags and use accept call */
    ctx->s = lwip_accept(*s, (struct sockaddr *)&addr, &addrlen);
	if ( ctx->s >= 0 )
	{
		//set to non-blocking mode
		int i = 1;
		lwip_ioctl(ctx->s, FIONBIO, &i);

		if (ctx->debug) 
		{
			SYSLOG_PRINTF( NULL, SYSLOG_DEBUG, "The client connection from %s is accepted",inet_ntoa(addr.sin_addr));
		}
	} 
	else if (errno != EAGAIN) 
	{
		#if defined(DEBUG)
			lwip_close(*s);
			*s = -1;
		#else
			chip_reset();		/* Fatal listener fault...brute force recover */
		#endif
        return -1;
    }

    return ctx->s;
}

int modbus_tcp_pi_accept(modbus_t *ctx, int *s)
{
	errno = EINVAL; /* TODO caribou translate */
	return -1;
}

/*
 * Return the number of bytes available in the receive queue.
 * @return <0 on error (errno), return 0 upon other end disconnect, < 0 socket closed
 */
static int _modbus_tcp_bytes_available(int s)
{
    int rc;
    char t[32];
    struct sockaddr_in fromaddr;
    socklen_t fromlen=sizeof(struct sockaddr_in);
    rc = lwip_recvfrom(s,t,32,MSG_DONTWAIT|MSG_PEEK,(struct sockaddr*)&fromaddr,&fromlen);
    if ( rc < 0 )
    {
        if ( errno == EAGAIN )
        {
            rc = 0;
        }
    }
    else if ( rc == 0 )
    {
        rc = (-1); /* socket closed. */
    }
    return rc;
}


static int _modbus_tcp_select(modbus_t *ctx, caribou_tick_t ms, int length_to_read)
{
    caribou_tick_t start = caribou_timer_ticks();
    int rc=0;
    while( (rc=_modbus_tcp_bytes_available(ctx->s)) == 0 && !caribou_timer_ticks_timeout(start,ms) )
    {
        caribou_thread_yield();
    }
    return rc;
}

static void _modbus_tcp_free(modbus_t *ctx) 
{
    free(ctx->backend_data);
    free(ctx);
}

const modbus_backend_t _modbus_tcp_backend = 
{
    _MODBUS_BACKEND_TYPE_TCP,
    _MODBUS_TCP_HEADER_LENGTH,
    _MODBUS_TCP_CHECKSUM_LENGTH,
    MODBUS_TCP_MAX_ADU_LENGTH,
    _modbus_set_slave,
    _modbus_tcp_build_request_basis,
    _modbus_tcp_build_response_basis,
    _modbus_tcp_prepare_response_tid,
    _modbus_tcp_send_msg_pre,
    _modbus_tcp_send,
    _modbus_tcp_receive,
    _modbus_tcp_recv,
    _modbus_tcp_check_integrity,
    _modbus_tcp_pre_check_confirmation,
    _modbus_tcp_connect,
    _modbus_tcp_close,
    _modbus_tcp_flush,
    _modbus_tcp_select,
    _modbus_tcp_free
};


const modbus_backend_t _modbus_tcp_pi_backend = 
{
    _MODBUS_BACKEND_TYPE_TCP,
    _MODBUS_TCP_HEADER_LENGTH,
    _MODBUS_TCP_CHECKSUM_LENGTH,
    MODBUS_TCP_MAX_ADU_LENGTH,
    _modbus_set_slave,
    _modbus_tcp_build_request_basis,
    _modbus_tcp_build_response_basis,
    _modbus_tcp_prepare_response_tid,
    _modbus_tcp_send_msg_pre,
    _modbus_tcp_send,
    _modbus_tcp_receive,
    _modbus_tcp_recv,
    _modbus_tcp_check_integrity,
    _modbus_tcp_pre_check_confirmation,
    _modbus_tcp_pi_connect,
    _modbus_tcp_close,
    _modbus_tcp_flush,
    _modbus_tcp_select,
    _modbus_tcp_free
};

modbus_t* modbus_new_tcp(const char *ip, int port)
{
    modbus_t *ctx;
    modbus_tcp_t *ctx_tcp;
    size_t dest_size;
    size_t ret_size;

    ctx = (modbus_t *) malloc(sizeof(modbus_t));
    _modbus_init_common(ctx);

    /* Could be changed after to reach a remote serial Modbus device */
    ctx->slave = MODBUS_TCP_SLAVE;

    ctx->backend = &(_modbus_tcp_backend);

    ctx->backend_data = (modbus_tcp_t *) malloc(sizeof(modbus_tcp_t));
    ctx_tcp = (modbus_tcp_t *)ctx->backend_data;

    dest_size = sizeof(char) * 16;
    ret_size = strlcpy(ctx_tcp->ip, ip, dest_size);
    if (ret_size == 0) {
        MODBUS_FPRINTF(stderr, "The IP string is empty");
        modbus_free(ctx);
        errno = EINVAL;
        return NULL;
    }

    if (ret_size >= dest_size) {
        MODBUS_FPRINTF(stderr, "The IP string has been truncated");
        modbus_free(ctx);
        errno = EINVAL;
        return NULL;
    }

    ctx_tcp->port = port;
    ctx_tcp->t_id = 0;

    return ctx;
}


modbus_t* modbus_new_tcp_pi(const char *node, const char *service)
{
	/* TODO caribou translate */
	return NULL;
}
