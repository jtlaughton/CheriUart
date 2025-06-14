 #ifndef UART_H
 #define UART_H

 // based on this file: https://github.com/umanovskis/baremetal-arm/blob/master/src/06_uart/src/uart_pl011.c
 // failed attempt but hey why not

 #include <sys/systm.h>
 #include <sys/bus.h>
 #include <sys/kernel.h>
 #include <sys/module.h>
 #include <sys/malloc.h>
 #include <sys/rman.h>
 #include <sys/ioccom.h>
 #include <sys/conf.h>
 #include <sys/param.h>
 #include <sys/lock.h>
 #include <sys/mutex.h>
 #include <sys/types.h>
 #include <sys/proc.h>
 #include <sys/ucred.h>
 #include <sys/rwlock.h>
 
 #include <vm/vm.h>
 #include <vm/pmap.h>
 #include <vm/vm_param.h>
 #include <vm/vm_object.h>
 #include <vm/vm_page.h>
 #include <vm/vm_pager.h>

#include <cheri/cheric.h>
#include <cheri/cheri.h>

// cheri specific setup
MALLOC_DECLARE(M_DEVBUF);
MALLOC_DEFINE(M_DEVBUF, "uart-cher", "Cherified Uart Device Driver");

typedef struct {
    uint8_t     data_bits;
    uint8_t     stop_bits;
    bool        parity;
    uint32_t    baudrate;
} uart_config;

typedef struct cap_req {
    void* __capability user_cap;
    void* __capability sealed_cap;
} cap_req_t;

typedef struct uart_header_req {
    cap_req_t cap_req;
} uart_header_req_t;

typedef struct tx_uart_req {
    cap_req_t cap_req;
    size_t length;
} tx_uart_req_t;

typedef struct rx_uart_req {
    cap_req_t cap_req;
    size_t length_wanted;
    size_t length_received;
} rx_uart_req_t;

typedef struct __attribute__((packed)) uart_buffers {
    char receive_buffer[PAGE_SIZE / 2];
    char transmit_buffer[PAGE_SIZE / 2];
} uart_buffers_t;

#define UART_RX    _IOWR('E', 1, tx_uart_req_t)
#define UART_TX    _IOWR('E', 2, rx_uart_req_t)

static d_open_t		uart_open;
static d_close_t	uart_close;
static d_ioctl_t	uart_ioctl;
static d_mmap_single_extra_t uart_mmap_single_extra;

static int	uart_pager_ctor(void *handle, vm_ooffset_t size,
    vm_prot_t prot, vm_ooffset_t foff, struct ucred *cred, u_short *color);
static void	uart_pager_dtor(void *handle);
static int	uart_pager_fault(vm_object_t obj, vm_ooffset_t offset,
    int prot, vm_page_t *mres);

static struct cdev_pager_ops uart_cdev_pager_ops = {
	.cdev_pg_ctor = uart_pager_ctor,
	.cdev_pg_dtor = uart_pager_dtor,
	.cdev_pg_fault = uart_pager_fault,
};

static struct cdevsw uart_cdevsw = {
	.d_name		= "uart-chert",
	.d_version	= D_VERSION,
	.d_open		= uart_open,
	.d_close	= uart_close,
	.d_ioctl	= uart_ioctl,
    .d_mmap_single_extra = uart_mmap_single_extra,
};

typedef volatile struct __attribute__((packed)) {
        uint32_t DR;                            /* 0x0 Data Register */
        uint32_t RSRECR;                        /* 0x4 Receive status / error clear register */
        uint32_t _reserved0[4];                 /* 0x8 - 0x14 reserved */
        const uint32_t FR;                      /* 0x18 Flag register */
        uint32_t _reserved1;                    /* 0x1C reserved */
        uint32_t ILPR;                          /* 0x20 Low-power counter register */
        uint32_t IBRD;                          /* 0x24 Integer baudrate register */
        uint32_t FBRD;                          /* 0x28 Fractional baudrate register */
        uint32_t LCRH;                          /* 0x2C Line control register */
        uint32_t CR;                            /* 0x30 Control register */
} uart_registers;

typedef enum {
        UART_OK = 0,
        UART_INVALID_ARGUMENT_BAUDRATE,
        UART_INVALID_ARGUMENT_WORDSIZE,
        UART_INVALID_ARGUMENT_STOP_BITS,
        UART_RECEIVE_ERROR,
        UART_NO_DATA
} uart_error;

typedef struct sealed_cap_state {
    void * __capability original_cap;
    void * __capability sealed_cap;
} sealed_cap_state_t;

typedef struct uart_soft_c {
    device_t dev;                           
    int mem_rid;
    struct resource* mem_res;               
    bool device_attached;
    
    uart_registers* registers;
    struct cdev* cdev;
    struct mtx      sc_mtx;

    uint32_t baudrate;

    // cheri specific things
    uart_buffers_t* __kerncap page;
    bool dying;
    bool mapped;

    void* __capability sealing_key;
    sealed_cap_state_t cap_state;

} uart_softc_t;

#define DR_DATA_MASK    (0xFFu)

#define FR_BUSY         (1 << 3u)
#define FR_RXFE         (1 << 4u)
#define FR_TXFF         (1 << 5u)

#define RSRECR_ERR_MASK (0xFu)

#define LCRH_FEN        (1 << 4u)
#define LCRH_PEN        (1 << 1u)
#define LCRH_EPS        (1 << 2u)
#define LCRH_STP2       (1 << 3u)
#define LCRH_SPS        (1 << 7u)
#define CR_UARTEN       (1 << 0u)

#define LCRH_WLEN_5BITS (0u << 5u)
#define LCRH_WLEN_6BITS (1u << 5u)
#define LCRH_WLEN_7BITS (2u << 5u)
#define LCRH_WLEN_8BITS (3u << 5u)

#endif
