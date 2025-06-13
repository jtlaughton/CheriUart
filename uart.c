#include "uart.h"
#include <dev/acpica/acpivar.h>

#define REFCLOCK 24000000

#define UART_LOCK_INIT(sc) mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), "uart softc lock", MTX_DEF)
#define UART_LOCK(sc)      mtx_lock(&(sc)->sc_mtx)
#define UART_UNLOCK(sc)    mtx_unlock(&(sc)->sc_mtx)
#define UART_LOCK_DESTROY(sc) mtx_destroy(&(sc)->sc_mtx)

// Forward declarations for device methods
static int uart_acpi_probe(device_t dev);
static int uart_acpi_attach(device_t dev);
static int uart_acpi_detach(device_t dev);

// ACPI-compatible hardware IDs for the PL011 UART
static char *uart_ids[] = { "ARMH0011", NULL };

static int check_cap_token(uart_soft_c_t* sc, void* __capability cap_token){
    if(sc->cap_state.sealed_cap == NULL){
        return EINVAL;
    }

    if(cap_token == NULL){
        return EINVAL;
    }

    void* __capability unsealed_token = cheri_unseal(cap_token, sc->sealing_key);
    if(!cheri_ptr_equal_exact(unsealed_token, sc->cap_state.original_cap)){
        return EPERM;
    }

    return 0;
}

static int check_attach_and_lock(uart_softc_t *sc){
    if(sc == NULL){
        return EINVAL;
    }

    UART_LOCK(sc);
    if(!sc->device_attached){
        UART_UNLOCK(sc);
        return EINVAL;
    }

    return 0;
}

static int
uart_open(struct cdev *dev, int flags, int devtype, struct thread *td)
{
    uart_softc_t *sc = dev->si_drv1;

    int err = check_attach_and_lock(sc);

    if(err){
        return ENXIO;
    }

    UARt_UNLOCK(sc);
	uprintf("UART: device opened\n");
	return (0);
}

static int
uart_close(struct cdev *dev, int flags, int devtype, struct thread *td)
{
	uprintf("UART: device closed\n");

    uart_softc_t *sc = dev->si_drv1;

    if(sc != NULL){
        UART_LOCK(sc);
        revoke_cap_token(sc);
        UART_UNLOCK(sc);
    }

	return (0);
}

// probably will be expanded in the future to revoke all caps in the vm object and such
static void revoke_cap_token(uart_soft_c_t* sc){
    sc->cap_state.original_cap = NULL;
    sc->cap_state.sealed_cap = NULL;
}

static int
create_our_cdev(uart_soft_c_t* sc){
    sc->cdev = make_dev(&uart_cdevsw, 0, UID_ROOT, GID_WHEEL,
        0600, "uart-cheri");
    if(sc->cdev == NULL){
        return EINVAL;
    }

    sc->cdev->si_drv1 = sc;

    // allocate shared mem using VM system instead of contigmalloc
    sc->page = (uart_registers* __kerncap)malloc(sizeof(uart_registers), M_DEVBUF, M_WAITOK | M_ZERO);
    if(sc->page == NULL){
        destroy_dev(sc->cdev);
        device_printf(sc->dev, "Failed to create shared mem\n");
        return EINVAL;
    }

    return 0;
}

static int
destroy_our_cdev(uart_softc_t* sc){
    UART_LOCK(sc);
    if(sc->mapped){
        UART_UNLOCK(sc);
        return EBUSY;
    }

    sc->dying = true;
    UART_UNLOCK(sc);

    destroy_dev(sc->cdev);
    free(sc->page, M_DEVBUF);
    return 0;
}

static int
uart_pager_ctor(void *handle, vm_ooffset_t size, vm_prot_t prot, vm_ooffset_t foff, struct ucred *cred, u_short *color){
    uart_softc_t *sc = handle;

	UART_LOCK(sc);
	sc->mapped = true;
    UART_UNLOCK(sc);

	*color = 0;
	return (0);
}

static void
uart_pager_dtor(void *handle){
    uart_softc_t *sc = handle;

	UART_LOCK(sc);
	sc->mapped = false;
	UART_UNLOCK(sc);
}

static int
uart_pager_fault(vm_object_t obj, vm_ooffset_t offset, int prot, vm_page_t *mres){
    uart_softc_t *sc = obj->handle;
	vm_page_t page;
	vm_paddr_t paddr;

	paddr = pmap_kextract(cheri_getaddress(sc->page) + offset);

	/* See the end of old_dev_pager_fault in device_pager.c. */
	if (((*mres)->flags & (PG_FICTITIOUS | PGA_CAPSTORE)) != 0) {
		page = *mres;
		vm_page_updatefake(page, paddr, VM_MEMATTR_DEFAULT);
	} else {
		VM_OBJECT_WUNLOCK(obj);
		page = vm_page_getfake(paddr, VM_MEMATTR_DEFAULT);
        page->a.flags |= PGA_CAPSTORE;
		VM_OBJECT_WLOCK(obj);
		vm_page_replace(page, obj, (*mres)->pindex, *mres);
		*mres = page;
	}

	vm_page_valid(page);
	return (VM_PAGER_OK);
}

static void* __capability
create_sealing_key(size_t id){
    if(id >= cheri_getbase(kernel_root_sealcap)){
        return NULL;
    }

    void * __capability derived = cheri_setaddress(kernel_root_sealcap, cheri_getbase(kernel_root_sealcap) + id);
    derived = cheri_setbounds(derived, 1);

    return derived;
}

static int uart_mmap_single_extra(struct cdev *cdev, vm_ooffset_t *offset, vm_size_t size, vm_object_t *object, int nprot, void * __kerncap extra){
    uart_softc_t *sc = cdev->si_drv1;
	vm_object_t obj;
    cap_req_t* __kerncap req = NULL;

    // need to have a user request at all to make this work
    if(extra == NULL){
        return EINVAL;
    }

    req = (cap_req_t* __kerncap)extra;

    // validate that request is properly formed
	UART_LOCK(sc);
    if (req->user_cap == NULL ||
        sc == NULL ||
        sc->cap_state.sealed_cap != NULL ||
        (offset != NULL && *offset != 0) ||
        size != PAGE_SIZE){
		UART_UNLOCK(sc);
        return EINVAL;
    }

    // only allow mmap if not in teardown
	if (sc->dying) {
		UART_UNLOCK(sc);
		return (ENXIO);
	}
	UART_UNLOCK(sc);

    // make sure tag provided is valid
    if(!cheri_gettag(req->user_cap)){
        return EINVAL;
    }

    // create vm object for user
	obj = cdev_pager_allocate(sc, OBJT_DEVICE, &uart_cdev_pager_ops,
	    OFF_TO_IDX(PAGE_SIZE), nprot | VM_PROT_CAP, *offset, curthread->td_ucred);
	if (obj == NULL)
		return (ENXIO);

    obj->flags |= OBJ_HASCAP;

	/*
	 * If an unload started while we were allocating the VM
	 * object, dying will now be set and the unloading thread will
	 * be waiting in destroy_dev().  Just release the VM object
	 * and fail the mapping request.
	 */
	UART_LOCK(sc);
	if (sc->dying) {
	    UART_UNLOCK(sc);
		vm_object_deallocate(obj);
		return (ENXIO);
	}

	*object = obj;

    // seal the cap the user provided and give it to them
    sc->cap_state.original_cap = req->user_cap;
    sc->cap_state.sealed_cap = cheri_seal(req->user_cap, sc->sealing_key);

    req->sealed_cap = sc->cap_state.sealed_cap;
	UART_UNLOCK(sc);

	return (0);
}

uart_error uart_configure(uart_config* config) {
    /* Validate config */
    if (config->data_bits < 5u || config->data_bits > 8u) {
        return UART_INVALID_ARGUMENT_WORDSIZE;
    }
    if (config->stop_bits == 0u || config->stop_bits > 2u) {
        return UART_INVALID_ARGUMENT_STOP_BITS;
    }
    if (config->baudrate < 110u || config->baudrate > 460800u) {
        return UART_INVALID_ARGUMENT_BAUDRATE;
    }
    /* Disable the UART */
    sc->registers->CR &= ~CR_UARTEN;
    /* Finish any current transmission, and flush the FIFO */
    while (sc->registers->FR & FR_BUSY);
    sc->registers->LCRH &= ~LCRH_FEN;

    /* Set baudrate */
    double intpart, fractpart;
    double baudrate_divisor = (double)refclock / (16u * config->baudrate);
    fractpart = modf(baudrate_divisor, &intpart);

    sc->registers->IBRD = (uint16_t)intpart;
    sc->registers->FBRD = (uint8_t)((fractpart * 64u) + 0.5);

    uint32_t lcrh = 0u;

    /* Set data word size */
    switch (config->data_bits)
    {
    case 5:
        lcrh |= LCRH_WLEN_5BITS;
        break;
    case 6:
        lcrh |= LCRH_WLEN_6BITS;
        break;
    case 7:
        lcrh |= LCRH_WLEN_7BITS;
        break;
    case 8:
        lcrh |= LCRH_WLEN_8BITS;
        break;
    }

    /* Set parity. If enabled, use even parity */
    if (config->parity) {
        lcrh |= LCRH_PEN;
        lcrh |= LCRH_EPS;
        lcrh |= LCRH_SPS;
    } else {
        lcrh &= ~LCRH_PEN;
        lcrh &= ~LCRH_EPS;
        lcrh &= ~LCRH_SPS;
    }

    /* Set stop bits */
    if (config->stop_bits == 1u) {
        lcrh &= ~LCRH_STP2;
    } else if (config->stop_bits == 2u) {
        lcrh |= LCRH_STP2;
    }

    /* Enable FIFOs */
    lcrh |= LCRH_FEN;

    sc->registers->LCRH = lcrh;

    /* Enable the UART */
    sc->registers->CR |= CR_UARTEN;

    return UART_OK;
}

void uart_putchar(char c) {
    while (sc->registers->FR & FR_TXFF);
    sc->registers->DR = c;
}

void uart_write(uart_soft_c_t* sc, tx_uart_req_t* req) {
    for(size_t i = 0; i < req->length; i++){
        uart_putchar(sc->page->transmit_buffer[i]);
    }
}

uart_error uart_getchar(uart_soft_c_t* sc, char* c) {
    if (sc->registers->FR & FR_RXFE) {
        return UART_NO_DATA;
    }

    *c = sc->registers->DR & DR_DATA_MASK;
    if (sc->registers->RSRECR & RSRECR_ERR_MASK) {
        /* The character had an error */
        sc->registers->RSRECR &= RSRECR_ERR_MASK;
        return UART_RECEIVE_ERROR;
    }
    return UART_OK;
}

int read_until(uart_soft_c_t* sc, rx_uart_req_t* req){
    size_t ammount_read = 0;
    while(ammount_read < req->length_wanted){
        char output_char;
        bool success = false;

        for(size_t i = 0; i < 10000; i++){
            uart_error error = uart_getchar(sc, &output_char);
            if ((error == UART_RECEIVE_ERROR) ||
                (error == UART_NO_DATA)){
                DELAY(1000000 / (sc->baudrate / 8))
                continue;
            }
            else{
                success = true;
                sc->page->receive_buffer[ammount_read] = output_char;
                break;
            }
        }

        if(!success){
            return ammount_read;
        }

        ammount_read++;
    }

    return ammount_read;
}

static int
uart_ioctl(struct cdev *dev, u_long cmd, caddr_t addr, int flags,
    struct thread *td){
    int error = 0;

    uprintf("UART: Addr check\n");
    if(addr == NULL){
        return EINVAL;
    }

    uprintf("UART: Cap cast\n");
    uart_header_req* header_req = (uart_header_req_t*)addr;
    uart_softc_t *sc = dev->si_drv1;

    uprintf("UART: Null check\n");
    if(sc == NULL){
        return EINVAL;
    }

    uprintf("UART: Cap Token Check\n");
    if(check_cap_token(sc, header_req->cap_req.sealed_cap)){
        return EPERM;
    }

    rx_uart_req_t* user_req_rx = NULL;
    tx_uart_req_t* user_req_tx = NULL;

    uprintf("UART: Switch statement\n");
    switch(cmd){
        case UART_RX:
            uprintf("UART: Lock rx\n");
            if(check_attach_and_lock(sc)){
                return EINVAL;
            }

            user_req_rx = (rx_uart_req_t *)addr;

            uint64_t max_len = PAGE_SIZE / 2;

            if(user_req->length_wanted > max_len){
                device_printf(sc->dev, "User Wants Too Many Bytes\n");
                UART_UNLOCK(sc);
                return EINVAL;
            }

            // call uart receive function
            int received = read_until(sc, user_req_rx);
            user_req_rx->length_received = received;

            UART_UNLOCK(sc);
            break;
        case UART_TX:
            uprintf("UART: Lock\n");
            if(check_attach_and_lock(sc)){
                return EINVAL;
            }

            uprintf("UART: Read tx\n");
            user_req_tx = (tx_uart_req_t *)addr;

            uprintf("UART: check length\n");
            if(user_req_tx->length > (PAGE_SIZE / 2)){
                device_printf(sc->dev, "User Wants To Send Too Many Bytes\n");
                UART_UNLOCK(sc);
                return EINVAL;
            }

            // call uart transmit function
            uart_write(sc, user_req_tx);

            UART_UNLOCK(sc);
            break;
        default:
            error = ENOTTY;
            break;
    }

    return error;
}

/**
 * @brief Probe for a PL011 UART device on the ACPI bus.
 */
static int
uart_acpi_probe(device_t dev)
{
    if (ACPI_MATCH_PNP_INFO(dev, uart_ids) == NULL)
        return (ENXIO);
    
    device_set_desc(dev, "PL011 Cheri-aware UART");
    return (BUS_PROBE_DEFAULT);
}

/**
 * @brief Attach the driver to the UART device.
 */
static int
uart_acpi_attach(device_t dev)
{
    uart_softc_t *sc = device_get_softc(dev);
    sc->dev = dev;

    // Allocate memory-mapped I/O resource for UART registers
    int rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Failed to allocate memory resource.\n");
        return ENXIO;
    }
    sc->mem_rid = rid;
    sc->registers = rman_get_virtual(sc->mem_res);

    UART_LOCK_INIT(sc);

    // Create a unique sealing key for this device instance
    sc->sealing_key = create_sealing_key(device_get_unit(dev));
    if (sc->sealing_key == NULL) {
        device_printf(dev, "Failed to create sealing key.\n");
        bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
        UART_LOCK_DESTROY(sc);
        return ENXIO;
    }

    // Create the character device and allocate shared memory page
    if (create_our_cdev(sc) != 0) {
        device_printf(dev, "Failed to create character device.\n");
        bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
        UART_LOCK_DESTROY(sc);
        return ENXIO;
    }

    // Set a default configuration
    uart_config config = {
        .baudrate = 115200,
        .data_bits = 8,
        .parity = false,
        .stop_bits = 1
    };
    uart_configure(sc, &config);

    device_printf(dev, "attached\n");
    return 0;
}

/**
 * @brief Detach the driver from the UART device.
 */
static int
uart_acpi_detach(device_t dev)
{
    uart_softc_t *sc = device_get_softc(dev);
    int rid = 0;

    // Destroy character device, which may fail if device is busy
    if (destroy_our_cdev(sc) != 0) {
        return EBUSY;
    }

    revoke_cap_token(sc);
    
    if (sc->mem_res) {
        bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
    }

    UART_LOCK_DESTROY(sc);

    device_printf(dev, "detached\n");
    return 0;
}

// Device method table
static device_method_t uart_methods[] = {
    // Device interface
    DEVMETHOD(device_probe,     uart_acpi_probe),
    DEVMETHOD(device_attach,    uart_acpi_attach),
    DEVMETHOD(device_detach,    uart_acpi_detach),

    DEVMETHOD_END
};

// Driver definition
static driver_t uart_acpi_driver = {
    "uart-cheri",
    uart_methods,
    sizeof(uart_softc_t),
};

// Register the driver with the system
DRIVER_MODULE(uart_cheri, acpi, uart_acpi_driver, 0, 0);
MODULE_VERSION(uart_cheri, 1);