#include <string.h>
#include <stdlib.h>
#include "LPC17xx.h"
#include "lpc_types.h"

#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"



#define UART0_DEV   ((LPC_UART_TypeDef*)LPC_UART0)

#ifndef HAVE_STRNLEN
static size_t my_strnlen(const char *s, size_t maxlen) {
    size_t i = 0;
    if (!s) return 0;
    while (i < maxlen && s[i] != '\0') { ++i; }
    return i;
}
#define strnlen my_strnlen
#endif


/* -------------------- Variables estáticas (ámbito del driver) -------------------- */

/* Cola simple de mensajes por línea (slots de tamaño fijo) */
typedef struct {
    uint16_t len;                          /* longitud útil a transmitir (bytes) */
    char     buf[UART_TX_SLOT_SIZE];       /* contenido ASCII (no es necesario '\0' para DMA) */
} tx_slot_t;

/* Cola y punteros (ámbito del driver) */
static volatile tx_slot_t tx_queue[UART_TX_SLOTS];
static volatile uint8_t   tx_head_idx = 0;     /* próximo slot libre para escribir */
static volatile uint8_t   tx_tail_idx = 0;     /* próximo slot pendiente por enviar */
static volatile Bool      dma_busy = FALSE;

/* Canal de DMA usado para TX UART0 */
#define UART0_TX_DMA_CH     (0)

/* -------------------- Helpers privados ----------------------------------------- */

/* Avanza un índice circular de 0..UART_TX_SLOTS-1 */
static inline uint8_t idx_next(uint8_t i) {
    return (uint8_t)((i + 1u) % UART_TX_SLOTS);
}

/* ¿Cola llena? (head siguiente == tail) */
static inline Bool queue_is_full(void) {
    return (idx_next(tx_head_idx) == tx_tail_idx) ? TRUE : FALSE;
}

/* ¿Cola vacía? (head == tail) */
static inline Bool queue_is_empty(void) {
    return (tx_head_idx == tx_tail_idx) ? TRUE : FALSE;
}

static inline uint8_t queue_free_slots(void){
    uint8_t N = UART_TX_SLOTS;
    return (uint8_t)((tx_tail_idx + N - tx_head_idx - 1u) % N);
}

uint8_t uart_tx_queue_free_slots(void){ return queue_free_slots(); }
Bool    uart_tx_queue_full(void){ return queue_is_full(); }

/* Lanza (si corresponde) una transferencia DMA con el slot en tx_tail */
static void start_dma_if_idle(void) {
    if (dma_busy || queue_is_empty()) {
        return; /* Nada que hacer */
    }

    /* Objeto de configuración de canal DMA (NXP driver) */
    GPDMA_Channel_CFG_Type cfg;

    /* NOTA: TransferWidth sólo aplica para M2M; en M2P el driver setea 8-bit por defecto.
       Usamos: TransferType = M2P, SrcMemAddr = dirección del slot, DstConn = UART0_Tx */
    cfg.ChannelNum   = UART0_TX_DMA_CH;
    cfg.TransferSize = tx_queue[tx_tail_idx].len;
    cfg.TransferWidth= 0; /* sin uso en M2P */
    cfg.SrcMemAddr   = (uint32_t)tx_queue[tx_tail_idx].buf;
    cfg.DstMemAddr   = 0;
    cfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
    cfg.SrcConn      = 0; /* sin uso en M2P */
    cfg.DstConn      = GPDMA_CONN_UART0_Tx;
    cfg.DMALLI       = 0; /* sin LLI (un slot por TC) */

    /* Configura el canal con el bloque actual */
    if (GPDMA_Setup(&cfg) == SUCCESS) {
        dma_busy = TRUE;
        GPDMA_ChannelCmd(UART0_TX_DMA_CH, ENABLE); /* Dispara la transferencia */
    }
    else {
        /* Si falló la configuración (no debería), marcamos como inactivo para reintentar luego */
        dma_busy = FALSE;
    }
}

/* -------------------- API pública ---------------------------------------------- */

void config_uart_dma(uint32_t baud)
{
    /* 1) Configurar pines P0.2 (TXD0) y P0.3 (RXD0) para función UART0 */
    PINSEL_CFG_Type pin;
    pin.OpenDrain = PINSEL_PINMODE_NORMAL;
    pin.Pinmode   = PINSEL_PINMODE_PULLUP;
    pin.Portnum   = PINSEL_PORT_0;

    pin.Funcnum   = PINSEL_FUNC_1; /* TXD0 en P0.2 */
    pin.Pinnum    = 2;
    PINSEL_ConfigPin(&pin);

    pin.Funcnum   = PINSEL_FUNC_1; /* RXD0 en P0.3 */
    pin.Pinnum    = 3;
    PINSEL_ConfigPin(&pin);

    /* 2) Inicializar UART0: 8N1, baud solicitado */
    UART_CFG_Type uart_cfg;
    UART_ConfigStructInit(&uart_cfg);
    uart_cfg.Baud_rate = baud;                   /* 115200 típicamente */
    uart_cfg.Parity    = UART_PARITY_NONE;
    uart_cfg.Databits  = UART_DATABIT_8;
    uart_cfg.Stopbits  = UART_STOPBIT_1;

    UART_Init(UART0_DEV, &uart_cfg);

    /* FIFO con modo DMA habilitado y trigger medio (8 bytes) para RX
       (aunque RX se procese por IRQ u otro módulo, el DMAMODE habilita las req del periférico) */
    UART_FIFO_CFG_Type fifo_cfg;
    UART_FIFOConfigStructInit(&fifo_cfg);
    fifo_cfg.FIFO_DMAMode  = ENABLE;             /* <-- Habilita solicitudes DMA en UART */
    fifo_cfg.FIFO_Level    = UART_FIFO_TRGLEV2;  /* 8 bytes */
    fifo_cfg.FIFO_ResetRxBuf = ENABLE;
    fifo_cfg.FIFO_ResetTxBuf = ENABLE;
    UART_FIFOConfig(UART0_DEV, &fifo_cfg);

    /* Habilitar transmisión */
    UART_TxCmd(UART0_DEV, ENABLE);


    /* 3) Inicializar GPDMA */
    GPDMA_Init();

    /* Limpieza de estado de la cola */
    tx_head_idx = tx_tail_idx = 0;
    dma_busy = FALSE;

    /* 4) Habilitar IRQ de DMA en NVIC (para TC del canal 0) */
    NVIC_EnableIRQ(DMA_IRQn);
}

static Status enqueue_common(const char *line, Bool allow_when_full){
    if (!line) return ERROR;
    __disable_irq();
    if (!allow_when_full && queue_is_full()){ __enable_irq(); return ERROR; }

    uint16_t len = (uint16_t)strnlen(line, UART_TX_SLOT_SIZE - 1);
    Bool ends_nl = (len>0 && line[len-1]=='\n')?TRUE:FALSE;

    if (ends_nl){
        memcpy((void*)tx_queue[tx_head_idx].buf, line, len);
        tx_queue[tx_head_idx].len = len;
    } else {
        uint16_t maxcopy = (UART_TX_SLOT_SIZE - 1 > len) ? len : (UART_TX_SLOT_SIZE - 1);
        memcpy((void*)tx_queue[tx_head_idx].buf, line, maxcopy);
        if (maxcopy < UART_TX_SLOT_SIZE) tx_queue[tx_head_idx].buf[maxcopy++] = '\n';
        tx_queue[tx_head_idx].len = maxcopy;
    }
    tx_head_idx = idx_next(tx_head_idx);
    start_dma_if_idle();
    __enable_irq();
    return SUCCESS;
}

Status uart_tx_enqueue(const char *line)
{
    if (line == NULL) return ERROR;
    /* Verifica espacio en cola */
    __disable_irq();
    if (queue_is_full()) {
        __enable_irq();
        return ERROR; /* Cola llena */
    }

    /* Copiar contenido al slot 'head', garantizando '\n' al final */
    uint16_t len = (uint16_t)strnlen(line, UART_TX_SLOT_SIZE - 1);
    /* ¿La línea ya trae '\n'? si no, lo agregamos si hay espacio */
    Bool ends_nl = (len > 0 && line[len - 1] == '\n') ? TRUE : FALSE;

    if (ends_nl) {
        /* Copiamos tal cual (hasta UART_TX_SLOT_SIZE-1) */
        memcpy((void*)tx_queue[tx_head_idx].buf, line, len);
        tx_queue[tx_head_idx].len = len;
    } else {
        /* Copiar y agregar '\n' si cabe */
        uint16_t maxcopy = (UART_TX_SLOT_SIZE - 1 > len) ? len : (UART_TX_SLOT_SIZE - 1);
        memcpy((void*)tx_queue[tx_head_idx].buf, line, maxcopy);
        if (maxcopy < UART_TX_SLOT_SIZE) {
            tx_queue[tx_head_idx].buf[maxcopy++] = '\n';
        }
        tx_queue[tx_head_idx].len = maxcopy;
    }

    /* Avanzar head */
    tx_head_idx = idx_next(tx_head_idx);

    /* Si no hay DMA en curso, iniciar inmediatamente */
    start_dma_if_idle();

    __enable_irq();
    return SUCCESS;
}
Status uart_tx_try_enqueue(const char *line){       // “try”: falla si cola llena (ADC)
    /* Evitamos la condición de carrera: delegamos en enqueue_common()
       solicitando que compruebe la cola CON las IRQ deshabilitadas.
       De este modo si la cola está llena devolvemos ERROR de forma segura. */
    return enqueue_common(line, FALSE);
}

Bool uart_tx_is_busy(void)
{
    return dma_busy;
}

/* -------------------- IRQ de DMA (encadenamiento por TC) ----------------------- */
/* Interrupción global del controlador GPDMA */
void DMA_IRQHandler(void)
{
    /* ¿Terminal Count en canal 0? */
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, UART0_TX_DMA_CH)) {
        /* Limpiar TC en canal 0 */
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, UART0_TX_DMA_CH);

        /* Avanzar cola: ya se transmitió tx_tail */
        tx_tail_idx = idx_next(tx_tail_idx);

        /* Marcar canal como libre y disparar siguiente si hay */
        dma_busy = FALSE;
        start_dma_if_idle();
    }

    /* ¿Error en canal 0? (opcional: reintento o descartar) */
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, UART0_TX_DMA_CH)) {
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, UART0_TX_DMA_CH);
        /* Recuperación simple: marcar libre y avanzar (evita quedar colgado) */
        tx_tail_idx = idx_next(tx_tail_idx);
        dma_busy = FALSE;
        start_dma_if_idle();
    }
}