#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Configuración */
#define BAUD_RATE       9600
#define RX_BUF_SIZE     128

/* Buffer circular RX */
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

/* Ticker de milisegundos */
static volatile uint32_t msTicks = 0;

/* Prototipos */
static void configPinsUART2(void);
static void configUART2(uint32_t baud);
static void uart_send_str(const char* s);
static void uart_send_bytes(const uint8_t* data, uint32_t len);
static int  rx_pop_byte(uint8_t* b);
static void delay_ms(uint32_t ms);

/* SysTick a 1 kHz */
void SysTick_Handler(void) {
    msTicks++;
}

int main(void)
{
    /* SysTick 1 kHz */
    SysTick_Config(SystemCoreClock / 1000);

    configPinsUART2();
    configUART2(BAUD_RATE);

    uart_send_str("LPC17xx UART2 listo @ ");
    char initMsg[32];
    sprintf(initMsg, "%lu bps\r\n", (unsigned long)BAUD_RATE);
    uart_send_str(initMsg);

    uint32_t n = 0;
    for (;;)
    {
        /* 1) Enviar patrón simple periódicamente */
        char line[32];
        int len = sprintf(line, "PING %lu\r\n", (unsigned long)n++);
        uart_send_bytes((uint8_t*)line, (uint32_t)len);

        /* 2) Si hay datos recibidos, hacer ECHO (hasta fin de línea o tope) */
        char echo[64];
        int i = 0;
        uint8_t c;

        while (i < (int)sizeof(echo) - 1 && rx_pop_byte(&c)) {
            echo[i++] = (char)c;
            if (c == '\n') break; /* eco por líneas si envías líneas desde Arduino */
        }
        if (i > 0) {
            uart_send_str("ECHO: ");
            uart_send_bytes((uint8_t*)echo, (uint32_t)i);
            /* Asegura CRLF si no vino con fin de línea */
            if (echo[i - 1] != '\n' && echo[i - 1] != '\r') {
                uart_send_str("\r\n");
            }
        }

        delay_ms(500);
    }
}

/* --------- Implementación --------- */

static void configPinsUART2(void)
{
    PINSEL_CFG_Type pinCFG;

    /* TXD2 -> P0.10, RXD2 -> P0.11 (Función 1) */
    pinCFG.Portnum   = PINSEL_PORT_0;
    pinCFG.Funcnum   = PINSEL_FUNC_1;
    pinCFG.OpenDrain = PINSEL_PINMODE_NORMAL;
    pinCFG.Pinmode   = PINSEL_PINMODE_TRISTATE;

    pinCFG.Pinnum    = PINSEL_PIN_10; /* TXD2 */
    PINSEL_ConfigPin(&pinCFG);

    pinCFG.Pinnum    = PINSEL_PIN_11; /* RXD2 */
    PINSEL_ConfigPin(&pinCFG);
}

static void configUART2(uint32_t baud)
{
    UART_CFG_Type      uartCfg;
    UART_FIFO_CFG_Type fifoCfg;

    UART_ConfigStructInit(&uartCfg);
    uartCfg.Baud_rate = baud; /* 8N1 por defecto */

    UART_Init(LPC_UART2, &uartCfg);

    UART_FIFOConfigStructInit(&fifoCfg);
    UART_FIFOConfig(LPC_UART2, &fifoCfg);

    /* Interrupciones por RX y estado de línea */
    UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig(LPC_UART2, UART_INTCFG_RLS, ENABLE);

    NVIC_EnableIRQ(UART2_IRQn);
}

static void uart_send_str(const char* s)
{
    if (!s) return;
    UART_Send(LPC_UART2, (uint8_t*)s, (uint32_t)strlen(s), BLOCKING);
}

static void uart_send_bytes(const uint8_t* data, uint32_t len)
{
    if (!data || !len) return;
    UART_Send(LPC_UART2, (uint8_t*)data, len, BLOCKING);
}

static inline uint32_t rb_next(uint32_t idx) {
    return (idx + 1U) & (RX_BUF_SIZE - 1U);
}

/* Extrae 1 byte del buffer RX si hay; retorna 1 si ok, 0 si vacío */
static int rx_pop_byte(uint8_t* b)
{
    if (rx_head == rx_tail) return 0; /* vacío */
    *b = rx_buf[rx_tail];
    rx_tail = rb_next(rx_tail);
    return 1;
}

static void delay_ms(uint32_t ms)
{
    uint32_t end = msTicks + ms;
    while ((int32_t)(end - msTicks) > 0) {
        __NOP();
    }
}

/* IRQ de UART2: maneja errores y llena el buffer RX */
void UART2_IRQHandler(void)
{
    uint32_t intsrc = UART_GetIntId(LPC_UART2);
    uint32_t iid = intsrc & UART_IIR_INTID_MASK;

    if (iid == UART_IIR_INTID_RLS) {
        /* Chequear errores de línea */
        uint32_t lsr = UART_GetLineStatus(LPC_UART2);
        if (lsr & (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE)) {
            /* Limpia leyendo si fuera necesario; aquí solo descartamos */
            (void)lsr;
        }
    }

    if (iid == UART_IIR_INTID_RDA || iid == UART_IIR_INTID_CTI) {
        uint8_t byte;
        /* Leer todos los disponibles en modo no bloqueante */
        while (UART_Receive(LPC_UART2, &byte, 1, NONE_BLOCKING) == 1) {
            uint32_t next = rb_next(rx_head);
            if (next != rx_tail) {
                rx_buf[rx_head] = byte;
                rx_head = next;
            } else {
                /* Overflow: descartamos el byte más viejo */
                rx_tail = rb_next(rx_tail);
                rx_buf[rx_head] = byte;
                rx_head = next;
            }
        }
    }
}