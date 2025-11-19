/*
PASAR DEL ADC AL DMA Y DEL DMA AL DAC USANDO INTERRUPCIONES DEL DMA Y BURST MODE EN EL ADC Y DAC
(Aca va a pasar que se van a producir cortes en la señal de salida del DAC cada vez que se complete)
*/

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"

#define ADCRATE   1000          // 200 kHz
#define TIMEOUT   25000           // PCLK_DAC / ADCRATE (25 MHz / 200 kHz)
#define LISTSIZE  4095              // cantidad de muestras
#define ADDRESS   ((uint32_t)0x2007C000)   // buffer en RAM2

static uint16_t * const adc_dac_buffer = (uint16_t *)ADDRESS;

//------------------ Configuración de pines -------------------------
void configPCB(void) {
    PINSEL_CFG_Type pin;

    // P0.23 -> AD0.0
    pin.Portnum   = PINSEL_PORT_0;
    pin.Pinnum    = PINSEL_PIN_23;
    pin.Funcnum   = PINSEL_FUNC_1;              // AD0.0
    pin.Pinmode   = PINSEL_PINMODE_TRISTATE;
    pin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&pin);

    // P0.26 -> AOUT (DAC)
    pin.Pinnum  = PINSEL_PIN_26;
    pin.Funcnum = PINSEL_FUNC_2;               // AOUT
    PINSEL_ConfigPin(&pin);
}

//------------------ ADC + DMA request ------------------------------
void configADC(void) {
    // Inicializo ADC con tasa de conversión deseada
    ADC_Init(LPC_ADC, ADCRATE);

    // Habilito canal 0
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);

    // No quiero interrupción por ADC, lo usaré con DMA
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, DISABLE);

    // BURST = ON para que el ADC convierta continuamente y
    // genere peticiones de DMA.
    ADC_BurstCmd(LPC_ADC, ENABLE);
}

//------------------ DAC + DMA request ------------------------------
void configDAC(void) {
    DAC_CONVERTER_CFG_Type dacCfg;

    DAC_Init(LPC_DAC);
    DAC_SetBias(LPC_DAC, 0);    // baja impedancia (más corriente, más rápido)

    CLKPWR_SetPCLKDiv (CLKPWR_PCLKSEL_DAC, CLKPWR_PCLKSEL_CCLK_DIV_4);
    // Habilito contador interno + DMA en el DAC
    dacCfg.CNT_ENA = SET;
    dacCfg.DMA_ENA = SET;
    DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);

    // Timeout para que las peticiones de DMA del DAC vayan a ADCRATE
    DAC_SetDMATimeOut(LPC_DAC, TIMEOUT);
}

//------------------ Inicialización del controlador GPDMA -----------
void configDMA(void) {
    GPDMA_Init();
    // Si quisieras usar interrupciones del DMA:
    NVIC_EnableIRQ(DMA_IRQn);
}

//------------------ DMA: ADC -> RAM (canal 0, P2M) ----------------
void configDmaAdc(void) {
    GPDMA_Channel_CFG_Type cfg;
    NVIC_DisableIRQ(DMA_IRQn);
    GPDMA_ChannelCmd(0, DISABLE);

    cfg.ChannelNum    = 0;
    cfg.SrcMemAddr    = 0;                          // fuente = periférico
    cfg.DstMemAddr    = (uint32_t)adc_dac_buffer;   // destino = RAM
    cfg.TransferSize  = LISTSIZE;
    cfg.TransferWidth = GPDMA_WIDTH_HALFWORD;
    cfg.TransferType  = GPDMA_TRANSFERTYPE_P2M;
    cfg.SrcConn       = GPDMA_CONN_ADC;
    cfg.DstConn       = 0;
    cfg.DMALLI        = 0;                          // sin lista enlazada

    GPDMA_Setup(&cfg);
    GPDMA_ChannelCmd(0, ENABLE);
    NVIC_EnableIRQ(DMA_IRQn);
}

//------------------ DMA: RAM -> DAC (canal 1, M2P) ----------------
void configDmaDac(void) {
    GPDMA_Channel_CFG_Type cfg;
    NVIC_DisableIRQ(DMA_IRQn);
    GPDMA_ChannelCmd(1, DISABLE);

    cfg.ChannelNum    = 1;
    cfg.SrcMemAddr    = (uint32_t)adc_dac_buffer;   // fuente = RAM
    cfg.DstMemAddr    = 0;                          // destino = periférico
    cfg.TransferSize  = LISTSIZE;
    cfg.TransferWidth = GPDMA_WIDTH_HALFWORD;
    cfg.TransferType  = GPDMA_TRANSFERTYPE_M2P;
    cfg.SrcConn       = 0;
    cfg.DstConn       = GPDMA_CONN_DAC;
    cfg.DMALLI        = 0;

    GPDMA_Setup(&cfg);
    GPDMA_ChannelCmd(1, ENABLE);
    NVIC_EnableIRQ(DMA_IRQn);
}

void DMA_IRQHandler(void) {
    // Manejo interrupción del canal 0 (ADC -> RAM)
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) {
        GPDMA_ClearIntPending(GPDMA_STAT_INTTC, 0);
        // Aquí podrías hacer algo cuando termina la transferencia del ADC
        GPDMA_ChannelCmd(0, DISABLE);
        ADC_BurstCmd(LPC_ADC, DISABLE);
        convertBufferAdcToDac(adc_dac_buffer, LISTSIZE);
        configDmaDac();
    }

    // Manejo interrupción del canal 1 (RAM -> DAC)
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 1)) {
        GPDMA_ClearIntPending(GPDMA_STAT_INTTC, 1);
        // Aquí podrías hacer algo cuando termina la transferencia del DAC
        GPDMA_ChannelCmd(1, DISABLE);
        ADC_BurstCmd(LPC_ADC, ENABLE);
        configDmaAdc();
        
    }
}

// Convierte cada muestra del buffer desde formato ADC (ADGDR/ADDRx)
// a formato DACR (VALUE en bits 15:6).
void convertBufferAdcToDac(uint16_t *buf, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        uint16_t adc_word = buf[i];

        // 1) Extraer los 12 bits de resultado del ADC: bits 15:4
        uint16_t adc12 = (adc_word >> 4) & 0x0FFF;   // 0..4095

        // 2) Pasar de 12 bits a 10 bits: dividir por 4 (>> 2)
        uint16_t dac10 = adc12 >> 2;                 // 0..1023

        // 3) Alinear para el DACR: VALUE en bits 15:6
        uint16_t dac_word = (dac10 & 0x03FF) << 6;

        // Guardar de vuelta en el mismo buffer
        buf[i] = dac_word;
    }
}

//------------------ main -------------------------------------------
int main(void) {
    SystemInit();

    configPCB();
    configADC();
    configDAC();
    configDMA();

    // Arranco con una fase, por ejemplo grabar primero:
    configDmaAdc();

    while (1) {
        // El trabajo duro lo hacen ADC, DAC y DMA.
        __WFI();        // bajo consumo mientras espero interrupciones
    }
}
