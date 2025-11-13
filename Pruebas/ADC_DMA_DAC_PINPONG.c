#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

#define ADDRESS ((0x2007C000))
#define SAMPLE_RATE 8000
#define BUFFER_SIZE 512

GPDMA_LLI_Type lli_adc_ping, lli_adc_pong;
GPDMA_LLI_Type lli_dac_ping, lli_dac_pong;

uint16_t ping_buffer[BUFFER_SIZE];
uint16_t pong_buffer[BUFFER_SIZE];

void configPCB(void);
void configDAC(void);
void configADC(void);
void configTimer(void);
void configDMA(void);


int main (void) {
    SystemInit();
    configPCB();
    configDAC();
    configADC();
    configTimer();
    configDMA();
    

    // Habilitar canales DMA. El ADC comienza a llenar el buffer 'ping',
    // mientras el DAC comienza a vaciar el buffer 'pong' (que está lleno de ceros al inicio).
    GPDMA_ChannelCmd(0, ENABLE); // ADC
    GPDMA_ChannelCmd(1, ENABLE); // DAC

    // Habilitar el Timer, que es el corazón del sistema.
    TIM_Cmd(LPC_TIM0, ENABLE);

    while (1) {
        __WFI();
    }
}

void configPCB(void) {
    //PINSEL_CFG_Type pinCfg = {PINSEL_PORT_0, PINSEL_PIN_23, PINSEL_FUNC_1, PINSEL_TRISTATE};
    PINSEL_CFG_Type pinCfg = {0};
    pinCfg.Portnum = PINSEL_PORT_0;
    pinCfg.Pinnum = PINSEL_PIN_23;
    pinCfg.Funcnum = PINSEL_FUNC_1;
    pinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&pinCfg);

    // Configurar P0.26 como salida de DAC (AOUT)
    pinCfg.Portnum = PINSEL_PORT_0;
    pinCfg.Pinnum = PINSEL_PIN_26;
    pinCfg.Funcnum = PINSEL_FUNC_2;
    PINSEL_ConfigPin(&pinCfg);

}

void configDAC(void) {
    DAC_CONVERTER_CFG_Type dacCfg = {0};
    dacCfg.DBLBUF_ENA = DISABLE;
    dacCfg.CNT_ENA = ENABLE; // El contador del DAC se usará para temporizar las transferencias DMA
    dacCfg.DMA_ENA = ENABLE; // Habilitar DMA para el DAC

    DAC_Init(LPC_DAC);
    DAC_SetDMATimeOut(LPC_DAC, (SystemCoreClock/4)/SAMPLE_RATE); // Configurar el timeout del DAC
    DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
    
}

void configADC(void) {
    ADC_Init(LPC_ADC, 200000);   // TODO
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
    ADC_BurstCmd(LPC_ADC, DISABLE);
    ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
}

void configTimer(void) {
    TIM_TIMERCFG_Type timCfg = {0};
    TIM_MATCHCFG_Type matCfg = {0};

    timCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timCfg.PrescaleValue = 1; // 1 us

    matCfg.MatchChannel = 1;
    matCfg.IntOnMatch = DISABLE; // No queremos interrupciones del timer
    matCfg.ResetOnMatch = ENABLE;
    matCfg.StopOnMatch = DISABLE;
    // El match del timer genera un evento de hardware, pero no interrumpe la CPU
    matCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    matCfg.MatchValue = (1000000 / SAMPLE_RATE) - 1; // Periodo de 1/SAMPLE_RATE segundos

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timCfg);
    TIM_ConfigMatch(LPC_TIM0, &matCfg);
    TIM_Cmd(LPC_TIM0, DISABLE);
}

void configDMA(void){
    GPDMA_Init();
     // --- Configuración del Canal 0: ADC a Memoria (P2M) ---
    GPDMA_Channel_CFG_Type dmaCfgAdc;
    dmaCfgAdc.ChannelNum = 0;
    dmaCfgAdc.TransferSize = BUFFER_SIZE;
    dmaCfgAdc.TransferWidth = GPDMA_WIDTH_HALFWORD; // 16 bits
    dmaCfgAdc.SrcConn = GPDMA_CONN_ADC;
    dmaCfgAdc.DstConn = 0;
    dmaCfgAdc.TransferType = GPDMA_TRANSFERTYPE_P2M;
    dmaCfgAdc.SrcMemAddr = 0; // No se usa para P2M
    dmaCfgAdc.DstMemAddr = (uint32_t)ping_buffer;
    dmaCfgAdc.DMALLI = (uint32_t)&lli_adc_pong; // Al terminar, salta a la LLI del pong buffer

    // LLI para que el ADC llene el buffer 'pong' y luego vuelva al 'ping'
    lli_adc_pong.SrcAddr = 0;
    lli_adc_pong.DstAddr = (uint32_t)pong_buffer;
    lli_adc_pong.NextLLI = (uint32_t)&lli_adc_ping;
    lli_adc_pong.Control = BUFFER_SIZE
                         | (GPDMA_WIDTH_HALFWORD << 18) // Ancho Origen
                         | (GPDMA_WIDTH_HALFWORD << 21) // Ancho Destino
                         | (1 << 27);                   // Incremento en Destino

    // LLI para que el ADC llene el buffer 'ping' y luego vuelva al 'pong'
    lli_adc_ping.SrcAddr = 0;
    lli_adc_ping.DstAddr = (uint32_t)ping_buffer;
    lli_adc_ping.NextLLI = (uint32_t)&lli_adc_pong;
    lli_adc_ping.Control = lli_adc_pong.Control;

    GPDMA_Setup(&dmaCfgAdc);

    // --- Configuración del Canal 1: Memoria a DAC (M2P) ---
    GPDMA_Channel_CFG_Type dmaCfgDac;
    dmaCfgDac.ChannelNum = 1;
    dmaCfgDac.TransferSize = BUFFER_SIZE;
    dmaCfgDac.TransferWidth = GPDMA_WIDTH_HALFWORD; // 16 bits
    dmaCfgDac.SrcConn = 0;
    dmaCfgDac.DstConn = GPDMA_CONN_DAC;
    dmaCfgDac.TransferType = GPDMA_TRANSFERTYPE_M2P;
    dmaCfgDac.SrcMemAddr = (uint32_t)pong_buffer; // Empieza leyendo el pong buffer
    dmaCfgDac.DstMemAddr = 0; // No se usa para M2P
    dmaCfgDac.DMALLI = (uint32_t)&lli_dac_ping;

    // LLI para que el DAC lea del buffer 'ping' y luego vuelva al 'pong'
    lli_dac_ping.SrcAddr = (uint32_t)ping_buffer;
    lli_dac_ping.DstAddr = 0;
    lli_dac_ping.NextLLI = (uint32_t)&lli_dac_pong;
    lli_dac_ping.Control = BUFFER_SIZE
                         | (GPDMA_WIDTH_HALFWORD << 18)
                         | (GPDMA_WIDTH_HALFWORD << 21)
                         | (1 << 26);                   // Incremento en Origen

    // LLI para que el DAC lea del buffer 'pong' y luego vuelva al 'ping'
    lli_dac_pong.SrcAddr = (uint32_t)pong_buffer;
    lli_dac_pong.DstAddr = 0;
    lli_dac_pong.NextLLI = (uint32_t)&lli_dac_ping;
    lli_dac_pong.Control = lli_dac_ping.Control;

    GPDMA_Setup(&dmaCfgDac);

    // No se necesitan interrupciones en este modo de passthrough totalmente automático
    NVIC_DisableIRQ(DMA_IRQn);
}

