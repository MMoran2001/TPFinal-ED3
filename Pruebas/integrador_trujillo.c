#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

#define ADDRESS ((0x2007C000))

void configPCB(void);
void configDAC(void);
void configADC(void);
void configTimer(void);
void configDMA(void);
void configDmaAdc(void);
void configDmaDac(void);

uint16_t *buffer = (uint16_t*)ADDRESS;
GPDMA_LLI_Type adcLLI = {0};
GPDMA_LLI_Type dacLLI = {0};

int main (void) {
    configPCB();
    configDAC();
    configADC();
    configTimer();
    GPDMA_Init();
    configDmaAdc();

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
}

void configDAC(void) {
    DAC_CONVERTER_CFG_Type dacCfg = {ENABLE, ENABLE, ENABLE};
    DAC_Init(LPC_DAC);
    DAC_ConfigDAConverterControl(LPC_DAC,&dacCfg);
    DAC_SetDMATimeOut(LPC_DAC,0xFF);    // TODO
}

void configADC(void) {
    ADC_Init(LPC_ADC, 200000);   // TODO
    ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0, ENABLE);
    ADC_BurstCmd(LPC_ADC,ENABLE);
}

void configTimer(void) {
    TIM_TIMERCFG_Type timCfg = {TIM_PRESCALE_USVAL, 1000};
    TIM_MATCHCFG_Type matCfg = {0, ENABLE, DISABLE, ENABLE, TIM_EXTMATCH_NOTHING, 60000};

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timCfg);
    TIM_ConfigMatch(LPC_TIM0, &matCfg);
    TIM_Cmd(LPC_TIM0, ENABLE);

    NVIC_EnableIRQ(TIMER0_IRQn);
}

void configDmaAdc(void) {
    GPDMA_Channel_CFG_Type dmaCfgAdc = {0};
    dmaCfgAdc.ChannelNum = 0;
    dmaCfgAdc.TransferSize = 1024;
    dmaCfgAdc.SrcMemAddr = LPC_ADC->ADGDR;
    dmaCfgAdc.DstMemAddr = (uint32_t)(uintptr_t)buffer;
    dmaCfgAdc.TransferType = GPDMA_TRANSFERTYPE_P2M;
    dmaCfgAdc.SrcConn = GPDMA_CONN_ADC;
    dmaCfgAdc.DMALLI = (uint32_t)&adcLLI;

    adcLLI.SrcAddr = LPC_ADC->ADGDR;
    adcLLI.DstAddr = (uint32_t)(uintptr_t)buffer;
    adcLLI.NextLLI = (uint32_t)&adcLLI;
    adcLLI.Control = 1024 | 1 << 12 | 1 << 15 | 1 << 18 | 1 << 21;

    GPDMA_Setup(&dmaCfgAdc);
    GPDMA_ChannelCmd(1, DISABLE);
    GPDMA_ChannelCmd(0, ENABLE);
}

void configDmaDac(void) {
    GPDMA_Channel_CFG_Type dmaCfgDac = {0};
    dmaCfgDac.ChannelNum = 1;
    dmaCfgDac.TransferSize = 1024;
    dmaCfgDac.SrcMemAddr = (uint32_t)(uintptr_t)buffer;
    dmaCfgDac.DstMemAddr = LPC_DAC->DACR;
    dmaCfgDac.TransferType = GPDMA_TRANSFERTYPE_M2P;
    dmaCfgDac.DstConn = GPDMA_CONN_DAC;
    dmaCfgDac.DMALLI = (uint32_t)&dacLLI;

    dacLLI.SrcAddr = (uint32_t)(uintptr_t)buffer;
    dacLLI.DstAddr = LPC_DAC->DACR;
    dacLLI.NextLLI = (uint32_t)&dacLLI;
    dacLLI.Control = 1024 | 1 << 12 | 1 << 15 | 1 << 18 | 1 << 21;

    GPDMA_Setup(&dmaCfgDac);
    GPDMA_ChannelCmd(0, DISABLE);
    GPDMA_ChannelCmd(1, ENABLE);
}

void TIMER0_IRQHandler(void) {
    static uint8_t status = 0;
    if (status) {
        configDmaAdc();
    } else {
        configDmaDac();
    }
    status = !status;

    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}