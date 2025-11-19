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

#define ADCRATE   200000         // 200 kHz
#define TIMEOUT   3124          // PCLK_DAC / ADCRATE (25 MHz / 200 kHz)
#define LISTSIZE  1024              // cantidad de muestras
#define ADDRESS   ((uint32_t)0x2007C000)   // buffer en RAM2

//static uint16_t * const adc_dac_buffer = (uint16_t *)ADDRESS;
uint32_t muestras_adc[LISTSIZE];
uint32_t muestras_dac[LISTSIZE];
uint32_t index = 0;
uint32_t buffer_full = 0;
uint32_t dmaON = 0;

void configDAC();
void configDMA();
void configPCB();
void configADC();

//------------------ Configuración de pines -------------------------
void configPCB(void) {
    PINSEL_CFG_Type pin;

    // P0.23 -> AD0.0
    pin.Portnum   = PINSEL_PORT_0;
    pin.Pinnum    = PINSEL_PIN_24;
    pin.Funcnum   = PINSEL_FUNC_1;              // AD0.0
    //pin.Pinmode   = PINSEL_PINMODE_TRISTATE;
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
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE);

    // No quiero interrupción por ADC, lo usaré con DMA
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, ENABLE);

    // BURST = ON para que el ADC convierta continuamente y
    // genere peticiones de DMA.
    ADC_BurstCmd(LPC_ADC, ENABLE);
}

void ADC_IRQHandler(){
	   uint16_t raw = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_1);
	   uint16_t sample12 = (raw >> 4) & 0x0FFF;   // 12 bits puros

	   if (!buffer_full) {
	           muestras_adc[index] = sample12;
	           muestras_adc[index] = muestras_adc[index] <<2;
	           //DAC_UpdateValue(LPC_DAC, adc_buffer[index]);
	           index++;
	           if (index >= LISTSIZE) {
	               buffer_full = 1;
	               index = 0;
	           }
	       }else{
	    	ADC_BurstCmd(LPC_ADC,DISABLE);
	    	for(uint32_t i = 0; i<LISTSIZE; i++){
	    		    		muestras_dac[i] = (muestras_adc[i] << 6);
	    		    	}
	    	if(dmaON == 0){
	    		configDMA();

	    		dmaON = 1;
	    	}
	    	//ADC_BurstCmd(LPC_ADC,ENABLE);
	       	//buffer_full = 0;
	       }
}

//------------------ DAC + DMA request ------------------------------
void configDAC(void) {
    DAC_CONVERTER_CFG_Type dacCfg;

    DAC_Init(LPC_DAC);
    DAC_SetBias(LPC_DAC, 0);    // baja impedancia (más corriente, más rápido)

    // Habilito contador interno + DMA en el DAC
    dacCfg.CNT_ENA = 1;
    dacCfg.DMA_ENA = 1;
    DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);

    // Timeout para que las peticiones de DMA del DAC vayan a ADCRATE
    //uint32_t tmp = (25*1000000)/(TIMEOUT);
    DAC_SetDMATimeOut(LPC_DAC, 25000000/ADCRATE);
}

//------------------ Inicialización del controlador GPDMA -----------
void configDMA(void) {
    GPDMA_Init();
    GPDMA_Channel_CFG_Type cfg;
    GPDMA_LLI_Type lli;

	lli.SrcAddr= (uint32_t)muestras_dac;
	lli.DstAddr= (uint32_t)&(LPC_DAC->DACR);
	lli.NextLLI= (uint32_t)&lli;
	lli.Control= LISTSIZE
			| (2<<12) //source width 32 bit
			| (1<<15) //dest. width 16 bit
			| (1<<26) //source increment
			| (1<<18)
			| (1<<22)
			| (0<<27)
			;

    cfg.ChannelNum    = 1;
    cfg.SrcMemAddr    = (uint32_t)muestras_dac;   // fuente = RAM
    cfg.DstMemAddr    = 0;                          // destino = periférico
    cfg.TransferSize  = LISTSIZE;
    cfg.TransferWidth = GPDMA_WIDTH_WORD;
    cfg.TransferType  = GPDMA_TRANSFERTYPE_M2P;
    cfg.SrcConn       = 0;
    cfg.DstConn       = GPDMA_CONN_DAC;
    cfg.DMALLI        = (uint32_t)&lli;

    GPDMA_Setup(&cfg);
    GPDMA_ChannelCmd(1, ENABLE);
}



//------------------ main -------------------------------------------
int main(void) {
    //SystemInit();

    configPCB();
    configDAC();
    configADC();


    NVIC_EnableIRQ(ADC_IRQn);

    // Arranco con una fase, por ejemplo grabar primero:


    while (1) {

        __WFI();        // bajo consumo mientras espero interrupciones
    }
}
