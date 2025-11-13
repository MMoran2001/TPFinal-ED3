#include <stdio.h>
#include <stdlib.h>
#include <LPC17xx.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_timer.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_gpdma.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_exti.h>

// --- Definiciones ---
#define ADCRATE 8000		// Frecuencia de muestreo
#define LISTSIZE 4000		// Tamaño del buffer de sonido (aprox 0.5s a 8kHz)

// --- Teclado (Keypad) ---
#define ROW_PORT 2
#define COL_PORT 2
static const uint32_t ROW_BITS[] = { (1<<0), (1<<1), (1<<2), (1<<3)};
static const uint32_t COL_BITS[] = { (1<<4), (1<<5), (1<<6), (1<<7)};
static const char keymap[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// --- Variables Globales ---
static volatile char last_key = 0;
static volatile uint32_t debounce_ms = 20;
volatile uint32_t ticks_ms = 0;

volatile int recording = 0;
volatile int sound_index = 0;
volatile uint32_t samples_count = 0;

// --- Buffers de Sonido ---
uint32_t sound1_list[LISTSIZE] = {0};
uint32_t sound2_list[LISTSIZE] = {0};
uint32_t sound3_list[LISTSIZE] = {0};

// --- Prototipos ---
void configPin(void);
void configADC(void);
void configDAC(void);
void configTimer(void);
void configDMA(void);
void Keypad_Init(void);
void Keypad_GetCharNonBlock(char *ch);
void startPlayback(uint32_t* sound_list);
void delay_ms(uint32_t ms);

// --- Handlers de Interrupción ---
void SysTick_Handler(void);
void EINT3_IRQHandler(void);
void ADC_IRQHandler(void);
void DMA_IRQHandler(void);

int main (void){
   // --- Inicializaciones ---
   SysTick_Config(SystemCoreClock / 1000); // SysTick a 1ms
   Keypad_Init();
   configPin();
   configADC();
   configDAC();
   configTimer();
   configDMA();

   // Apagar todos los LEDs al inicio
   GPIO_SetValue(0, (1<<22)); // Rojo
   GPIO_SetValue(3, (1<<25)); // Verde
   GPIO_SetValue(3, (1<<26)); // Azul

   while (1){
       char ch;
       Keypad_GetCharNonBlock(&ch);

        switch(ch){
            case '1':
                if (!recording) {
                    startPlayback(sound1_list);
                    GPIO_ClearValue(0, (1<<22)); // Led Rojo ON
                    GPIO_SetValue(3, (1<<25));
                    GPIO_SetValue(3, (1<<26));
                }
                break;
            case '2':
                if (!recording) {
                    startPlayback(sound2_list);
                    GPIO_ClearValue(3, (1<<25)); // Led Verde ON
                    GPIO_SetValue(0, (1<<22));
                    GPIO_SetValue(3, (1<<26));
                }
                break;
            case '3':
                if (!recording) {
                    startPlayback(sound3_list);
                    GPIO_ClearValue(3, (1<<26)); // Led Azul ON
                    GPIO_SetValue(0, (1<<22));
                    GPIO_SetValue(3, (1<<25));
                }
                break;

            case '9':
                recording = 1;
                sound_index = 0;
                samples_count = 0;

                // Indicar modo "grabación armada" (ej: parpadeo o color específico)
                GPIO_ClearValue(0, (1<<22)); // Led Rojo ON
                GPIO_ClearValue(3, (1<<25)); // Led Verde ON

                while (recording) {
                    Keypad_GetCharNonBlock(&ch);

                    // Iniciar grabación al presionar 1, 2 o 3
                    if (sound_index == 0 && (ch == '1' || ch == '2' || ch == '3')) {
                        if (ch == '1') sound_index = 1;
                        if (ch == '2') sound_index = 2;
                        if (ch == '3') sound_index = 3;

                        // Iniciar el proceso de hardware para la grabación
                        ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
                        NVIC_EnableIRQ(ADC_IRQn);
                        TIM_Cmd(LPC_TIM0, ENABLE);
                    }

                    // Salir del modo grabación si se presiona 9 de nuevo
                    if (ch == '9') {
                        recording = 0;
                    }

                    // El bucle también terminará cuando 'recording' se ponga a 0 en la ISR del ADC
                }

                // Limpieza al salir del modo grabación
                TIM_Cmd(LPC_TIM0, DISABLE);
                ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE);
                NVIC_DisableIRQ(ADC_IRQn);
                GPIO_SetValue(0, (1<<22)); // Apagar todos los LEDs
                GPIO_SetValue(3, (1<<25));
                GPIO_SetValue(3, (1<<26));
                break;

            default:
                break;
      }
    }
    return 0;
}

// --- Configuraciones de Periféricos ---

void configPin(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;

    // ADC (Microfono) en AD0.0
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&PinCfg);

    // DAC (Salida de audio) en AOUT
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Funcnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    // LEDs
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.Portnum = 0; PinCfg.Pinnum = 22; PINSEL_ConfigPin(&PinCfg); // Rojo
    PinCfg.Portnum = 3; PinCfg.Pinnum = 25; PINSEL_ConfigPin(&PinCfg); // Verde
    PinCfg.Portnum = 3; PinCfg.Pinnum = 26; PINSEL_ConfigPin(&PinCfg); // Azul
    GPIO_SetDir(0, (1<<22), 1);
    GPIO_SetDir(3, (1<<25), 1);
    GPIO_SetDir(3, (1<<26), 1);
}

void configADC(void){
    ADC_Init(LPC_ADC, ADCRATE);
    ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01); // Usando el define que especificaste.
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
    ADC_BurstCmd(LPC_ADC, DISABLE);
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1); // Prioridad alta para la adquisición
}

void configDAC(void){
    DAC_CONVERTER_CFG_Type dacCfg;
    dacCfg.DBLBUF_ENA = 0;
    dacCfg.CNT_ENA = 1;
    dacCfg.DMA_ENA = 1;

    uint32_t dac_timeout = 25000000 / ADCRATE;

    DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
    DAC_SetDMATimeOut(LPC_DAC, dac_timeout);
    DAC_Init(LPC_DAC);
}

void configTimer(void){
    TIM_TIMERCFG_Type TIM_ConfigStruct;
    TIM_MATCHCFG_Type TIM_MatchConfigStruct;

    TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIM_ConfigStruct.PrescaleValue = 1;

    TIM_MatchConfigStruct.MatchChannel = 1;
    TIM_MatchConfigStruct.IntOnMatch = DISABLE;
    TIM_MatchConfigStruct.ResetOnMatch = ENABLE;
    TIM_MatchConfigStruct.StopOnMatch = DISABLE;
    TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
    TIM_MatchConfigStruct.MatchValue = 1000000 / ADCRATE;

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIM_ConfigStruct);
    TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);
    TIM_Cmd(LPC_TIM0, DISABLE);
}

void configDMA(void){
    GPDMA_Init();
    NVIC_DisableIRQ(DMA_IRQn);
    NVIC_SetPriority(DMA_IRQn, 2);
}

// --- Funciones Auxiliares y de Reproducción ---

void startPlayback(uint32_t* sound_list) {
    GPDMA_ChannelCmd(0, DISABLE);

    GPDMA_Channel_CFG_Type DMA_CFG;
    DMA_CFG.ChannelNum = 0;
    DMA_CFG.SrcMemAddr = (uint32_t)sound_list;
    DMA_CFG.DstMemAddr = (uint32_t)&(LPC_DAC->DACR);
    DMA_CFG.TransferSize = LISTSIZE;
    DMA_CFG.TransferWidth = GPDMA_WIDTH_WORD; // Transferencia de 32 bits
    DMA_CFG.TransferType = GPDMA_TRANSFERTYPE_M2P;
    DMA_CFG.SrcConn = 0;
    DMA_CFG.DstConn = GPDMA_CONN_DAC;
    DMA_CFG.DMALLI = 0;

    GPDMA_Setup(&DMA_CFG);
    NVIC_EnableIRQ(DMA_IRQn);
    GPDMA_ChannelCmd(0, ENABLE);
}

// --- Handlers de Interrupción ---

void SysTick_Handler(void){
    ticks_ms++;
}

void ADC_IRQHandler(void) {
    if (ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)) {
        uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0);

        if (recording && samples_count < LISTSIZE && sound_index != 0) {
            uint32_t dac_data = (adc_value << 6); // Alinea el dato del ADC para el DAC

            if (sound_index == 1) {
                sound1_list[samples_count] = dac_data;
            } else if (sound_index == 2) {
                sound2_list[samples_count] = dac_data;
            } else if (sound_index == 3) {
                sound3_list[samples_count] = dac_data;
            }
            samples_count++;
        }

        if (samples_count >= LISTSIZE) {
            recording = 0; // Detiene el bucle en main
        }
    }
}

void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0)) {
        if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) {
            GPDMA_ChannelCmd(0, DISABLE);
            GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
            // Apagar el LED correspondiente al finalizar la reproducción
            GPIO_SetValue(0, (1<<22));
            GPIO_SetValue(3, (1<<25));
            GPIO_SetValue(3, (1<<26));
        }
    }
}

// --- Código del Teclado ---

void delay_ms(uint32_t ms){
  uint32_t start_ticks = ticks_ms;
  while ((ticks_ms - start_ticks) < ms) {}
}

void rows_all_high(void){
  for (int i=0;i<4;i++) GPIO_SetValue(ROW_PORT, ROW_BITS[i]);
}

void rows_all_down (void){
	for (int i=0;i<4;i++) GPIO_ClearValue(ROW_PORT, ROW_BITS[i]);
}

int read_col_index(void){
  uint32_t pins = GPIO_ReadValue(COL_PORT);
  for (int c=0;c<4;c++){
    if ((pins & COL_BITS[c]) == 0) return c;
  }
  return -1;
}

int scan_key_position(int *r_out, int *c_out){
  for (int r=0;r<4;r++){
    rows_all_high();
    GPIO_ClearValue(ROW_PORT, ROW_BITS[r]);
    for (volatile int d=0; d<200; d++) __NOP();

    int c = read_col_index();
    if (c >= 0){
      *r_out = r;
      *c_out = c;
      rows_all_high();
      return 1;
    }
  }
  rows_all_high();
  return 0;
}

void keypad_pins_init(void){
  PINSEL_CFG_Type cfg;
  cfg.Funcnum = 0;
  cfg.OpenDrain = 0;
  cfg.Portnum = ROW_PORT;

  for (int i=0;i<4;i++){
    cfg.Pinnum = i;
    cfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&cfg);
    GPIO_SetDir(ROW_PORT, ROW_BITS[i], 1);
  }

  cfg.Portnum = COL_PORT;
  for (int i=0;i<4;i++){
    cfg.Pinnum = 4 + i;
    cfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&cfg);
    GPIO_SetDir(COL_PORT, COL_BITS[i], 0);
  }
  rows_all_high();
}

void keypad_irq_init(void){
  uint32_t col_mask = COL_BITS[0] | COL_BITS[1] | COL_BITS[2] | COL_BITS[3];
  GPIO_IntCmd(COL_PORT, col_mask, 0); // Interrupción por flanco de bajada
  NVIC_EnableIRQ(EINT3_IRQn);
  NVIC_SetPriority(EINT3_IRQn, 3);
}

void Keypad_Init(void){
  keypad_pins_init();
  keypad_irq_init();
}

void Keypad_GetCharNonBlock(char *ch){
    if (last_key != 0) {
        *ch = last_key;
        last_key = 0;
    } else {
        *ch = 0;
    }
}

void EINT3_IRQHandler(void){
  uint32_t col_mask = COL_BITS[0] | COL_BITS[1] | COL_BITS[2] | COL_BITS[3];
  if (GPIO_GetIntStatus(COL_PORT, 4, 0) || GPIO_GetIntStatus(COL_PORT, 5, 0) ||
      GPIO_GetIntStatus(COL_PORT, 6, 0) || GPIO_GetIntStatus(COL_PORT, 7, 0))
  {
      NVIC_DisableIRQ(EINT3_IRQn);
      GPIO_ClearInt(COL_PORT, col_mask);

      delay_ms(debounce_ms);

      int row,col;
      if (scan_key_position(&row,&col)){
        last_key = keymap[row][col];
        while(read_col_index() != -1); // Esperar a que se suelte la tecla
      }

      delay_ms(debounce_ms);
      GPIO_ClearInt(COL_PORT, col_mask);
      NVIC_EnableIRQ(EINT3_IRQn);
  }
}
