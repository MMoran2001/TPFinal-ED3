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

#define ROW_PORT 2
#define COL_PORT 2
//#define TIMEOUT 3124
#define ADCRATE 200000		// A mayor ADCRATE mayor fidelidad de sonido.
#define LISTSIZE 2000		// Cantidad de muestras
#define DACRATE (25000000)/ADCRATE

static const uint32_t ROW_BITS[] = { (1<<0), (1<<1), (1<<2), (1<<3)};
static const uint32_t COL_BITS[] = { (1<<4), (1<<5), (1<<6), (1<<7)};

static const char keymap[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

static volatile char last_key = 0;
static volatile uint32_t debounce_ms = 10;
volatile int newkey = 0;
volatile int recording = 0;
volatile int recording_frecuency=0;
volatile int playing = 0;
volatile int sound_index = 0;
volatile int row, col;
volatile uint32_t ticks_ms = 0;
volatile uint16_t DACRATE1 = DACRATE;
volatile uint16_t DACRATE2 = DACRATE;
volatile uint16_t DACRATE3 = DACRATE;
volatile uint16_t adc_buffer[LISTSIZE] = {0};
volatile int index = 0;
volatile int buffer_full = 0;
uint32_t dmaON = 0;

#define SEQUENCE_BUFFER_SIZE 5
char sequence_buffer[SEQUENCE_BUFFER_SIZE] = {0};
int sequence_index = 0;
int sequence_recording = 0;

/* Esta lista guarda el sonido grabado por el microfono. */
volatile uint32_t muestras_adc[LISTSIZE] = {0};
volatile uint32_t sound1_list[LISTSIZE] = {0};		// Lista de sonido 1
volatile uint32_t sound2_list[LISTSIZE] = {0};		// Lista de sonido 2
volatile uint32_t sound3_list[LISTSIZE] = {0};		// Lista de sonido 3

 // Puntero a la direccion de memoria del buffer de transferencia
 //static uint16_t * const adc_dac_buffer = (uint16_t *)ADDRESS;

// --- Prototipos para el teclado ---
void rows_all_high(void);
void rows_all_down (void);
int read_col_index(void);
int scan_key_position(int *r_out, int *c_out);
void keypad_pins_init(void);
void keypad_irq_init(void);
void delay_ms(uint32_t ms);
void Keypad_GetCharNonBlock(char *ch);
void Keypad_Init(void);
void Keypad_clear(char *ch);


// --- Prototipos para las configuraciones ---
void configPin(void);
void configADC(void);
void configDAC(void);
void configTimer(void);
void configDMA(void);
void configUART(void);
void configDmaDac(volatile uint32_t listtoDAC[]);
//void configDmaAdc(void);

// --- Prototipos para las funciones auxiliares ---
void Keypad_clear(char *ch);
void startPlayback(volatile uint32_t *sound_list);
void convertBufferAdcToDac(uint16_t *buf, uint32_t n, int index);
void ledOff();
void ledRojo();
void ledVerde();
void ledAzul();
void ledCeleste();
void ledVioleta();
void ledBlanco();
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

// --- Handlers de interrupción ---
void SysTick_Handler(void);
void EINT3_IRQHandler(void);
void ADC_IRQHandler(void);
void DMA_IRQHandler(void);



int main (void){
   Keypad_Init();
   configPin();
   configADC();
   configDAC();
   //configTimer();
   configDMA();


   ledOff();
   while (1){
       char ch;
       Keypad_GetCharNonBlock(&ch);
        // acá procesás la tecla (enviar por UART, etc.)
        switch(ch){
            case '1':
                  if (!recording) { // Reproducir solo si no se está grabando
                    DAC_SetDMATimeOut(LPC_DAC, DACRATE1);
                    startPlayback(sound1_list);
                  }
                  ledRojo();
                  //NVIC_DisableIRQ(DMA_IRQn);
                break;
            case '2':
                  if (!recording) { // Reproducir solo si no se está grabando
                    DAC_SetDMATimeOut(LPC_DAC, DACRATE2);
                    startPlayback(sound2_list);

                  }
                  ledVerde();
                  //NVIC_DisableIRQ(DMA_IRQn);
                break;
            case '3':
                  if (!recording) { // Reproducir solo si no se está grabando
                    DAC_SetDMATimeOut(LPC_DAC, DACRATE3);
                    startPlayback(sound3_list);

                  }
                  ledAzul();
                  //NVIC_DisableIRQ(DMA_IRQn);
                break;
            case '5':
                ledBlanco();
                playing =1;
                while(playing){
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);
                  for(int i = 0 ; i<sequence_index ; i++){
                	  if((sequence_buffer[i]) == 1){
                      DAC_SetDMATimeOut(LPC_DAC, DACRATE1);
                		  startPlayback(sound1_list);
                	  }
                	  else if((sequence_buffer[i]) == 2){
                      DAC_SetDMATimeOut(LPC_DAC, DACRATE2);
                		  startPlayback(sound2_list);
                	  }
                	  else if((sequence_buffer[i]) == 3){
                      DAC_SetDMATimeOut(LPC_DAC, DACRATE3);
                		  startPlayback(sound3_list);
                	  }
                	  delay_ms(500);
                  }
                  if(ch == '5'){
                    playing =0;
                    ledOff();
                    GPDMA_ChannelCmd(1, DISABLE); // Detengo la reproduccion
                  }
                }
                break;
            case '7':
                sequence_recording = 1;
                ledVioleta();
                sequence_index = 0; // Resetea el índice al iniciar
                for(int i=0; i<SEQUENCE_BUFFER_SIZE; i++) sequence_buffer[i] = 0; // Limpia el buffer
                while (sequence_recording) { // Mientras se mantenga presionada la tecla y este grabando
                  //TO DO: Espera a que se suelte la tecla, porque read_col_index me bloquea el programa en scan_key_position
                  //TO DO: Ver como hacer para que el programa tome dos teclas
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);

                  if(ch == '1'){
                    sequence_buffer[sequence_index] = ch;
                    sequence_index = (sequence_index + 1) % SEQUENCE_BUFFER_SIZE;
                    Keypad_clear(&ch); // Limpia la tecla para que no se procese de nuevo
                    ledRojo();
                  }
                  else if(ch == '2'){
                    sequence_buffer[sequence_index] = ch;
                    sequence_index = (sequence_index + 1) % SEQUENCE_BUFFER_SIZE;
                    ledVerde();
                  }
                  else if(ch == '3'){
                    sequence_buffer[sequence_index] = ch;
                    sequence_index = (sequence_index + 1) % SEQUENCE_BUFFER_SIZE;
                    ledAzul();
                  }else if (ch == '7'){
                    sequence_recording = 0;
                    ledOff();
                  }
                }
                break;
            //------------------------ BOTÓN PARA CAMBIAR FRECUENCIA ------------------------
            case '8':
              recording_frecuency = 1;

              ledVioleta();

                while (recording_frecuency) {
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);

                  if(ch == '1'){
                    sound_index = 4; // Si toco la tecla '1' grabo en la lista de sonido 1
                    //configDmaAdc();
                    ledRojo();
                    NVIC_EnableIRQ(ADC_IRQn);
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE); // activo el canal del potenciometro
                    ADC_BurstCmd(LPC_ADC, ENABLE);

                  }
                  else if(ch == '2'){
                    sound_index = 5; // Si toco la tecla '2' grabo en la lista de sonido 2
                    //configDmaAdc();
                    ledVerde();
                    NVIC_EnableIRQ(ADC_IRQn);
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE); // activo el canal del potenciometro
                    ADC_BurstCmd(LPC_ADC, ENABLE);
                  }
                  else if(ch == '3'){
                    sound_index = 6; // Si toco la tecla '3' grabo en la lista de sonido 3
                    //configDmaAdc();
                    ledAzul();
                    NVIC_EnableIRQ(ADC_IRQn);
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE); // activo el canal del potenciometro
                    ADC_BurstCmd(LPC_ADC, ENABLE);
                  }else if (ch == '8'){
                    recording_frecuency = 0;
                    ledOff();
                    NVIC_DisableIRQ(ADC_IRQn);
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, DISABLE); // Desactiva el canal del potenciometro
                    ADC_BurstCmd(LPC_ADC, DISABLE);

                    //GPDMA_ChannelCmd(1, DISABLE); // Detengo la grabacion
//                    moveListDAC(sound1_list);
//                    moveListDAC(sound2_list);
//                    moveListDAC(sound3_list);
                  }
                  }
                  NVIC_DisableIRQ(ADC_IRQn);
                  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, DISABLE); // Desactiva el canal del potenciometro
                  ADC_BurstCmd(LPC_ADC, DISABLE);
                break;
            //------------------------ BOTÓN PARA GRABAR SONIDOS ------------------------
            case '9':
              recording = 1;
              sound_index = 0;

              ledCeleste();

                while (recording) {
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);

                  if(ch == '1'){
                    sound_index = 1; // Si toco la tecla '1' grabo en la lista de sonido 1
                    ledRojo();
                    buffer_full = 0;
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
                    ADC_BurstCmd(LPC_ADC, ENABLE); // activo el canal del microfono en BURST
                    NVIC_EnableIRQ(ADC_IRQn);

                    //configDmaAdc();
                  }
                  else if(ch == '2'){
                    sound_index = 2; // Si toco la tecla '2' grabo en la lista de sonido 2
                    ledVerde();
                    buffer_full = 0;
                    //configDmaAdc();
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
                    ADC_BurstCmd(LPC_ADC, ENABLE); // activo el canal del microfono en BURST
                    NVIC_EnableIRQ(ADC_IRQn);


                  }
                  else if(ch == '3'){
                    sound_index = 3; // Si toco la tecla '3' grabo en la lista de sonido 3
                    ledAzul();
                    buffer_full = 0;
                    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
                    ADC_BurstCmd(LPC_ADC, ENABLE); // activo el canal del microfono en BURST
                    NVIC_EnableIRQ(ADC_IRQn);

                    //configDmaAdc();
                  }
                  else if(ch == '9'){
                    recording = 0;
                    ledOff();

                  }
                }
                sound_index = 0;
                buffer_full = 0;
                NVIC_DisableIRQ(ADC_IRQn);
                GPDMA_ChannelCmd(0, DISABLE); // Detengo la grabacion
                ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE); // Desactiva el canal del microfono
                ADC_BurstCmd(LPC_ADC, DISABLE);
                //TIM_Cmd(LPC_TIM0, DISABLE);
                break;

            default:
                 // GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                  //GPIO_SetValue(3, (1<<25)); // Apaga led verde
                  //GPIO_SetValue(3, (1<<26)); // Apaga led azul
                break;
      }
    }
    return 0;
}

/*



TECLADO



*/

// --- utilidades ---
void rows_all_high(void){
  // Configuradas como salida, subir todas a '1'
  for (int i=0;i<4;i++) GPIO_ClearValue(ROW_PORT, ROW_BITS[i]);
}
void rows_all_down (void){
	for (int i=0;i<4;i++) GPIO_SetValue(ROW_PORT, ROW_BITS[i]);
}

int read_col_index(void){
  uint32_t pins = GPIO_ReadValue(COL_PORT);
  for (int c=0;c<4;c++){
    if ((pins & COL_BITS[c]) == 0) return c; // activo en 0 (pull-up)
  }
  return -1;
}

int scan_key_position(int *r_out, int *c_out){
  // Barrido: una fila por vez en 0, resto en 1
  for (int r=0;r<4;r++){
    rows_all_down();
    GPIO_ClearValue(ROW_PORT, ROW_BITS[r]); // fila activa = 0
    for (volatile int d=0; d<200; d++) __NOP(); // breve settle

    int c = read_col_index();
    if (c >= 0){
      *r_out = r;
      *c_out = c;
      rows_all_high();
      return 1;
    }
    GPIO_SetValue(ROW_PORT, ROW_BITS[r]);
  }
  rows_all_high();
  return 0;
}

// --- inicialización de pines (PINSEL+PINMODE=GPIO con pull-up) ---
void keypad_pins_init(void){
  PINSEL_CFG_Type cfg;
  cfg.Funcnum = 0; // GPIO
  cfg.OpenDrain = 0;
  cfg.Portnum = ROW_PORT;

  // Filas
  for (int i=0;i<4;i++){
    cfg.Pinnum = 0 + i;
    cfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&cfg);
    GPIO_SetDir(ROW_PORT, ROW_BITS[i], 1); // salida sin pull-up o pull-down
  }
  // Columnas
  for (int i=0;i<4;i++){
    cfg.Pinnum = 0 + i;
    cfg.Pinmode = 0; // 00 = pull-up
    PINSEL_ConfigPin(&cfg);
    GPIO_SetDir(COL_PORT, COL_BITS[i], 0); // entrada con pull-up
  }

  rows_all_high();
}

// --- interrupciones GPIO (EINT3) en columnas por flanco descendente ---
void keypad_irq_init(void){
  // Habilitar interrupción por flanco descendente en columnas
  uint32_t col_mask = 0;
  for (int i=0;i<4;i++) col_mask |= COL_BITS[i];

  GPIO_IntCmd(COL_PORT, col_mask, 1); // 1 = falling edge
  NVIC_EnableIRQ(EINT3_IRQn);         // IRQ de GPIO P0/P2
  NVIC_SetPriority(EINT3_IRQn, 3);
}



void delay_ms(uint32_t ms){
  while (ticks_ms < ms) {} // a ver si el antirebote anda bien asi
  ticks_ms = 0;
}

// --- API mínima ---
void Keypad_Init(void){
  keypad_pins_init();
  SysTick_Config(100000); // Systick a 1ms
  NVIC_SetPriority(SysTick_IRQn, 2); // Asignar mayor prioridad a SysTick
  keypad_irq_init();
}

void Keypad_GetCharNonBlock(char *ch){
    *ch = last_key;
    last_key = 0;
}



/*
* ---------------------------------------------------------------
* ---------------------------------------------------------------
* ---------------------------------------------------------------
*                   CONFIGURACIONES DEL LPC
* ---------------------------------------------------------------
* ---------------------------------------------------------------
* ---------------------------------------------------------------
*/

// --- configuración de pines ---
void configPin(void){
  PINSEL_CFG_Type PinCfg = {0};

  // ADC - Microfono
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 23;
  PinCfg.Funcnum = 1;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PINSEL_ConfigPin(&PinCfg); // AD0.0

  // DAC
  PinCfg.Funcnum = 2;
  PinCfg.Pinnum = 26;
  PINSEL_ConfigPin(&PinCfg); // AOUT

  // UART2
  // PinCfg.Portnum = 0;
  // PinCfg.Pinnum = 10;
  // PinCfg.Funcnum = 1;
  // PINSEL_ConfigPin(&PinCfg);	// TXD2
  // PinCfg.Pinnum 		= 11;
  // PINSEL_ConfigPin(&PinCfg);	// RXD2

  //Led rojo
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 22;
  PinCfg.Funcnum = 0;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PINSEL_ConfigPin(&PinCfg);

  // Led verde
  PinCfg.Portnum = 3;
  PinCfg.Pinnum = 25;
  PINSEL_ConfigPin(&PinCfg);

  // Led azul
  PinCfg.Pinnum = 26;
  PinCfg.Portnum = 3;
  PINSEL_ConfigPin(&PinCfg);

  GPIO_SetDir(3, (1<<25), 1); // Led verde y azul como salida
  GPIO_SetDir(3, (1<<26), 1);
  GPIO_SetDir(0, (1<<22), 1); // Led rojo como salida

}

// --- configuración del ADC ---
/* El ADC se utiliza en modo burst para tener el maximo de resolucion.
	 * La interrupcion se activa en el main y comienza apagada ya que se prende con el pulsador de "write".
	 */
void configADC(void){
  ADC_Init(LPC_ADC, ADCRATE);
  ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE); // El canal del microfono se configura apagado hasta que empiece la grabacion
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, DISABLE); // El canal del potenciometro empieza desactivado
  ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE); // Interrupcion del microfono
  ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, ENABLE); // Innterrupcion del potenciometro
  ADC_BurstCmd(LPC_ADC, DISABLE); // Modo burst arranca deshabilitado
}

// --- configuración del DAC ---
void configDAC(void){
  DAC_CONVERTER_CFG_Type dacCfg;
  dacCfg.DBLBUF_ENA = 0; // Deshabilitar double buffer
  dacCfg.CNT_ENA = 1;    // Habilitar contador de tiempo
  dacCfg.DMA_ENA = 1;    // Habilitar acceso a DMA

  DAC_Init(LPC_DAC);
  DAC_SetBias(LPC_DAC, 0);
  DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
  DAC_SetDMATimeOut(LPC_DAC, DACRATE); // Tiempo de espera DMA

}

// --- configuración de DMA ---
void configDMA(void){
  GPDMA_Init();
}

//------------------ DMA: RAM -> DAC (canal 1, M2P) ----------------
void configDmaDac(__IO uint32_t listtoDAC[]) {
    GPDMA_Channel_CFG_Type cfg;
    GPDMA_LLI_Type lli;

    lli.SrcAddr= (uint32_t)listtoDAC;
    lli.DstAddr= (uint32_t)&(LPC_DAC->DACR);
    lli.NextLLI= (uint32_t)&lli;
    lli.Control= LISTSIZE
    			| (4<<12) //source width 32 bit
    			| (4<<15) //
    			| (1<<26) //source increment
    			| (2<<18)
    			| (1<<22)
				&~(1<<27)
    			;
    //NVIC_DisableIRQ(DMA_IRQn);
    GPDMA_ChannelCmd(1, DISABLE);

    cfg.ChannelNum    = 1;
    cfg.SrcMemAddr    = (uint32_t)listtoDAC;   // fuente = lista de sonido seleccionada
    cfg.DstMemAddr    = 0;
    cfg.TransferSize  = LISTSIZE;
    cfg.TransferWidth = GPDMA_WIDTH_WORD;
    cfg.TransferType  = GPDMA_TRANSFERTYPE_M2P;
    cfg.SrcConn       = 0;
    cfg.DstConn       = GPDMA_CONN_DAC;
    cfg.DMALLI        = (uint32_t)&lli;

    GPDMA_Setup(&cfg);
    GPDMA_ChannelCmd(1, ENABLE);
    //NVIC_EnableIRQ(DMA_IRQn);
}

// void configTimer(void){
//     TIM_TIMERCFG_Type TIM_ConfigStruct;
//     TIM_MATCHCFG_Type TIM_MatchConfigStruct;

//     // Configuración del Timer
//     TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
//     TIM_ConfigStruct.PrescaleValue = 1; // El prescaler cuenta en microsegundos

//     // Configuración del Match
//     TIM_MatchConfigStruct.MatchChannel = 1;
//     TIM_MatchConfigStruct.IntOnMatch = DISABLE;
//     TIM_MatchConfigStruct.ResetOnMatch = ENABLE;
//     TIM_MatchConfigStruct.StopOnMatch = DISABLE;
//     TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
//     TIM_MatchConfigStruct.MatchValue = 1000000 / ADCRATE; //

//     TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIM_ConfigStruct);
//     TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);

//     // El timer se habilita solo cuando se empieza a grabar.
//     TIM_Cmd(LPC_TIM0, DISABLE);

//     //NVIC_SetPriority(TIMER0_IRQn, 4); // Prioridad intermedia
//     //NVIC_EnableIRQ(TIMER0_IRQn);
// }

/*
* ---------------------------------------------------------------
* ---------------------------------------------------------------
* ---------------------------------------------------------------
*                   HANDLERS DE INTERRUPCION
* ---------------------------------------------------------------
* ---------------------------------------------------------------
* ---------------------------------------------------------------
*/

// --- Handler de Systick ---

void SysTick_Handler(void){ ticks_ms++; }

// --- Handler de interrupcion por GPIO ---
// --- ISR de GPIO: localizar tecla, antirrebote, limpiar flags ---
void EINT3_IRQHandler(void){
  // Deshabilito nuevas IRQ de columnas durante el manejo
  uint32_t col_mask = 0;
  for (int i=0;i<4;i++) col_mask |= COL_BITS[i];
  NVIC_DisableIRQ(EINT3_IRQn); // Deshabilitar Interrupciones de GPIO

  // Limpiar flags latentes (falling)
  GPIO_ClearInt(COL_PORT, col_mask);

  // Antirrebote: tomar estado estable luego de breve espera
  delay_ms(debounce_ms);

  int row,col;
  if (scan_key_position(&row,&col)){
    last_key = keymap[row][col];
    newkey = 1;
    // Esperar liberación
    //while (read_col_index() >= 0) { /* tecla aun presionada */ }
    //delay_ms(debounce_ms);
  }

  // Limpiar y re-habilitar
  GPIO_ClearInt(COL_PORT, col_mask);
  NVIC_EnableIRQ(EINT3_IRQn); // 0 = falling edge (enable)
}

void ADC_IRQHandler(){

	static uint32_t ADCVAL 		= 0;
    static uint32_t ADCVALMAP	= 0;
	uint32_t stat = ADC_ChannelGetStatus(LPC_ADC, 0, 0);
	//uint32_t stat2 = ADC_ChannelGetStatus(LPC_ADC, 1, 0);
	   //if(!stat){
	if(recording){
		uint16_t raw = ADC_ChannelGetData(LPC_ADC, 0);
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
				   	    	NVIC_DisableIRQ(ADC_IRQn);
				   	    	ADC_BurstCmd(LPC_ADC,DISABLE);
				   	    	for(uint32_t i = 0; i<LISTSIZE; i++){
				                     switch (sound_index){
				                     case 1:
				                       sound1_list[i]=(muestras_adc[i]<< 6);
				                       break;
				                     case 2:
				                       sound2_list[i]=(muestras_adc[i]<< 6);
				                       break;
				                     case 3:
				                       sound3_list[i]=(muestras_adc[i]<< 6);
				                       break;
				                     default:
				                       break;
				                     }

				   	    		    	}

				                   ledCeleste();
				   	       }
	   	}else if(recording_frecuency){
	   		ADCVAL 		= ((LPC_ADC->ADDR1)>>6) & 0x3FF;
	   		ADCVALMAP	= map(ADCVAL, 0, 1024, 5000, 20000);
	   		NVIC_DisableIRQ(ADC_IRQn);
	        ADC_BurstCmd(LPC_ADC,DISABLE);
	   		switch (sound_index){
	   			case 4:
	   				DACRATE1 = ADCVALMAP;
	   				break;
	   			case 5:
	   				DACRATE2 = ADCVALMAP;
	   				break;
	   			case 6:
	   				DACRATE3 = ADCVALMAP;
	   				break;
	   			default:
	   			break;
	   		 }
	   	     ledVioleta();


}
}
// --- Handler de interrupción de DMA ---
// void DMA_IRQHandler(void) {

//         if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) { // Verificar si el dma termino de tomar muestras del adc
//             GPDMA_ChannelCmd(0, DISABLE); // Deshabilitar el canal DMA
//             GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0); // Limpiar el flag de interrupción

//             ADC_BurstCmd(LPC_ADC, DISABLE); // Deshabilitar el modo burst del ADC

//             convertBufferAdcToDac(adc_dac_buffer, LISTSIZE, sound_index);

//             configDmaAdc(); // Reconfigurar el DMA para la próxima grabación

//             if(recording){
//               ADC_BurstCmd(LPC_ADC, ENABLE); // Habilitar el modo burst del ADC
//             }

//         }

//         if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 1)) { // Verificar si el dma termino de sacar muestras por el dac
//             GPDMA_ChannelCmd(1, DISABLE); // Deshabilitar el canal DMA
//             GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 1); // Limpiar el flag de interrupción
//             NVIC_DisableIRQ(DMA_IRQn); // Deshabilitar la interrupción del DMA hasta la próxima reproducción
//         }
// }


// void TIMER0_IRQHandler(void){
//     TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT); // Limpiar la interrupción del Match 1
//     if(ADC_ChannelGetStatus(LPC_ADC, 0, 1)){ // Verifico si se levanto el flag de DONE del canal del microfono (0)
// 			/* Comenzamos a grabar un audio y guardarlo en el array que corresponda. */
//       uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0); // Leo el valor del microfono
//      if(sound_index == 1 && samples_count <= LISTSIZE){
//        sound1_list[samples_count++] = (adc_value & 0xFFC0); // Usar los 10 bits más significativos (alineados para DAC)

//      }
//      else if(sound_index == 2 && samples_count <= LISTSIZE){
//        sound2_list[samples_count++] = (adc_value & 0xFFC0);

//      }
//      else if(sound_index == 3 && samples_count <= LISTSIZE){
//        sound3_list[samples_count++] = (adc_value & 0xFFC0);

//      }

//      if(samples_count >= LISTSIZE){
//        samples_count = 0;
// 		  }
//    }
//   //  if(ADC_ChannelGetStatus(LPC_ADC, 1, 1)){ // Verifico si se levanto el flag de DONE del canal del potenciometro (1)
//   //    uint32_t pot_value = ADC_ChannelGetData(LPC_ADC, 1); // Leo el valor del potenciometro
//   //    uint32_t timeout = map(pot_value, 0, 1024, 5000, 20000); // Escalo el valor del potenciometro para usarlo como timeout (Me base en el otro codigo)
//   //    DAC_SetDMATimeOut(LPC_DAC, timeout); // Actualizo el timeout del DAC
//   //  }
// }

/*
* ---------------------------------------------------------------
* ---------------------------------------------------------------
* ---------------------------------------------------------------
*                   FUNCIONES AUXILIARES
* ---------------------------------------------------------------
* ---------------------------------------------------------------
* ---------------------------------------------------------------
*/

void Keypad_clear(char *ch){
	*ch = 0;
}

void startPlayback(volatile uint32_t *sound_list) {

    if(sound_list == NULL) return; // Verificar que la lista no sea NULL
    configDmaDac(sound_list);
}

// Convierte cada muestra del buffer desde formato ADC (ADGDR/ADDRx)
// a formato DACR (VALUE en bits 15:6).
// void convertBufferAdcToDac(uint16_t *buf, uint32_t n, int index) {
//     for (uint32_t i = 0; i < n; i++) {

//         switch (index) {
//             case 1:
//                 sound1_list[i] = dac_word;
//                 break;
//             case 2:
//                 sound2_list[i] = dac_word;
//                 break;
//             case 3:
//                 sound3_list[i] = dac_word;
//                 break;
//             case 4:
//                 DACRATE1 = dac_word;
//                 break;
//             case 5:
//                 DACRATE2 = dac_word;
//                 break;
//             case 6:
//                 DACRATE3 = dac_word;
//                 break;
//         }
//     }
// }
void ledOff(){
  GPIO_SetValue(0, (1<<22));
  GPIO_SetValue(3, (1<<25));
  GPIO_SetValue(3, (1<<26));
}
void ledRojo(){
  GPIO_SetValue(3, (1<<25));
  GPIO_SetValue(3, (1<<26));
  GPIO_ClearValue(0, (1<<22));
}
void ledVerde(){
  GPIO_SetValue(0, (1<<22));
  GPIO_SetValue(3, (1<<26));
  GPIO_ClearValue(3, (1<<25));
}
void ledAzul(){
  GPIO_SetValue(3, (1<<25));
  GPIO_SetValue(0, (1<<22));
  GPIO_ClearValue(3, (1<<26));
}
void ledCeleste(){
  GPIO_SetValue(0, (1<<22));
  GPIO_ClearValue(3, (1<<26));
  GPIO_ClearValue(3, (1<<25));
}
void ledVioleta(){
	GPIO_ClearValue(3, (1<<26));
	GPIO_SetValue(3, (1<<25));
	GPIO_ClearValue(0, (1<<22));
}
void ledBlanco(){
	GPIO_ClearValue(3, (1<<26));
	GPIO_ClearValue(3, (1<<25));
	GPIO_ClearValue(0, (1<<22));
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	/* Convierte el valor recibido a un valor correspondiente dentro de una escala MIN-MAX dada. */
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

