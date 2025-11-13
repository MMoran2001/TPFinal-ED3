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
#define TIMEOUT 3124	// Arranca por default en un valor, pero el potenciometro lo varia durante la ejecucion.
#define ADCRATE 50000		// A mayor ADCRATE mayor fidelidad de sonido.
#define LISTSIZE 2500		// No superar los 15k muestras por que se llena la SRAM 32kB.
#define transferSize 4096	// Tamaño de transferencia del DMA


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
volatile int playing = 0;
volatile int sound_index = 0;
volatile int row, col;
volatile uint32_t ticks_ms = 0;

#define SEQUENCE_BUFFER_SIZE 5
char sequence_buffer[SEQUENCE_BUFFER_SIZE] = {0};
int sequence_index = 0;
int sequence_recording = 0;

/* Esta lista guarda el sonido grabado por el microfono. */
 uint16_t listTransfer[LISTSIZE] = {0};
 uint32_t samples_count = 0;		// Contador de muestras para la lista del ADC.
 uint32_t sound1_list[LISTSIZE] = {0};		// Lista de sonido 1
 uint32_t sound2_list[LISTSIZE] = {0};		// Lista de sonido 2
 uint32_t sound3_list[LISTSIZE] = {0};		// Lista de sonido 3

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

// --- Prototipos para las funciones auxiliares ---
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
void Keypad_clear(char *ch);
void moveListDAC(uint32_t *list);
void startPlayback(uint32_t* sound_list);


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
   configTimer();
   configDMA();


   GPIO_SetValue(0, (1<<22)); // Apaga led rojo
   GPIO_SetValue(3, (1<<25)); // Apaga led verde
   GPIO_SetValue(3, (1<<26)); // Apaga led azul
   while (1){
       char ch;
       Keypad_GetCharNonBlock(&ch);
        // acá procesás la tecla (enviar por UART, etc.)
        switch(ch){
            case '1':
                  if (!recording) { // Reproducir solo si no se está grabando
                    startPlayback(sound1_list);
                  }
                  GPIO_SetValue(3, (1<<25)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<26)); // Apaga led azul
                  GPIO_ClearValue(0, (1<<22)); // Enciende led rojo
                  //NVIC_DisableIRQ(DMA_IRQn);
                break;
            case '2':
                  if (!recording) { // Reproducir solo si no se está grabando
                    startPlayback(sound2_list);

                  }
                  GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<26)); // Apaga led azul
                  GPIO_ClearValue(3, (1<<25)); // Enciende led verde
                  //NVIC_DisableIRQ(DMA_IRQn);
                break;
            case '3':
                  if (!recording) { // Reproducir solo si no se está grabando
                    startPlayback(sound3_list);

                  }
                  GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<25)); // Apaga led verde
                  GPIO_ClearValue(3, (1<<26)); // Enciende led azul
                  //NVIC_DisableIRQ(DMA_IRQn);
                break;
            case '5':
                GPIO_ClearValue(3, (1<<26));
                GPIO_SetValue(3, (1<<25));
                GPIO_ClearValue(0, (1<<22));
                playing =1;
                while(playing){
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);
                  for(int i = 0 ; i<sequence_index ; i++){
                	  if((sequence_buffer[i]) == 1){
                		  startPlayback(sound1_list);
                	  }
                	  else if((sequence_buffer[i]) == 2){
                		  startPlayback(sound2_list);
                	  }
                	  else if((sequence_buffer[i]) == 3){
                		  startPlayback(sound3_list);
                	  }
                	  delay_ms(500);
                  }
                  if(ch == '5'){
                    playing =0;
                    GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul
                    GPDMA_ChannelCmd(0, DISABLE); // Detengo la reproduccion
                  }
                }
                break;
            case '7':
                sequence_recording = 1;
                GPIO_ClearValue(3, (1<<26));
                GPIO_SetValue(3, (1<<25));
                GPIO_ClearValue(0, (1<<22));
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
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul
                    GPIO_ClearValue(0, (1<<22));
                  }
                  else if(ch == '2'){
                    sequence_buffer[sequence_index] = ch;
                    sequence_index = (sequence_index + 1) % SEQUENCE_BUFFER_SIZE;
                    GPIO_SetValue(3, (1<<26)); // Apaga led verde
                    GPIO_SetValue(0, (1<<22)); // Apaga led azul
                    GPIO_ClearValue(3, (1<<25));
                  }
                  else if(ch == '3'){
                    sequence_buffer[sequence_index] = ch;
                    sequence_index = (sequence_index + 1) % SEQUENCE_BUFFER_SIZE;
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(0, (1<<22)); // Apaga led azul
                    GPIO_ClearValue(3, (1<<26));
                  }else if (ch == '7'){
                    sequence_recording = 0;
                    GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul
                  }
                }
                break;
            case '8':
              recording = 1;
              GPIO_ClearValue(3, (1<<26));
              GPIO_ClearValue(3, (1<<25));
              GPIO_ClearValue(0, (1<<22));
              //ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE); // activo el canal del microfono
              //NVIC_EnableIRQ(ADC_IRQn);
                while (recording) { // Mientras se mantenga presionada la tecla y este grabando
                  //TO DO: Espera a que se suelte la tecla, porque read_col_index me bloquea el programa en scan_key_position
                  //TO DO: Ver como hacer para que el programa tome dos teclas
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);

                  if(ch == '1'){
                    sound_index = 4; // Si toco la tecla '1' grabo en la lista de sonido 1
                    //NVIC_EnableIRQ(ADC_IRQn); // Habilito la interrupcion del ADC para empezar a grabar
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul
                    GPIO_ClearValue(0, (1<<22));

                  }
                  else if(ch == '2'){
                    sound_index = 5; // Si toco la tecla '2' grabo en la lista de sonido 2
                    //NVIC_EnableIRQ(ADC_IRQn);
                    GPIO_SetValue(3, (1<<26)); // Apaga led verde
                    GPIO_SetValue(0, (1<<22)); // Apaga led azul
                    GPIO_ClearValue(3, (1<<25));
                  }
                  else if(ch == '3'){
                    sound_index = 6; // Si toco la tecla '3' grabo en la lista de sonido 3
                    //NVIC_EnableIRQ(ADC_IRQn);
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(0, (1<<22)); // Apaga led azul
                    GPIO_ClearValue(3, (1<<26));
                  }else if (ch == '8'){
                    recording = 0;
                    GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul
                    NVIC_DisableIRQ(ADC_IRQn); // Deshabilito la interrupcion del ADC para dejar de grabar
                    //ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, DISABLE); // Desactiva el canal del microfono
//                    moveListDAC(sound1_list);
//                    moveListDAC(sound2_list);
//                    moveListDAC(sound3_list);
                  }
                  }
                break;
            case '9': // Boton para grabar, despues podemos cambiar el valor del case
              recording = 1;
              sound_index = 0;
              samples_count = 0;
              GPIO_SetValue(0, (1<<22)); // Apaga led rojo
              GPIO_SetValue(3, (1<<25)); // Apaga led verde
              GPIO_SetValue(3, (1<<26)); // Apaga led azul
              GPIO_ClearValue(3, (1<<26));
              GPIO_ClearValue(3, (1<<25));
                while (recording) { // Mientras se mantenga presionada la tecla y este grabando
                  //TO DO: Espera a que se suelte la tecla, porque read_col_index me bloquea el programa en scan_key_position
                  //TO DO: Ver como hacer para que el programa tome dos teclas
                  Keypad_clear(&ch);
                  Keypad_GetCharNonBlock(&ch);

                  if(ch == '1' || ch == '2' || ch == '3'){
                    if (sound_index == 0) { // Si es la primera vez que se presiona una tecla
                        if (ch == '1') sound_index = 1;
                        if (ch == '2') sound_index = 2;
                        if (ch == '3') sound_index = 3;

                        // Iniciar la grabación AHORA
                        ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
                        NVIC_EnableIRQ(ADC_IRQn);
                        TIM_Cmd(LPC_TIM0, ENABLE);
                    }
                  }
                  if(ch == '1'){
                    sound_index = 1; // Si toco la tecla '1' grabo en la lista de sonido 1
                    //NVIC_EnableIRQ(ADC_IRQn); // Habilito la interrupcion del ADC para empezar a grabar
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul
                    GPIO_ClearValue(0, (1<<22));
                  }
                  else if(ch == '2'){
                    sound_index = 2; // Si toco la tecla '2' grabo en la lista de sonido 2
                    //NVIC_EnableIRQ(ADC_IRQn);
                    GPIO_SetValue(3, (1<<26)); // Apaga led verde
                    GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                    GPIO_ClearValue(3, (1<<25));
                  }
                  else if(ch == '3'){
                    sound_index = 3; // Si toco la tecla '3' grabo en la lista de sonido 3
                    //NVIC_EnableIRQ(ADC_IRQn);
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(0, (1<<22)); // Apaga led azul
                    GPIO_ClearValue(3, (1<<26));
                  }
                  else if(ch == '9'){
                    recording = 0;
                    samples_count = 0;
                    GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                    GPIO_SetValue(3, (1<<25)); // Apaga led verde
                    GPIO_SetValue(3, (1<<26)); // Apaga led azul


                  }
                }// Recording va a cambiar a 0 en el handler del ADC cuando se llene la lista
                sound_index = 0;
                samples_count = 0;
                NVIC_DisableIRQ(ADC_IRQn); // Deshabilito la interrupcion del ADC para dejar de grabar
                ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE); // Desactiva el canal del microfono
                TIM_Cmd(LPC_TIM0, DISABLE);
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
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 10;
  PinCfg.Funcnum = 1;
  PINSEL_ConfigPin(&PinCfg);	// TXD2
  PinCfg.Pinnum 		= 11;
  PINSEL_ConfigPin(&PinCfg);	// RXD2

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
  ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE); // El canal del microfono se configura apagado hasta que empiece la grabacion
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, DISABLE); // El canal del potenciometro empieza activo
  ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE); // Interrupcion por canal 0 (microfono)
  ADC_BurstCmd(LPC_ADC, DISABLE); // Modo burst deshabilitado

  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 4); // Prioridad intermedia
}

// --- configuración del DAC ---
void configDAC(void){
  DAC_CONVERTER_CFG_Type dacCfg;
  dacCfg.DBLBUF_ENA = 0; // Deshabilitar double buffer
  dacCfg.CNT_ENA = 1;    // Habilitar contador de tiempo
  dacCfg.DMA_ENA = 1;    // Habilitar acceso a DMA

  uint32_t temp = (25000000)/ADCRATE;

  DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
  DAC_SetDMATimeOut(LPC_DAC, temp); // Tiempo de espera DMA
  DAC_Init(LPC_DAC); // Se inicializa el DAC con BIAS de 700uA

}

// --- configuración de DMA ---
// Podemos copiar lo de los chicos y modificarlo para lo nuestro
void configDMA(void){


  GPDMA_Init();
//  GPDMA_Channel_CFG_Type  DMAcfg;

//   // Configuración para Memoria a Periférico (DAC)
//   DMAcfg.ChannelNum = 0;
//   DMAcfg.TransferSize = LISTSIZE;
//   DMAcfg.TransferWidth = GPDMA_WIDTH_HALFWORD; // 32 bits
//   DMAcfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
//   DMAcfg.SrcMemAddr = (uint32_t)sound1_list; // Dirección de origen (se cambiará dinámicamente)
//   DMAcfg.DstMemAddr = 0; // No se usa para M2P
//   DMAcfg.SrcConn = 0; // No se usa para M2P
//   DMAcfg.DstConn = GPDMA_CONN_DAC;
//   DMAcfg.DMALLI = 0; // Sin listas enlazadas por ahora

//   GPDMA_Setup(&DMAcfg);

//   // Habilitar interrupción de DMA para saber cuándo termina la transferencia
//   NVIC_DisableIRQ(DMA_IRQn);
//   NVIC_SetPriority(DMA_IRQn, 5); // Prioridad alta para el DMA

}

void configTimer(void){
    TIM_TIMERCFG_Type TIM_ConfigStruct;
    TIM_MATCHCFG_Type TIM_MatchConfigStruct;

    // Configuración del Timer
    TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIM_ConfigStruct.PrescaleValue = 1; // El prescaler cuenta en microsegundos

    // Configuración del Match
    TIM_MatchConfigStruct.MatchChannel = 1;
    TIM_MatchConfigStruct.IntOnMatch = DISABLE;
    TIM_MatchConfigStruct.ResetOnMatch = ENABLE;
    TIM_MatchConfigStruct.StopOnMatch = DISABLE;
    TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
    TIM_MatchConfigStruct.MatchValue = 1000000 / ADCRATE; //

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIM_ConfigStruct);
    TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);

    // El timer se habilita solo cuando se empieza a grabar.
    TIM_Cmd(LPC_TIM0, DISABLE);

    //NVIC_SetPriority(TIMER0_IRQn, 4); // Prioridad intermedia
    //NVIC_EnableIRQ(TIMER0_IRQn);
}

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

// --- Handler de interrupción de DMA ---
void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0)) { // Verificar si la interrupción es del canal 0
        if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) { // Verificar si fue por finalización de transferencia (Terminal Count)
            GPDMA_ChannelCmd(0, DISABLE); // Deshabilitar el canal DMA
            GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0); // Limpiar el flag de interrupción
            NVIC_DisableIRQ(DMA_IRQn); // Deshabilitar la interrupción del DMA hasta la próxima reproducción
        }
    }
}

void ADC_IRQHandler(void) {
    if (ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)) {
        uint32_t adc_value = ADC_ChannelGetData(LPC_ADC, 0);

        // Guardar el valor alineado para el DAC
        if (recording && samples_count < LISTSIZE) {
            if (sound_index == 1) {
                sound1_list[samples_count] = (adc_value << 6);
            } else if (sound_index == 2) {
                sound2_list[samples_count] = (adc_value << 6);
            } else if (sound_index == 3) {
                sound3_list[samples_count] = (adc_value << 6);
            }
            samples_count++;
        }

        if (samples_count >= LISTSIZE) {
            TIM_Cmd(LPC_TIM0, DISABLE); // Detener el timer y las conversiones
            recording = 0; // O manejarlo en el bucle principal
            // Indicar que la grabación terminó (e.g., apagar LED)
            GPIO_SetValue(0, (1<<22));
            GPIO_SetValue(3, (1<<25));
            GPIO_SetValue(3, (1<<26));
            NVIC_DisableIRQ(ADC_IRQn);
        }
    }
}


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

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	/* Convierte el valor recibido a un valor correspondiente dentro de una escala MIN-MAX dada. */
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void Keypad_clear(char *ch){
	*ch = 0;
}

void startPlayback(uint32_t* sound_list) {

    GPDMA_LLI_Type LLI_Array;


	LLI_Array.DstAddr = (uint32_t) &(LPC_DAC->DACR);
	LLI_Array.SrcAddr = (uint32_t)(sound_list);
	LLI_Array.NextLLI = (uint32_t)&LLI_Array;
	LLI_Array.Control = 4095
							 | (1 << 18)	// source width 16 bit
							 | (1 << 22)	// dest width = word 32 bits
							 | (1 << 26)	// source increment
							 ;

    // Detener cualquier reproducción anterior
    GPDMA_ChannelCmd(0, DISABLE);
    GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
    // Configurar el canal DMA para la nueva lista de sonido
    GPDMA_Channel_CFG_Type DMA_CFG;

    DMA_CFG.ChannelNum = 0;
    DMA_CFG.SrcMemAddr = (uint32_t)sound_list;
    DMA_CFG.DstMemAddr = (uint32_t)&(LPC_DAC->DACR);
    DMA_CFG.TransferSize = LISTSIZE;
    DMA_CFG.TransferWidth = GPDMA_WIDTH_WORD;
    DMA_CFG.TransferType = GPDMA_TRANSFERTYPE_M2P;
    DMA_CFG.SrcConn = 0;
    DMA_CFG.DstConn = GPDMA_CONN_DAC;
    DMA_CFG.DMALLI = 0;

    GPDMA_Setup(&DMA_CFG);

    // Habilitar el canal DMA para iniciar la transferencia
    GPDMA_ChannelCmd(0, ENABLE);
    NVIC_EnableIRQ(DMA_IRQn);
}


void moveListDAC(uint32_t *list)
{
	/* Desplazamos los valor de la lista 6 lugares, 4 para el DAC y 2 mas para recortar los LSB. */

	for (uint32_t i = 0; i < LISTSIZE; i++)
	{
		list[i] = list[i]<<6;
	}
	return;
}
