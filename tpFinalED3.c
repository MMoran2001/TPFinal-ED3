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
#define TIMEOUT 7000		// Arranca por default en un valor, pero el potenciometro lo varia durante la ejecucion.
#define ADCRATE 10000		// A mayor ADCRATE mayor fidelidad de sonido.
#define LISTSIZE 12000		// No superar los 15k muestras por que se llena la SRAM 32kB.


static const uint32_t ROW_BITS[] = { (1<<0), (1<<1), (1<<2), (1<<3)};
static const uint32_t COL_BITS[] = { (1<<4), (1<<5), (1<<6), (1<<7)};

static const char keymap[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

static volatile char last_key = 0;
static volatile uint32_t debounce_ms = 20;
volatile int newkey = 0;
volatile int recording = 0;
volatile int sound_index = 0;

/* Esta lista guarda el sonido grabado por el microfono. */
__IO uint16_t listADC[LISTSIZE] = {0};
__IO uint32_t *samples_count = (__IO uint32_t *)0x2007C000;		// Contador de muestras para la lista del ADC.
__IO uint32_t sound1_list[LISTSIZE] = {0};		// Lista de sonido 1
__IO uint32_t sound2_list[LISTSIZE] = {0};		// Lista de sonido 2
__IO uint32_t sound3_list[LISTSIZE] = {0};		// Lista de sonido 3

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


// --- Prototipos para las configuraciones ---
void configPin(void);
void configADC(void);
void configDAC(void);
void configTimer(void);
void configDMA(void);
void configUART(void);


// --- Handlers de interrupción ---
void SysTick_Handler(void);
void EINT3_IRQHandler(void);
void ADC_IRQHandler(void);

int main (void){
   Keypad_Init();
   configPin();

   GPIO_SetValue(0, (1<<22)); // Apaga led rojo
   GPIO_SetValue(3, (1<<25)); // Apaga led verde
   GPIO_SetValue(3, (1<<26)); // Apaga led azul
   while (1){
       char ch;
       Keypad_GetCharNonBlock(&ch);
        // acá procesás la tecla (enviar por UART, etc.)
       if(newkey){
        switch(ch){
            case '1':
                  GPIO_SetValue(3, (1<<25)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<26)); // Apaga led azul
                  GPIO_ClearValue(0, (1<<22)); // Enciende led rojo
                break;
            case '2':
                  GPIO_SetValue(3, (1<<22)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<26)); // Apaga led azul
                  GPIO_ClearValue(3, (1<<25)); // Enciende led verde
                break;
            case '3':
                  GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<25)); // Apaga led verde
                  GPIO_ClearValue(3, (1<<26)); // Enciende led azul
                break;
            case '4': // Boton para grabar, despues podemos cambiar el valor del case
              recording = 1;
                while (read_col_index() >= 0 && recording) { // Mientras se mantenga presionada la tecla y este grabando
                  //TO DO: Espera a que se suelte la tecla, porque read_col_index me bloquea el programa en scan_key_position
                  //TO DO: Ver como hacer para que el programa tome dos teclas
                  Keypad_GetCharNonBlock(&ch);
                  if(ch == '1'){
                    sound_index = 1; // Si toco la tecla '1' grabo en la lista de sonido 1
                  }
                  else if(ch == '2'){
                    sound_index = 2; // Si toco la tecla '2' grabo en la lista de sonido 2
                  }
                  else if(ch == '3'){
                    sound_index = 3; // Si toco la tecla '3' grabo en la lista de sonido 3
                  }
                  else{
                    sound_index = 0;
                  }
                }// Recording va a cambiar a 0 en el handler del ADC cuando se llene la lista
                NVIC_DisableIRQ(ADC_IRQn); // Deshabilito la interrupcion del ADC para dejar de grabar
                recording = 0;
                sound_index = 0;
                break;
            default:
                  GPIO_SetValue(0, (1<<22)); // Apaga led rojo
                  GPIO_SetValue(3, (1<<25)); // Apaga led verde
                  GPIO_SetValue(3, (1<<26)); // Apaga led azul
                break;


        }
        newkey = 0;
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
  //NVIC_SetPriority(EINT3_IRQn, 5);
}



void delay_ms(uint32_t ms){
  ticks_ms = 0;
  while (ticks_ms < ms) {} // a ver si el antirebote anda bien asi
}

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
    while (read_col_index() >= 0) { /* tecla aun presionada */ }
    delay_ms(debounce_ms);
  }

  // Limpiar y re-habilitar
  GPIO_ClearInt(COL_PORT, col_mask);
  NVIC_EnableIRQ(EINT3_IRQn); // 0 = falling edge (enable)
}

// --- API mínima ---
void Keypad_Init(void){
  keypad_pins_init();
  SysTick_Config(100000); // Systick a 1ms
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
  
  // ADC
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
  ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE); // El canal del microfono se configura apagado hasta que empiece la grabacion
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE); // El canal del potenciometro empieza activo
  ADC_BurstCmd(LPC_ADC, ENABLE); // Modo burst
  ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE); // Activo la interrupción del microfono, pero no se si activar la interrupción del potenciometro tambien 
  ADC_IntConfig(LPC_ADC, ADC_ADINTEN1, ENABLE); 
}

// --- configuración del DAC ---
void configDAC(void){
  DAC_CONVERTER_CFG_Type dacCfg;
  dacCfg.DBLBUF_ENA = 0; // Deshabilitar double buffer
  dacCfg.CNT_ENA = 1;    // Habilitar contador de tiempo
  dacCfg.DMA_ENA = 1;    // Habilitar acceso a DMA

  DAC_Init(LPC_DAC); // Se inicializa el DAC con BIAS de 700uA
  DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
  DAC_SetDMATimeOut(LPC_DAC, TIMEOUT); // Tiempo de espera DMA
}

// --- configuración de DMA ---
// Podemos copiar lo de los chicos y modificarlo para lo nuestro


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
volatile uint32_t ticks_ms = 0;
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
    while (read_col_index() >= 0) { /* tecla aun presionada */ }
    delay_ms(debounce_ms);
  }

  // Limpiar y re-habilitar
  GPIO_ClearInt(COL_PORT, col_mask);
  NVIC_EnableIRQ(EINT3_IRQn); // 0 = falling edge (enable)
}

// --- Handler de interrupcion del ADC ---
void ADC_IRQHandler(void){
  if(ADC_GlobalGetStatus(LPC_ADC, 1)){ // Verifico si se levanto la flag DONE global
    if(ADC_ChannelGetStatus(LPC_ADC, 0, 1)){ // Verifico si se levanto el flag de DONE del canal del microfono (0)
      if (*samples_count <= LISTSIZE)
		{
			/* Comenzamos a grabar un audio y guardarlo en el array que corresponda. */
			GPIO_ClearValue(0, (1<<22)); // Enciende led rojo
      if(sound_index == 1){
        sound1_list[*samples_count] = ((LPC_ADC->ADDR0)>>6) & 0x3FF;
      }
      else if(sound_index == 2){
        sound2_list[*samples_count] = ((LPC_ADC->ADDR0)>>6) & 0x3FF;
      }
      else{
        sound3_list[*samples_count] = ((LPC_ADC->ADDR0)>>6) & 0x3FF;
      }
		}
		else
		{
			GPIO_SetValue(0, (1<<22));  	// Apaga el led rojo.
			GPIO_SetValue(0, (1<<25));  	// Apaga el led verde.
			GPIO_SetValue(0, (1<<26));  	// Apaga el led azul.
			*samples_count = 0;
			recording = 0;
			moveListDAC();
		}

    }
    if(ADC_ChannelGetStatus(LPC_ADC, 1, 1)){ // Verifico si se levanto el flag de DONE del canal del potenciometro (1)
      uint32_t pot_value = ADC_ChannelGetData(LPC_ADC, 1); // Leo el valor del potenciometro
      uint32_t timeout = map(pot_value, 0, 1024, 5000, 20000); // Escalo el valor del potenciometro para usarlo como timeout (Me base en el otro codigo)
      DAC_SetDMATimeOut(LPC_DAC, timeout); // Actualizo el timeout del DAC
    }

  }
}

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
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}