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
            case '4':
                  GPIO_ClearValue(0, (1<<22)); // Enciende led rojo
                  GPIO_ClearValue(3, (1<<25)); // Enciende led verde
                  GPIO_SetValue(3, (1<<26)); // Enciende led azul
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

// --- timer simple de antirrebote con SysTick a 1ms ---
volatile uint32_t ticks_ms = 0;
void SysTick_Handler(void){ ticks_ms++; }

void delay_ms(uint32_t ms){
  //uint32_t start = ticks_ms;
  //while ((ticks_ms - start) < ms) { __WFI(); }
for (uint32_t i = 0; i < 50000; i++){}
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
    //printf("columna: %d\n",c);
    //printf("fila: %d\n",r);
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
  // SysTick 1ms (asumiendo CCLK conocido, p.ej. 100 MHz → 100000-1 para 1ms si SysTick usa CCLK/100)
  SysTick_Config(SystemCoreClock/1000);
  keypad_irq_init();
}

void Keypad_GetCharNonBlock(char *ch){
    *ch = last_key;
    last_key = 0;

}



/*



CONFIGURACIONES DE LPC



*/

// --- configuración de pines ---
void configPin(void){
  PINSEL_CFG_Type PinCfg = {0};

  // ADC
  //PinCfg.Portnum = 0;
  //PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  //PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  //PINSEL_ConfigPin(&PinCfg);

  // DAC
  //PinCfg.Funcnum = 2;
  //PinCfg.Pinnum = 26;
  //PINSEL_ConfigPin(&PinCfg);

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

  // UART

}
