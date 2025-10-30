#include <stdio.h>
#include <LPC17xx.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_timer.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_gpdma.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_exti.h>

#define ROW_PORT 0
#define COL_PORT 0

static const uint32_t ROW_BITS[] = { (1<<19), (1<<20), (1<<21), (1<<22) };
static const uint32_t COL_BITS[] = { (1<<23), (1<<24), (1<<25), (1<<26) };

static const char keymap[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

static volatile char last_key = 0;
static volatile uint32_t debounce_ms = 20; // antirrebote básico



void configPin(void);
void configADC(void);
void configDAC(void);
void configTimer(void);
void configDMA(void);
void configUART(void);


int main (void){
    configPin();
    configADC();
    configDAC();
    configTimer();
    configDMA();
    configUART();

    while (1){

    }

    return 0;
}

void configPin(void){
    PINSEL_CFG_Type pinCfg;

    // 
}

// --- utilidades ---
static void rows_all_high(void){
  // Configuradas como salida, subir todas a '1'
  for (int i=0;i<4;i++) GPIO_SetValue(ROW_PORT, ROW_BITS[i]);
}

static int read_col_index(void){
  uint32_t pins = GPIO_ReadValue(COL_PORT);
  for (int c=0;c<4;c++){
    if ((pins & COL_BITS[c]) == 0) return c; // activo en 0 (pull-up)
  }
  return -1;
}

static int scan_key_position(int *r_out, int *c_out){
  // Barrido: una fila por vez en 0, resto en 1
  for (int r=0;r<4;r++){
    rows_all_high();
    GPIO_ClearValue(ROW_PORT, ROW_BITS[r]); // fila activa = 0
    for (volatile int d=0; d<200; d++) __NOP(); // breve settle

    int c = read_col_index();
    if (c >= 0){
      *r_out = r; *c_out = c;
      rows_all_high();
      return 1;
    }
  }
  rows_all_high();
  return 0;
}

// --- inicialización de pines (PINSEL+PINMODE=GPIO con pull-up) ---
static void keypad_pins_init(void){
  PINSEL_CFG_Type cfg;
  cfg.Funcnum = 0; // GPIO
  cfg.OpenDrain = 0;
  cfg.Pinmode = 0; // 00 = pull-up
  cfg.Portnum = ROW_PORT;

  // Filas
  for (int i=0;i<4;i++){
    cfg.Pinnum = 19 + i;
    PINSEL_ConfigPin(&cfg);
    GPIO_SetDir(ROW_PORT, ROW_BITS[i], 1); // salida
  }
  // Columnas
  for (int i=0;i<4;i++){
    cfg.Pinnum = 23 + i;
    PINSEL_ConfigPin(&cfg);
    GPIO_SetDir(COL_PORT, COL_BITS[i], 0); // entrada con pull-up
  }

  rows_all_high();
}

// --- interrupciones GPIO (EINT3) en columnas por flanco descendente ---
static void keypad_irq_init(void){
  // Habilitar interrupción por flanco descendente en columnas
  uint32_t col_mask = 0;
  for (int i=0;i<4;i++) col_mask |= COL_BITS[i];

  GPIO_IntCmd(COL_PORT, col_mask, 0); // 0 = falling edge
  NVIC_EnableIRQ(EINT3_IRQn);         // IRQ de GPIO P0/P2
}

// --- timer simple de antirrebote con SysTick a 1ms ---
static volatile uint32_t ticks_ms = 0;
void SysTick_Handler(void){ ticks_ms++; }

static void delay_ms(uint32_t ms){
  uint32_t start = ticks_ms;
  while ((ticks_ms - start) < ms) { __WFI(); }
}

// --- ISR de GPIO: localizar tecla, antirrebote, limpiar flags ---
void EINT3_IRQHandler(void){
  // Deshabilito nuevas IRQ de columnas durante el manejo
  uint32_t col_mask = 0;
  for (int i=0;i<4;i++) col_mask |= COL_BITS[i];
  GPIO_IntCmd(COL_PORT, col_mask, 2); // Deshabilitar el NVIC despues

  // Limpiar flags latentes (falling)
  GPIO_ClearInt(COL_PORT, col_mask);

  // Antirrebote: tomar estado estable luego de breve espera
  delay_ms(debounce_ms);

  int r,c;
  if (scan_key_position(&r,&c)){
    last_key = keymap[r][c];
    // Esperar liberación
    while (read_col_index() >= 0) { /* tecla aun presionada */ }
    delay_ms(debounce_ms);
  }

  // Limpiar y re-habilitar
  GPIO_ClearInt(COL_PORT, col_mask);
  GPIO_IntCmd(COL_PORT, col_mask, 0); // 0 = falling edge (enable)
}

// --- API mínima ---
void Keypad_Init(void){
  keypad_pins_init();
  // SysTick 1ms (asumiendo CCLK conocido, p.ej. 100 MHz → 100000-1 para 1ms si SysTick usa CCLK/100)
  SysTick->LOAD = (SystemCoreClock/1000) - 1;
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  keypad_irq_init();
}

int Keypad_GetCharNonBlock(char *ch){
  if (last_key){
    *ch = last_key;
    last_key = 0;
    return 1;
  }
  return 0;
}