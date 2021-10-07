/*
 * File:   newmain.c
 * Author: Vladimír Kelbl
 *
 * Mám kit Cislo 9.
 * 0:GPIO-Tetris
 * 1:UART-Mod kalkulacka secteni/odecteni dvou cisel
 * 2:PWM-Ovladani rychlosti motoru potenciometrem bez led
 * 3:ADC-Ovladani bargraph (led) POT1
 * 4:GAME-1D Pong 
 * 
 * poznámky vyvojare
 * 0: funguje
 * 1: nefunguje spravne vypsani vysledku a vraceni se do menu
 * 2: funguje
 * 3: funguje
 * 4: funguje, jen se mi nepodarilo dat znak ASCII 219 jako odpalova palky. Podle mne to nejde.
 */


#pragma config FOSC = HSMP      // Oscillator Selection bits (HS oscillator (medium power 4-16 MHz))
#pragma config PLLCFG = OFF    // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
 
 
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <pic18.h>

#include "lcd.h"
 
#define ISR_PERIOD 0xFFFF - 50000
#define _XTAL_FREQ 8E6
#define TETRIS_SPEED 100
#define GAME_SPEED 200
 
#define BTN1    PORTCbits.RC0
#define BTN2    PORTAbits.RA4
#define BTN3    PORTAbits.RA3
#define BTN4    PORTAbits.RA2

#define SETDUTY(x) CCPR1L = x

typedef enum{
    STATE0obrTET = 0,
    STATE1obrKAL,
    STATE2obrPWM,
    STATE3obrADC,
    STATE4obrHRA,
    STATE5TET,
    STATE6KAL,
    STATE7PWM,
    STATE8ADC,
    STATE9HRA,
} StateMachine;

typedef struct
{
    char data [80];
    uint8_t length;
    char full;
    int vysledek;
} priklad;

volatile uint16_t pot1_val;
//volatile uint8_t data_buffer[20];
volatile priklad message;

//volatile uint8_t pomoc = 0;

// Functions prototypes
void state0obrtet(void);
void state1obrkal(void);
void state2obrpwm(void);
void state3obradc(void);
void state4obrhra(void);
void state5tet(void);
void state6kal(void);
void state7pwm(void);
void state8adc(void);
int state9hra(void);

void driveLED(char in);
void putch(char msg);
char getche (void);
char getch (void);
  
volatile char gFlag = 0;
 
void __interrupt() ISR(void){
    char aktualni;
    static uint8_t rx_i = 0;
    
    if(TMR1IE & TMR1IF){
        TMR1 = ISR_PERIOD;
        gFlag = 1;
        TMR1IF = 0;
    }
    if (ADIE && ADIF) {
        pot1_val = (ADRESH << 8) | ADRESL;
        ADIF = 0;
    }
    /*
    if(RC1IF && RC1IE){
        aktualni = RCREG1;
        //get_string
        data_buffer[pomoc] = aktualni;
        if (data_buffer[pomoc] == '='){
            pomoc = 0;
        }
        pomoc++;
        //RC1IF = 0;
     }
    */
    if(RC1IE && RC1IF) {  
        aktualni = RCREG1;
        if (aktualni == '=') {
            message.length = rx_i;
            rx_i = 0;             
            message.full = 1;
        } else {
            message.data[rx_i] = aktualni;
            rx_i++;
        }        
    }
    
    if(TX1IE && TX1IF){
        TXREG1 = message.vysledek;
        //printf("\n");
        TX1IE = 0;
        /*
        if (message.length == 1){
            TX1IE = 0;
        } else {
            message.length--;
        }
        */ 
    }
}
 

void FSM(void);
 
void init(void)
{
    // disable analog properties
    ANSELA = 0x00;
    ANSELC = 0x00;
 
    // set pins as outputs
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISDbits.TRISD4 = 0;
    TRISDbits.TRISD5 = 0;
    TRISDbits.TRISD6 = 0;
    
    // LEDs off
    LATD2 = 1;
    LATD3 = 1;
    LATC4 = 1;
    LATD4 = 1;
    LATD5 = 1;
    LATD6 = 1;
    
    
    // set pins as inputs
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA3 = 1;
    TRISAbits.TRISA2 = 1;
    TRISCbits.TRISC0 = 1;
    
    // uart
    SPBRG1 = 12;              // (8_000_000 / (64 * 9600)) - 1
    TXSTA1bits.SYNC = 0;      // nastaveni asynchroniho modu
    RCSTA1bits.SPEN = 1;      // zapnuti UART
    TXSTA1bits.TXEN = 1;      // zapnuti TX
    RCSTA1bits.CREN = 1;      // zapnuti RX
 
    TRISD = 0x00;           // PORTD jako vystup
    TRISCbits.TRISC6 = 1;   // TX pin jako vstup
    TRISCbits.TRISC7 = 1;   // rx pin jako vstup
    
    // TIMER
    T1CONbits.TMR1CS = 0b00;
    T1CONbits.T1CKPS = 0b11;    // TMR1 prescaler
   
    // interrupts
    PEIE = 1;                   // global interrupt enable// peripheral interrupt enable
    GIE = 1;                    // global interrupt enable
    TMR1IE = 1;                 // enable TMR1 interrupt
    T1CONbits.TMR1ON = 1;       // timer ON
    
    ADIF = 0;
    ADIE = 1;
    
    RC1IE = 1;
    
    // adc
    TRISAbits.RA5 = 1;
    ANSELAbits.ANSA5 = 1;
    
    ADCON2bits.ADCS = 0b110; // Fosc/64
    ADCON1bits.PVCFG = 0b00; // Vrefp = Vdd 3,3 V
    ADCON1bits.NVCFG = 0b00; // Vrefn = Vss 0,0 V
    ADCON2bits.ACQT = 0b110; // 16Tad
    
    ADCON0bits.ADON = 1;
    
    // adc pro pwm
    ANSELE = 0b1;                   //RE0/AN5
    
    // PWM
    //TRISDbits.RD5 = 1;              // nastavim jako vstup pin P1B
    TRISCbits.RC2 = 1;              // nastavim jako vstup pin P1A
    PSTR1CON |= 0b01;               // steering na P1B a P1A
    
    CCPTMRS0bits.C1TSEL = 0b00;     // Timer 2 
    PR2 = 199;                      // f = 10kHz
    CCP1CONbits.P1M = 0b00;         // PWM single
    CCP1CONbits.CCP1M = 0b1100;     // PWM single
    CCPR1L = 0;                     // strida 0%    
    T2CONbits.T2CKPS = 0b00;        // 1:1 Prescaler
    TMR2IF = 0;                     // nastavi se az pretece timer
    TMR2ON = 1;                     // staci zapnout defaultne je nastaven jak chceme
    while(!TMR2IF){};               // cekam az jednou pretece
        
    //TRISDbits.RD5 = 0;              // nastavim jako vystup pin P1B
    TRISCbits.RC2 = 0;              // nastavim jako vystup pin P1A
    
}
 
void main(void)
{
    init();
    LCD_Init();
 
    while(1)
    {
        if (gFlag)
        {
            FSM();
            gFlag = 0;
        }
    }
}
 
void FSM(void)
{
    static StateMachine state = STATE0obrTET;
    
    switch (state)
    {
        /************************************************************0stav*/
    case STATE0obrTET:
        if (BTN2){
            __delay_ms(5);
            if(BTN2){
                state = STATE1obrKAL;
                while(BTN2);
            }
        }
        if (BTN3){
            __delay_ms(5);
            if(BTN3){
                state = STATE5TET;
                while(BTN3);
            }
        }
        state0obrtet();
        driveLED(0);
        break;
        /************************************************************1stav*/
    case STATE1obrKAL:
        if (BTN1){
            __delay_ms(5);
            if(BTN1){
                state = STATE0obrTET;
                while(BTN1);
            }
        }
        if (BTN2){
            __delay_ms(5);
            if(BTN2){
                state = STATE2obrPWM;
                while(BTN2);
            }
        }
        if (BTN3){
            __delay_ms(5);
            if(BTN3){
                state = STATE6KAL;
                while(BTN3);
            }
        }
        state1obrkal();
        break;
        /************************************************************2stav*/
    case STATE2obrPWM:
        if (BTN1){
            __delay_ms(5);
            if(BTN1){
                state = STATE1obrKAL;
                while(BTN1);
            }
        }
        if (BTN2){
            __delay_ms(5);
            if(BTN2){
                state = STATE3obrADC;
                while(BTN2);
            }
        }
        if (BTN3){
            __delay_ms(5);
            if(BTN3){
                state = STATE7PWM;
                while(BTN3);
            }
        }
        state2obrpwm();
        SETDUTY(0);
        break;
        /************************************************************3stav*/
    case STATE3obrADC:
        if (BTN1){
            __delay_ms(5);
            if(BTN1){
                state = STATE2obrPWM;
                while(BTN1);
            }
        }
        if (BTN2){
            __delay_ms(5);
            if(BTN2){
                state = STATE4obrHRA;
                while(BTN2);
            }
        }
        if (BTN3){
            __delay_ms(5);
            if(BTN3){
                state = STATE8ADC;
                while(BTN3);
            }
        }
        state3obradc();
        driveLED(0);
        break;
        /************************************************************4stav*/
    case STATE4obrHRA:
        if (BTN1){
            __delay_ms(5);
            if(BTN1){
                state = STATE3obrADC;
                while(BTN1);
            }
        }
        if (BTN3){
            __delay_ms(5);
            if(BTN3){
                state = STATE9HRA;
                while(BTN3);
            }
        }
        state4obrhra();
        break;
        /************************************************************5stav*/
    case STATE5TET:
        if (BTN4){
            __delay_ms(5);
            if(BTN4){
                state = STATE0obrTET;
                while(BTN4);
            }
        }
        state5tet();
        break;
        /************************************************************6stav*/
    case STATE6KAL:
        if (BTN4){
            __delay_ms(5);
            if(BTN4){
                state = STATE1obrKAL;
                while(BTN4);
            }
        }
        state6kal();
        break;
        /************************************************************7stav*/
    case STATE7PWM:
        if (BTN4){
            __delay_ms(5);
            if(BTN4){
                state = STATE2obrPWM;
                while(BTN4);
            }
        }
        state7pwm();
        break;
        /************************************************************8stav*/
    case STATE8ADC:
        if (BTN4){
            __delay_ms(5);
            if(BTN4){
                state = STATE3obrADC;
                while(BTN4);
            }
        }
        state8adc();
        break;
        /************************************************************9stav*/
    case STATE9HRA:
        state = state9hra();
        break;
    }
}


void state0obrtet(void){
    
    LCD_ShowString(1, "-0. Tetris      ");
    LCD_ShowString(2, " 1. Kalkulacka  ");    
    
}
void state1obrkal(void){
    
    LCD_ShowString(1, " 0. Tetris      ");
    LCD_ShowString(2, "-1. Kalkulacka  ");
    
}
void state2obrpwm(void){
    
    LCD_ShowString(1, "-2: PWM motor   ");
    LCD_ShowString(2, " 3: ADC bargraph");
    
}
void state3obradc(void){
    
    LCD_ShowString(1, " 2: PWM motor   ");
    LCD_ShowString(2, "-3: ADC bargraph");
    
}
void state4obrhra(void){
    
    LCD_ShowString(1, "-4: 1D Pong     ");
    LCD_ShowString(2, "                ");
    
}
void state5tet(void){
    
    LCD_ShowString(1, "State: Tetris      ");
    LCD_ShowString(2, "State: Tetris      ");
    //printf("Current state: tetris\n");
    
    int i;
    driveLED(0);
    __delay_ms(TETRIS_SPEED);
    for(i = 1; i < 33; i = i<<1){
        driveLED(i);
        __delay_ms(TETRIS_SPEED);
    }
    for(i = 1; i < 17; i = i<<1){
        i = i + 32;
        driveLED(i);
        i = i - 32;
        __delay_ms(TETRIS_SPEED);
    }
    for(i = 1; i < 9; i = i<<1){
        i = i + 48;
        driveLED(i);
        i = i - 48;
        __delay_ms(TETRIS_SPEED);
    }
    for(i = 1; i < 5; i = i<<1){
        i = i + 56;
        driveLED(i);
        i = i - 56;
        __delay_ms(TETRIS_SPEED);
    }
    for(i = 1; i < 3; i = i<<1){
        i = i + 60;
        driveLED(i);
        i = i - 60;
        __delay_ms(TETRIS_SPEED);
    }
    LATD2 = 0;
    __delay_ms(TETRIS_SPEED);
    
}
void state6kal(void){
    
    LCD_ShowString(1, "State:kalkulacka");
    LCD_ShowString(2, "State:kalkulacka");
    
    //printf("Welcome!\nJsem jednoducha kalkulacka. Zadejte priklad\ns jednou jednoduchou operaci (+-*/) a ukoncete ho znamenkem =. \nAplikaci Termit mejte nastavenou\ndo stavu Transmitted text -> Append nothing.\n\n");
    
    char operace, aktualni;
    //int i = 0, size = 0, zacatekPrvniho, operaceMozna = 0, dekadicky = 1, prvni = 0, druhy = 0, vysledek = 0;
    int zacatekPrvniho, dekadicky = 1, prvni = 0, druhy = 0;
    /*
    for(int i=0; i<=49; i++){
        text[i] = NULL;
    }    
    while(1){
        aktualni = getch();
        if (aktualni == '='){
            break;
        }else{  
            text[i] = aktualni;
        }
        i++;
    }
    */
    //while(!RC1IF);
    /*
    for(int i=0; i<=49; i++){
        if (data_buffer[i] != '=')
            size++;
    }
    size--;
    zacatekPrvniho = size;
    */
    while(1){
        if (message.full){
            message.full = 0;  
            zacatekPrvniho = message.length;
            for(int i = message.length; i >= 0; i--){
                aktualni = message.data[i];
                if((aktualni == '+') | (aktualni == '-') | (aktualni == '*') | (aktualni == '/')){
                    operace = aktualni;
                    //operaceMozna++;
                    zacatekPrvniho--;
                    break;
                }else{
                    aktualni = aktualni - '0';
                    druhy += dekadicky*aktualni;
                    dekadicky *= 10;
                    zacatekPrvniho--;
                }   
            }
            dekadicky = 1;
            for(int i = zacatekPrvniho; i >= 0; i--){
                aktualni = message.data[i] - '0';
                prvni += dekadicky*aktualni;
                dekadicky *= 10;
            }
            switch (operace){
                case 43 :
                    message.vysledek = prvni + druhy;
                    //printf("%d\n", vysledek);
                    break;
                case 45 :
                    message.vysledek = prvni - druhy;
                    //printf("%d\n", vysledek);
                    break;
                case 42 :
                    message.vysledek = prvni * druhy;
                    //printf("%d\n", vysledek);
                    break;
                case 47 :
                    message.vysledek = prvni / druhy;
                    //printf("%d\n", vysledek);
                    break;
            }
            
            TX1IE = 1;            
        }            
    }   
}
void state7pwm(void){
    
    ADCON2bits.ADFM = 0;            //left justified
    ADCON0bits.CHS = 5;             // kanal AN5
    
    LCD_ShowString(1, "State:motorSpeed");
    LCD_ShowString(2, "State:motorSpeed");
    
        GODONE = 1;                 // spustim konverzi
        while(GODONE){};            // cekam na konverzi
        SETDUTY(ADRESH);            // nastavim stridu pouzivam jen 8hornich bitu
    
}
void state8adc(void){
    ADCON2bits.ADFM = 1;            //right justified
    ADCON0bits.CHS = 0b00100;       // kanal AN4
    
    LCD_ShowString(1, "State: ADC      ");
    LCD_ShowString(2, "       bargraph ");
    
    uint8_t led_state = 0;
    
    ADCON0bits.GODONE = 1;
        
        if (pot1_val < 145) {
            led_state = 0;
        } else if (pot1_val >= 145 && pot1_val < 290) {
            led_state = 0b100000;
        } else if (pot1_val >= 290 && pot1_val < 435) {
            led_state = 0b110000;
        } else if (pot1_val >= 435 && pot1_val < 585) {
            led_state = 0b111000;
        } else if (pot1_val >= 585 && pot1_val < 730) {
            led_state = 0b111100;
        } else if (pot1_val >= 730 && pot1_val < 875) {
            led_state = 0b111110;
        } else if (pot1_val >= 875) {
            led_state = 0b111111;
        }
        
        driveLED(led_state);
        __delay_ms(50);
    
}
int state9hra(void){
    //printf("Current state: 1D pong\n");
    int player1 = 3, player2 = 3, konec = 0;
    
while (1) {    
    switch (player1) {
        case 3:
            switch (player2) {
                case 3:
                    LCD_ShowString(1, "Zivoty: +++  +++");
                    break;
                case 2:
                    LCD_ShowString(1, "Zivoty: +++   ++");
                    break;
                case 1:
                    LCD_ShowString(1, "Zivoty: +++    +");
                    break;
                case 0:
                    LCD_ShowString(1, "   GAME OVER!   ");
                    LCD_ShowString(2, "  Player 1 won  ");
                    konec = 1;
                    break;
            }
            break;
        case 2:
            switch (player2) {
                case 3:
                    LCD_ShowString(1, "Zivoty: ++   +++");
                    break;
                case 2:
                    LCD_ShowString(1, "Zivoty: ++    ++");
                    break;
                case 1:
                    LCD_ShowString(1, "Zivoty: ++     +");
                    break;
                case 0:
                    LCD_ShowString(1, "   GAME OVER!   ");
                    LCD_ShowString(2, "  Player 1 won  ");
                    konec = 1;
                    break;
            }
            break;
        case 1:
            switch (player2) {
                case 3:
                    LCD_ShowString(1, "Zivoty: +    +++");
                    break;
                case 2:
                    LCD_ShowString(1, "Zivoty: +     ++");
                    break;
                case 1:
                    LCD_ShowString(1, "Zivoty: +      +");
                    break;
                case 0:
                    LCD_ShowString(1, "   GAME OVER!   ");
                    LCD_ShowString(2, "  Player 1 won  ");
                    konec = 1;
                    break;
            }
            break;
        case 0:
            LCD_ShowString(1, "   GAME OVER!   ");
            LCD_ShowString(2, "  Player 2 won  ");
            konec = 1;
            break;
    }
    if (konec == 1) {
        __delay_ms(2000);
        break;
    }
    
    __delay_ms(GAME_SPEED/2 - 5);
    LCD_ShowString(2, "M o            M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M  o           M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M   o          M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M    o         M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M     o        M");
    __delay_ms(GAME_SPEED);    
    LCD_ShowString(2, "M      o       M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M       o      M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M        o     M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M         o    M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M          o   M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M           o  M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M            o M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M             oM");
    __delay_ms(GAME_SPEED/2 - 5);
    
    if (BTN4){
        __delay_ms(5);
        if(BTN4){}
    }else{
        player2--;
    }
    switch (player1) {
        case 3:
            switch (player2) {
                case 3:
                    LCD_ShowString(1, "Zivoty: +++  +++");
                    break;
                case 2:
                    LCD_ShowString(1, "Zivoty: +++   ++");
                    break;
                case 1:
                    LCD_ShowString(1, "Zivoty: +++    +");
                    break;
                case 0:
                    LCD_ShowString(1, "   GAME OVER!   ");
                    LCD_ShowString(2, "  Player 1 won  ");
                    konec = 1;
                    break;
            }
            break;
        case 2:
            switch (player2) {
                case 3:
                    LCD_ShowString(1, "Zivoty: ++   +++");
                    break;
                case 2:
                    LCD_ShowString(1, "Zivoty: ++    ++");
                    break;
                case 1:
                    LCD_ShowString(1, "Zivoty: ++     +");
                    break;
                case 0:
                    LCD_ShowString(1, "   GAME OVER!   ");
                    LCD_ShowString(2, "  Player 1 won  ");
                    konec = 1;
                    break;
            }
            break;
        case 1:
            switch (player2) {
                case 3:
                    LCD_ShowString(1, "Zivoty: +    +++");
                    break;
                case 2:
                    LCD_ShowString(1, "Zivoty: +     ++");
                    break;
                case 1:
                    LCD_ShowString(1, "Zivoty: +      +");
                    break;
                case 0:
                    LCD_ShowString(1, "   GAME OVER!   ");
                    LCD_ShowString(2, "  Player 1 won  ");
                    konec = 1;
                    break;
            }
            break;
        case 0:
            LCD_ShowString(1, "   GAME OVER!   ");
            LCD_ShowString(2, "  Player 2 won  ");
            konec = 1;
            break;
    }
    if (konec == 1) {
        __delay_ms(2000);
        break;
    }
    
    __delay_ms(GAME_SPEED/2 - 5);    
    LCD_ShowString(2, "M            o M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M           o  M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M          o   M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M         o    M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M        o     M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M       o      M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M      o       M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M     o        M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M    o         M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M   o          M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M  o           M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "M o            M");
    __delay_ms(GAME_SPEED);
    LCD_ShowString(2, "Mo             M");
    __delay_ms(GAME_SPEED/2 - 5);
    
    if (BTN1){
        __delay_ms(5);
        if(BTN1){}
    }else{
        player1--;
    }
}
    return 4;
}

void driveLED(char in){
        in = ~in;
        LATD2 = in & 1;             //LED0
        LATD3 = in & 2 ? 1 : 0;     //LED1
        LATC4 = in & 4 ? 1 : 0;     //LED2
        LATD4 = in & 8 ? 1 : 0;     //LED3
        LATD5 = in & 16 ? 1 : 0;    //LED4
        LATD6 = in & 32 ? 1 : 0;    //LED5
}

void putch(char msg){
    while(!TX1IF){};
    TX1REG = msg;
}

char getche (void){
    while(!RC1IF);

    return RCREG1;
}

char getch (void){
     return getche();
}