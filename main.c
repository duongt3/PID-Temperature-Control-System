#include <msp430.h>
#include <math.h>
#include <stdint.h>
#include "adc.h"
#include "project_settings.h"
#include "timing.h"
#include "task.h"
#include "subsystem.h"
#include "uart.h"
#include "hal_general.h"

/* Function Declarations */
void SetClk24MHz(void);
void SetVcoreUp (unsigned int level);

//PID Variables
float Kp = 3;
float Kd = 2;
float Ki = 0.5;

int PrevError;
int Error;
int SamplingTime;
int RunningError;
int PWMadjustment;
float PropError;
float DerivError;
float IntError;

volatile float Res; //resistance of thermistor
volatile float voltage; //ADC input
volatile float temp; //current temperature calculated by the equations
volatile float lastTemp; //previous temperature
volatile float tempInt; //current temperature stored as an integer for UART transmission
volatile int targetTemp; //target temperature as defined via UART
volatile float deltaTemp; //difference from target temperature to current temperature reading
volatile float timeTemp; //difference from current temperature to last temperature
volatile float av[5];
int i;
static uint8_t sys_id;

void Average(void);
void PID(void);
void measure(uint16_t value);
void Display(void);
static void Receiver(int argc, char *argv[]);

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    SetClk24MHz();
    DisableInterrupts();

    Timing_Init();
    Task_Init();
    UART_Init(UART1);

    lastTemp = 0;                               //initialize previous temperature to 0 degrees
    i = 0;
    Error = 0;
    PrevError = 0;
    RunningError = 0;
    targetTemp = 45;
    PropError = 0;
    DerivError = 0;
    IntError = 0;
    ADC_AddChannel(0, 10, (callback_input_t)measure, 0);

    //PWM
    P2DIR |= BIT0;                                //set out
    P2SEL |= BIT0;                               //Set pin 2.0 as PWM OUT


    //PWM
    TA1CCR0 = 1000;                           //total period
    TA1CCTL1 = OUTMOD_7;                      //PWM mode reset/set
    TA1CCR1 = 100;                        //initial ON period
    TA1CTL = TASSEL_2 + MC_1 + ID_2;      //SMCLK, UP mode, divide by 4


    EnableInterrupts();
    ADC_Init();
    version_t version;
    version.word = 0x01010001;
    sys_id = Subsystem_Init("Temp", version, Receiver);
    LogMsg(sys_id, "Initialized");
    SamplingTime = 1;
    Task_Schedule(Average, 0, 50, 10);
    Task_Schedule(Display, 0, 60, 1000);
    Log_EchoOn();
    while(1)
    {
        SystemTick();
    }

}

void Average(void)
{
    av[i] = voltage - 100;
    i++;
    if (i > 4)
    {
       int total  = av[0] + av[1] + av[2] + av[3] + av[4];
       int avFilter = total/5;
       temp = avFilter/10;
       PID();
       i = 0;
       tempInt = (int)temp;
    }
}

void Display(void)
{
    LogMsg(sys_id, "Temperature, PWM, Error: %f, %i, %f, %f, %f", tempInt, TA1CCR1, PropError, DerivError, IntError);
}

void measure(uint16_t value)
{
   voltage = value;
}

static void Receiver(int argc, char *argv[])
{

    LogMsg(sys_id, "Temperature: %f:", tempInt);

}

void PID (void)
{
    PrevError = Error;
    Error = targetTemp - tempInt;
    RunningError += Error*SamplingTime;
    PropError = Kp * Error;
    DerivError = Kd * ((Error - PrevError)/SamplingTime);
    IntError = Ki * RunningError;
    if (IntError < -40)
    {
        IntError = -40;
    }
    else if (IntError > 40)
    {
        IntError = 40;
    }
    PWMadjustment = PropError + DerivError + IntError;
    if (TA1CCR1 < 1000 && TA1CCR1 > 100)
    {
        TA1CCR1 = TA1CCR1 - PWMadjustment;
    }
    else if(TA1CCR1 >= 1000)
    {
        TA1CCR1 = 999;
    }
    else if(TA1CCR1  <= 100)
    {
        TA1CCR1 = 101;
    }


}

void SetClk24MHz(){
    // Increase Vcore setting to level3 to support fsystem=25MHz
    // NOTE: Change core voltage one level at a time..
    SetVcoreUp (0x01);
    SetVcoreUp (0x02);
    SetVcoreUp (0x03);

    P5SEL |= BIT2+BIT3;
    UCSCTL6 &= ~XT2OFF; // Enable XT2
    UCSCTL6 &= ~XT2BYPASS;
    UCSCTL3 = SELREF__XT2CLK; // FLLref = XT2
    UCSCTL4 |= SELA_2 + SELS__DCOCLKDIV + SELM__DCOCLKDIV;

    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
    do
    {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                        // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

    // Disable the FLL control loop
    __bis_SR_register(SCG0);

    // Select DCO range 24MHz operation
    UCSCTL1 = DCORSEL_7;
    /* Set DCO Multiplier for 24MHz
    (N + 1) * FLLRef = Fdco
    (5 + 1) * 4MHz = 24MHz  */
    UCSCTL2 = FLLD0 + FLLN0 + FLLN2;
    // Enable the FLL control loop
    __bic_SR_register(SCG0);

  /* Worst-case settling time for the DCO when the DCO range bits have been
     changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
     UG for optimization.
     32 x 32 x 24MHz / 4MHz = 6144 = MCLK cycles for DCO to settle */
  __delay_cycles(70000);

    // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
    do {
        // Clear XT2,XT1,DCO fault flags
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        // Clear fault flags
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG); // Test oscillator fault flag
}

void SetVcoreUp (unsigned int level)
{
  // Open PMM registers for write
  PMMCTL0_H = PMMPW_H;
  // Set SVS/SVM high side new level
  SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
  // Set SVM low side to new level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  // Wait till SVM is settled
  while ((PMMIFG & SVSMLDLYIFG) == 0);
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;
  // Wait till new level reached
  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);
  // Set SVS/SVM low side to new level
  SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;
}




