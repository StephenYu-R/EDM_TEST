//*****************************************************//
/*
 * EDM_TEST.c
 *
 *  Created on: 2016-3-10
 *  	1st Edition: 2016-3-20
 *  	2nd Edition: 2016-4-3
 *  	3rd Edition: 2016-4-20
 *  	4th Edition: 2016-5-3
 *  Author: StephenYu
 */
// Description:
// Test on Single Pulse EDM Discharge Detecting System
//*****************************************************//

#include "DSP2833x_Device.h"     // DSP2833x Header file Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

/****************Initial Ports Definition*****************/
//	LED_DOWN GPIO86				LED_RUN GPIO84
//	LED_UP GPIO82				LED_DC GPIO80
//	XINT GPIO12
//	Pulse GPIO1/EPWM1B
//	Motor_Direction GPIO10
//	Motor_Drive		GPIO0/EPWM1A
#define KEY_1 GpioDataRegs.GPCDAT.bit.GPIO72	//Preset Key
#define KEY_2 GpioDataRegs.GPCDAT.bit.GPIO74	//Experiment Key
#define KEY_3 GpioDataRegs.GPCDAT.bit.GPIO76	//Reverse Key
#define KEY_4 GpioDataRegs.GPCDAT.bit.GPIO78	//Stop Key
//	Discharge Gap Switch
#define KEY_5 GpioDataRegs.GPBDAT.bit.GPIO62
#define KEY_6 GpioDataRegs.GPBDAT.bit.GPIO60
#define KEY_7 GpioDataRegs.GPBDAT.bit.GPIO58
#define KEY_8 GpioDataRegs.GPBDAT.bit.GPIO56
//	Discharge Frequency Switch
#define KEY_9 GpioDataRegs.GPBDAT.bit.GPIO54
#define KEY_10 GpioDataRegs.GPBDAT.bit.GPIO52
#define KEY_11 GpioDataRegs.GPBDAT.bit.GPIO50
#define KEY_12 GpioDataRegs.GPBDAT.bit.GPIO48


/***************Global Variables Definition****************/
Uint16 timer_int_cnt = 0;	//Initialize the Timer Interrupt Counter
Uint16 Direction=0;	//Motor Rotating Direction
Uint16 Discharge=0;	//Discharge Flag
Uint16 Exper=0;	//Experiment Flag
#define DG (KEY_5)*8+(KEY_6)*4+(KEY_7)*2+(KEY_8)*1;	//Discharge Gap
#define DF (KEY_9)*8+(KEY_10)*4+(KEY_11)*2+(KEY_12)*1;	//Discharge Frequency


/****************EPWM1 Period(Frequency) Configuration*******************/
#define EPWM1_TIMER_TBPRD  3750  // Period Value
#define EPWM1_MAX_CMPA     3700
#define EPWM1_MID_CMPA	   1500
#define EPWM1_MIN_CMPA       0
#define EPWM1_MAX_CMPB     3700
#define EPWM1_MIN_CMPB       0

/****************Prototype Statements for Functions *******************/
void Gpio_select(void);
void Delay(void);
void Scan_Key(void);
void InitEPwm1(void);
void Run(void);
interrupt void epwm1_isr(void);
interrupt void cpu_timer0_isr(void);
interrupt void xint1_isr(void);


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:Main function        	    	*/
/*------------------------------------------*/
void main(void)
{
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the DSP2833x_SysCtrl.c file.
	   InitSysCtrl();

	// Step 2. Initalize GPIO:
	// This example function is found in the DSP2833x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	// InitGpio();  // Skipped for this example

	// For this example use the following configuration:
	   Gpio_select();
    // For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
    // These functions are in the DSP2833x_EPwm.c file
	   InitEPwm1Gpio();

	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	   DINT;

	// Initialize PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	   InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	   IER = 0x0000;
	   IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
	// This function is found in DSP2833x_PieVect.c.
	   InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
	   EALLOW;  // This is needed to write to EALLOW protected registers
	   PieVectTable.TINT0 = &cpu_timer0_isr;
	   PieVectTable.EPWM1_INT = &epwm1_isr;
	   PieVectTable.XINT1 = &xint1_isr;
	   EDIS;    // This is needed to disable write to EALLOW protected registers

    // Step 4. Initialize the Device Peripheral.
	   EALLOW;
	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;	// Disable the Time Base Clock for Epwm1
	   EDIS;

	   InitEPwm1();	// Initialize the Epwm1

	   EALLOW;
	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// Enable the Time Base Clock for Epwm1
	   EDIS;

	// Initialize the Device Peripheral. This function can be
	// found in DSP2833x_CpuTimers.c
	   InitCpuTimers();   // Initialize the Cpu Timers

	#if (CPU_FRQ_150MHZ)
	// Configure the Period for CPU Timer:
	// 150MHz CPU frequency, units(us):
       ConfigCpuTimer(&CpuTimer0, 150, 100000);
	#endif

// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
// below settings must also be updated.

   CpuTimer0Regs.TCR.all = 0x0001; // Configure TIE = 1. Disable TIMER0 INT

	// Step 5. User specific code:

    // Enable PIE fetching from PIE vector table
	   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1
       PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
	   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable XINT0 in the PIE: Group 1 interrupt 4
	   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;

	// Configure the XINT1 Control Register
	   XIntruptRegs.XINT1CR.bit.POLARITY = 0;      // Interrupt generated on a falling edge(high to low)

	// Enable the XINT1
	   XIntruptRegs.XINT1CR.bit.ENABLE = 1;

	// Enable CPU INT3 which is connected to EPWM1 INT:
	   IER |= M_INT3;
	// Enable CPU int1 which is connected to CPU-Timer 0 & XINT 1
	   IER |= M_INT1;

    // Enable global Interrupts and higher priority real-time debug events:
       EINT;   // Enable Global interrupt INTM
	   ERTM;   // Enable Global realtime interrupt DBGM

	// Step 6. IDLE_loop
	   for(;;)
	   {
		   asm("NOP");	//Short delay
	   }
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:Delay					    	*/
/*------------------------------------------*/
void Delay()
{
    Uint32      i;
	Uint32      j;
	for(i=0;i<1;i++)
    for (j = 0; j < 100000; j++);
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:Motor drive				 	  	*/
/*------------------------------------------*/
void Run(void)
{
	if(Direction==0)	//Down
	{
	    // Set actions.
		GpioDataRegs.GPASET.bit.GPIO10=1;			  //CW
	    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set EPWM1A on Zero
	    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear EPWM1A on event A, up count
		if(Exper==0)
		    // Set Compare values
		    EPwm1Regs.CMPA.half.CMPA = EPWM1_MAX_CMPA;     // Set compare A value
		else if(Exper==1)
			EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;          	  // Set PWM1B on Zero
			EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear PWM1B on event B, up count
			// Set Compare values
		    EPwm1Regs.CMPA.half.CMPA = EPWM1_MID_CMPA;     // Set compare A value
	}
	else if(Direction==1&&Exper==0)	//Up
	{
	    // Set actions.
		GpioDataRegs.GPACLEAR.bit.GPIO10=1;			  //CCW
	    // Set Compare values
	    EPwm1Regs.CMPA.half.CMPA = EPWM1_MAX_CMPA;     // Set compare A value
	    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Clear EPWM1A on Zero
	    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear EPWM1A on event A, up count
	}
	else	//Stop or other Conditions
	{
	    // Set actions. Both low level
	    EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;          // Clear EPWM1A on Zero
	    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear EPWM1A on event A, up count
	    EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;          	  // Clear PWM1B on Zero
	    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear PWM1B on event B, up count
	}
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:Scanning Key             	   	*/
/*------------------------------------------*/
void Scan_Key(void)
{
	if(KEY_1==0)//Scanning the Preset Key
	{
	  Delay();
	  if(KEY_1==0)
	  {
		  while(!KEY_1)
			  Delay();
		  Direction=0;
		  Exper=0;
		  GpioDataRegs.GPCSET.bit.GPIO82=1;
		  GpioDataRegs.GPCSET.bit.GPIO84=1;
		  GpioDataRegs.GPCCLEAR.bit.GPIO86=1;
		  Delay();
	  }
	}
	else if(KEY_2==0)//Scanning the Experiment Key
	{
	  Delay();
	  if(KEY_2==0)
	  {
		  while(!KEY_2)
			  Delay();
		  Direction=0;
		  Exper=1;
		  GpioDataRegs.GPCSET.bit.GPIO82=1;
		  GpioDataRegs.GPCCLEAR.bit.GPIO84=1;
		  GpioDataRegs.GPCCLEAR.bit.GPIO86=1;
		  Delay();
	  }
	}
	else if(KEY_3==0)//Scanning the Reverse Key
	{
	  Delay();
	  if(KEY_3==0)
	  {
		  while(!KEY_3)
			  Delay();
		  Direction=1;
		  Exper=0;
		  GpioDataRegs.GPCSET.bit.GPIO86=1;
		  GpioDataRegs.GPCSET.bit.GPIO84=1;
		  GpioDataRegs.GPCCLEAR.bit.GPIO82=1;
		  Delay();
	  }
	}
	else if(KEY_4==0)//Scanning the Stop Key
	{
		Delay();
		if(KEY_4==0)
		{
			while(!KEY_4)
				Delay();
			Direction=2;
			Discharge=0;
			Exper=0;
			GpioDataRegs.GPCSET.bit.GPIO86=1;
			GpioDataRegs.GPCSET.bit.GPIO84=1;
			GpioDataRegs.GPCSET.bit.GPIO82=1;
			GpioDataRegs.GPCSET.bit.GPIO80=1;
			Delay();
		}
	}
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:EPwm1 interrupt service routine */
/*------------------------------------------*/
interrupt void epwm1_isr(void)
{
	if((KEY_1==0)|(KEY_2==0)|(KEY_3==0)|(KEY_4==0))	//Scanning Keys
    {
	   Scan_Key();
	   Run();
    }
//	else if(Discharge==1)
//		Run();

    // Clear the flag of Timer interrupt
    EPwm1Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:Timer0 interrupt service routine	*/
/*------------------------------------------*/
interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;

   EALLOW;

   Discharge=0;
   if(timer_int_cnt++ >= 6)		//LED blinks 6 times when discharge
   {
       timer_int_cnt=0;
       CpuTimer0Regs.TCR.all=0x0001;           // Configure TIE = 0. Disable the Tiemr0 interrupt
   }
   GpioDataRegs.GPCTOGGLE.bit.GPIO80=1;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

   EDIS;
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:XINT1 interrupt service routine 	*/
/*------------------------------------------*/
interrupt void xint1_isr(void)
{
	if(Exper==1)
	{
		Exper=0;
	    Direction=1;
	    Discharge=1;
	    GpioDataRegs.GPCSET.bit.GPIO86=1;
	    GpioDataRegs.GPCSET.bit.GPIO84=1;
	    GpioDataRegs.GPCCLEAR.bit.GPIO82=1;

	    // Reload all counter register with period value:
	    CpuTimer0Regs.TCR.bit.TRB = 1;	//Reload Timer0
	    CpuTimer0Regs.TCR.all=0x4001;	//Configure TIE = 1. Enable Timer0 interrupt

	    Run();
	}

	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:初始化Epwm1寄存器             	    	*/
/*------------------------------------------*/
void InitEPwm1()
{

  // Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = (EPWM1_TIMER_TBPRD)/DF;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT/4=37.5MHZ;
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;	  // PWM1 freq = 10KHZ；

   // Setup shadow register load on ZERO
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = EPWM1_MAX_CMPA;     // Set compare A value
   EPwm1Regs.CMPB = (EPWM1_MAX_CMPB)/DG;               // Set compare B value

   // Set actions (motionless)
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;            // Clear PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Clear PWM1A on event A, up count

   EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;          	  // Clear PWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear PWM1B on event B, up count

   // Interrupt where we will change the Compare Values(100us*3)
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event


}



/*------------------------------------------*/
/*形式参数：void		            		*/
/*返回值:void				    			*/
/*函数描述:GPIO function select  	    	*/
/*------------------------------------------*/
void Gpio_select(void)
{


    EALLOW;

/*  GPIO Control Registers Configuration	*/
    GpioCtrlRegs.GPCMUX2.bit.GPIO86=0;	//LED ports
    GpioCtrlRegs.GPCMUX2.bit.GPIO84=0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO82=0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO80=0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO72=0;	//Key ports
    GpioCtrlRegs.GPCMUX1.bit.GPIO74=0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO72=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12=0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO78=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10=0;	//Motor Direction

/*	GPIO Inner Latch Configuration		*/
	GpioCtrlRegs.GPCPUD.bit.GPIO76=0;	//Allow Key ports to latch
	GpioCtrlRegs.GPCPUD.bit.GPIO74=0;
	GpioCtrlRegs.GPCPUD.bit.GPIO72=0;
	GpioCtrlRegs.GPCPUD.bit.GPIO86=0;	//Allow LED ports to latch
	GpioCtrlRegs.GPCPUD.bit.GPIO84=0;
	GpioCtrlRegs.GPCPUD.bit.GPIO82=0;
	GpioCtrlRegs.GPCPUD.bit.GPIO80=0;
    GpioCtrlRegs.GPAPUD.bit.GPIO12=1;   //Disallow GPIO12,GPIO78 to latch
    GpioCtrlRegs.GPCPUD.bit.GPIO78=1;

/*	Clear Input Data	*/
	GpioDataRegs.GPCDAT.bit.GPIO76=1;
	GpioDataRegs.GPCDAT.bit.GPIO74=1;
	GpioDataRegs.GPCDAT.bit.GPIO72=1;
	GpioDataRegs.GPADAT.bit.GPIO12=1;
	GpioDataRegs.GPCDAT.bit.GPIO78=1;

/*	GPIO Input/Output Configuration	*/
    GpioCtrlRegs.GPCDIR.bit.GPIO86=1;	//Configure GPIO Output for LED
    GpioCtrlRegs.GPCDIR.bit.GPIO84=1;
    GpioCtrlRegs.GPCDIR.bit.GPIO82=1;
    GpioCtrlRegs.GPCDIR.bit.GPIO80=1;
    GpioCtrlRegs.GPCDIR.bit.GPIO76=0;	//Configure GPIO Input for Keys
    GpioCtrlRegs.GPCDIR.bit.GPIO74=0;
    GpioCtrlRegs.GPCDIR.bit.GPIO72=0;
    GpioCtrlRegs.GPADIR.bit.GPIO12=0;
    GpioCtrlRegs.GPCDIR.bit.GPIO78=0;
    GpioCtrlRegs.GPADIR.bit.GPIO10=1;	//Configure GPIO Output for Motor Direction

/*	Initialize LED	*/
    GpioDataRegs.GPCSET.bit.GPIO86=1;	//GPIO high level. Shut off LED
    GpioDataRegs.GPCSET.bit.GPIO84=1;
    GpioDataRegs.GPCSET.bit.GPIO82=1;
    GpioDataRegs.GPCSET.bit.GPIO80=1;

/*	Sync EPwm1 Clock	*/
    GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // Sync GPIO0-GPIO15 with SYSCLKOUT

/*	Configure XINT1 Port		*/
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 0x0C;   // XINT1 is GPIO12

    EDIS;

}
//===========================================================================
// No more.
//===========================================================================


