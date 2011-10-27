#include "Piccolo_PWM.h"

void InitEPwm1()
{
	EPwm1Regs.TBPRD = INITIAL_PERIOD;              // Set timer period
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Setup compare
	EPwm1Regs.CMPA.half.CMPA = 0;
	
	   // Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on Zero
	EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;
	
	
	EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;          // Set PWM1A on Zero
	EPwm1Regs.AQCTLB.bit.PRD = AQ_CLEAR;

   // Active Low PWMs - Setup Deadband
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm1Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm1Regs.DBRED = DB_RED;
	EPwm1Regs.DBFED = DB_FED;
}

void InitEPwm2()
{
	EPwm2Regs.TBPRD = INITIAL_PERIOD;              // Set timer period
	EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Setup compare
	EPwm2Regs.CMPA.half.CMPA = 0;
	
	   // Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on Zero
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	
	
	EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
	EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active Low PWMs - Setup Deadband
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm2Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm2Regs.DBRED = DB_RED;
	EPwm2Regs.DBFED = DB_FED;
}

void InitEPwm3()
{
	EPwm3Regs.TBPRD = INITIAL_PERIOD;              // Set timer period
	EPwm3Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm3Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Setup compare
	EPwm3Regs.CMPA.half.CMPA = 0;
	
	   // Set actions
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on Zero
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	
	
	EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
	EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active Low PWMs - Setup Deadband
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm3Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm3Regs.DBRED = DB_RED;
	EPwm3Regs.DBFED = DB_FED;
}

void InitEPwm4()
{
	EPwm4Regs.TBPRD = INITIAL_PERIOD;              // Set timer period
	EPwm4Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm4Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Setup compare
	EPwm4Regs.CMPA.half.CMPA = 0;
	
	   // Set actions
	EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on Zero
	EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	
	
	EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
	EPwm4Regs.AQCTLB.bit.CAD = AQ_SET;

   // Active Low PWMs - Setup Deadband
	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm4Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm4Regs.DBRED = DB_RED;
	EPwm4Regs.DBFED = DB_FED;
}

void InitEPwm1Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

/* Configure EPwm-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

void InitEPwm2Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)

/* Configure EPwm-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

void InitEPwm3Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)

/* Configure EPwm-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}

void InitEPwm4Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)

/* Configure EPWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM4 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}
