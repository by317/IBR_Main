#include "DSP2802x_Device.h"
#include "DSP2802x_Examples.h"
#include "easy2802x_sci_v7.3.h"
#include "DSP2802x_GlobalPrototypes.h"
#include "Piccolo_PWM.h"


//#define VIN_SCALE	508		//Scaling for Q15 Format
//#define VBUS_SCALE	508		//Scaling for Q15 Format
//#define	I_SCALE		150		//Scaling for Q15 Format

#define v_b0 2418
#define v_b1 -4190
#define v_b2 1814

unsigned int VIN_SCALE;
unsigned int VBUS_SCALE;
unsigned int I_SCALE;

interrupt void pwm_int(void);
void pwm_setup(void);

unsigned int x;
unsigned int duty;
unsigned int period;
unsigned int overlap;
unsigned int rising_edge_delay;
unsigned int falling_edge_delay;
unsigned int first_run;
int y;

long int OUT_MAX;
long int duty_output;
long int err_delay1;
long int err_delay2;
long int out_delay1;
long int vcomp_out;
long int Vin_reference_Q15;
long long int v_comp_out;
long int Vin_err_Q15;
long int control_gain;

int32 Input_Voltage_Q15;
int32 Bus_Voltage_Q15;
int32 Input_Current_Q15;
extern void DSP28x_usDelay(Uint32);
void ms_delay(unsigned int);
void SetupAdc(void);
void initVariables(void);

void main()
{
	initVariables();
	
	rising_edge_delay = DB_RED;
	falling_edge_delay = DB_FED;
	overlap = 0;
	duty = 500;
	period = 1000;
	DINT;
	ms_delay(1);
	InitAdc();
	SetupAdc();
	InitSysCtrl();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();
	easyDSP_SCI_Init();
	InitEPwm1();
	pwm_setup();
	InitEPwm1Gpio();
	EALLOW;
	PieVectTable.EPWM1_INT = &pwm_int;
	PieCtrlRegs.PIEIER3.bit.INTx1 = 0x1;
	EDIS;
	IER |= M_INT3;
	EINT;
	ERTM;
	for(;;)
	{
		EPwm1Regs.DBRED = rising_edge_delay;
		EPwm1Regs.DBFED = falling_edge_delay;
		EPwm1Regs.TBPRD = period;

		Bus_Voltage_Q15 = ((long int) AdcResult.ADCRESULT1*VBUS_SCALE);
		Input_Current_Q15 = ((long int) AdcResult.ADCRESULT2*I_SCALE);
	}
}

interrupt void pwm_int()
{
	AdcRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC2 = 1;
	Input_Voltage_Q15 = ((long int) AdcResult.ADCRESULT0*VIN_SCALE);
	Vin_err_Q15 = Input_Voltage_Q15 - Vin_reference_Q15;
	if (first_run)
	{
		err_delay1 = Vin_err_Q15;
		err_delay2 = Vin_err_Q15;
		out_delay1 = 0;
		first_run = 0;
	}
	v_comp_out = ((long long int) Vin_err_Q15*v_b0 + err_delay1*v_b1 + err_delay2*v_b2 + out_delay1)>>15;

	if (v_comp_out < 0)
	{
		v_comp_out = 0;
	}
	else if (v_comp_out > OUT_MAX)
	{
		v_comp_out = OUT_MAX;
	}
	out_delay1 = v_comp_out;
	err_delay2 = err_delay1;
	err_delay1 = Vin_err_Q15;

	duty_output = ((long long int) v_comp_out*control_gain >> 20);
	duty = ((unsigned int) duty_output);

	EPwm1Regs.CMPA.half.CMPA = duty;
	EPwm1Regs.ETCLR.bit.INT = 0x1;  			//Clear the Interrupt Flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  	//Acknowledge the interrupt
	return;
}

void pwm_setup()
{
	EPwm1Regs.ETSEL.bit.INTEN = 0x1;
	EPwm1Regs.ETSEL.bit.INTSEL = 0x2;
	EPwm1Regs.ETPS.bit.INTPRD = 0x1;	
}

void ms_delay(unsigned int wait_time)
{
	volatile unsigned int i;
	CpuTimer1Regs.PRD.all = 0x0000EA60;
	for (i=0; i < wait_time; i++)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;
		CpuTimer1Regs.TCR.bit.TRB = 1;
		CpuTimer1Regs.TCR.bit.TSS = 0;
		while (CpuTimer1Regs.TCR.bit.TIF == 0)
		{	
		}
	}
}

void InitAdc(void)
{
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2802x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    EDIS;

    ms_delay(10);         // Delay before converting ADC channels
}

void SetupAdc(void)
{
	EALLOW;
	//Input Voltage Sampling on SOC0
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x00;
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x2;
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 0x6;	
	//Output Voltage Sampling on SOC1
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x00;
	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x0;
	AdcRegs.ADCSOC1CTL.bit.ACQPS = 0x6;	
	//Input Current Sampling on SOC4
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x00;
	AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x4;
	AdcRegs.ADCSOC2CTL.bit.ACQPS = 0x6;
	EDIS;
}

void initVariables (void)
{
	first_run = 1;
	Input_Voltage_Q15 = 0;
	Bus_Voltage_Q15 = 0;
	Input_Current_Q15 = 0;
	VIN_SCALE = 508;
	VBUS_SCALE = 667;
	I_SCALE = 150;
	OUT_MAX = 0x599A;
	control_gain = 0x8000;
}

