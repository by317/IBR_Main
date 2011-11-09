#include "DSP2802x_Device.h"
#include "DSP2802x_Examples.h"
#include "easy2802x_sci_v7.3.h"
#include "DSP2802x_GlobalPrototypes.h"
#include "Piccolo_PWM.h"

//#define VIN_SCALE	508		//Scaling for Q15 Format
//#define VBUS_SCALE	508		//Scaling for Q15 Format
//#define	I_SCALE		150		//Scaling for Q15 Format

//#define v_b0 2418
//#define v_b1 -4190
//#define v_b2 1814

#define v_b0 256
#define v_b1 -444
#define v_b2 230

#define MIN_OPERATING_VOLTAGE 491520; //Minimum Operating Voltage of 15V
#define MIN_STARTUP_VOLTAGE 655360 //Minimum Startup Voltage of 20V

#define OUT_MAX 0x599A
#define DELAY_MAX 45876 //(+1.4)
#define DELAY_MIN -45876 //(-1.4)

#pragma CODE_SECTION(pwm_int, "ramfuncs");
#pragma CODE_SECTION(mppt_int, "ramfuncs");


//ADC Scaling/Offset Variables
unsigned int VIN_SCALE;
unsigned int VBUS_SCALE;
unsigned int I_SCALE;
unsigned int I_OFFSET;

//MPPT Variables
unsigned int step_direction;
unsigned int startup_flag;
long int Min_Startup_Voltage_Q15;
long int Min_Operating_Voltage_Q15;
long int Previous_Power_Q15;
long int MPPT_Step_Size_Q15

interrupt void pwm_int(void);
interrupt void mppt_int(void);

void pwm_setup(void);

unsigned int x;
volatile unsigned int duty;
unsigned int period;
unsigned int overlap;
unsigned int rising_edge_delay;
unsigned int falling_edge_delay;
volatile unsigned int first_run;
int y;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

volatile long int duty_output;
volatile long int err_delay1;
volatile long int err_delay2;
volatile long int out_delay1;
volatile long int Vin_reference_Q15;
volatile long long int v_comp_out;
volatile long int Vin_err_Q15;
volatile long int control_gain;
volatile long long int v_temporary;

int32 Input_Voltage_Q15;
int32 Bus_Voltage_Q15;
int32 Input_Current_Q15;
extern void DSP28x_usDelay(Uint32);
void ms_delay(unsigned int);
void SetupAdc(void);
void initVariables(void);
void initialize_mppt_timer(void);

void main()
{
	initVariables();
	
	DINT;
	ms_delay(1);
	InitAdc();
	SetupAdc();
	InitSysCtrl();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitPieVectTable();
	easyDSP_SCI_Init();
	InitEPwm1();
	pwm_setup();
	InitEPwm1Gpio();
	EALLOW;
	PieVectTable.TINT2 = &mppt_int;
	PieVectTable.EPWM1_INT = &pwm_int;
	PieCtrlRegs.PIEIER3.bit.INTx1 = 0x1;
	EDIS;
	IER |= M_INT3;
	IER |= M_INT14;
	EINT;
	ERTM;
//	for(;;)
//	{
////		EPwm1Regs.DBRED = rising_edge_delay;
////		EPwm1Regs.DBFED = falling_edge_delay;
////		EPwm1Regs.TBPRD = period;
////
////		Bus_Voltage_Q15 = ((long int) AdcResult.ADCRESULT1*VBUS_SCALE);
////		Input_Current_Q15 = ((long int) AdcResult.ADCRESULT2*I_SCALE);
//	}
}

interrupt void mppt_int()
{
	Input_Current_Q15 = ((long int) (AdcResult.ADCRESULT2 - I_OFFSET )*I_SCALE);
	Input_Power_Q15 = ((long long int) Input_Voltage_Q15*Input_Current_Q15 >> 15);
	if (!startup_flag && Input_Voltage_Q15 > Min_Startup_Voltage_Q15)
	{
		Vin_reference_Q15 = Input_Voltage_Q15 - MPPT_Step_Size;
		step_dir = 0;
		startup_flag = 1;
	}
	else if (Input_Voltage_Q15 <= Min_Operating_Voltage)
	{
		Vin_reference_Q15 = Min_Operating_Voltage + MPPT_Step_Size;
		step_direction = 1;
	}
	else if (Input_Voltage_Q15 >= Max_Operating_Voltage)
	{
		Vin_reference_Q15 = Max_Operating_Voltage - MPPT_Step_Size;
		step_direction = 0;
	}
	else if (startup_flag)
	{
		if (Input_Power_Q15 > Previous_Power_Q15)
		{
			if (step_direction)
			{
				Vin_reference_Q15 += MPPT_Step_Size;
			}
			else
			{
				Vin_reference_Q15 -= MPPT_Step_Size;
			}
		}
		else
		{
			if (step_direction)
			{
				step_direction = !step_direction;
				Vin_reference_Q15 -= MPPT_Step_Size;
			}
			else
			{
				step_direction = !step_direction;
				Vin_reference_Q15 += MPPT_Step_Size;
			}
		}
	}
	else
	{
		Vin_reference_Q15 = 3276800; //100V
	}
	Previous_Power_Q15 = Input_Power_Q15;
}

interrupt void pwm_int()
{
	DINT;
	GpioDataRegs.GPASET.bit.GPIO3 = 1;
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

	v_temporary = (Vin_err_Q15*v_b0) >> 15;
	v_temporary = v_temporary + ((err_delay1*v_b1)>>15);
	v_temporary = v_temporary + ((err_delay2*v_b2)>>15);
	v_temporary = v_temporary + (out_delay1);
	v_temporary = v_temporary;

	v_comp_out = v_temporary;

	out_delay1 = v_comp_out;

	if (v_comp_out < 0)
	{
		v_comp_out = 0;
	}
	else if (v_comp_out > OUT_MAX)
	{
		v_comp_out = OUT_MAX;
	}
	if (out_delay1 < DELAY_MIN)
	{
		out_delay1 = DELAY_MIN;
	}
	else if (out_delay1 > DELAY_MAX)
	{
		out_delay1 = DELAY_MAX;
	}


	err_delay2 = err_delay1;
	err_delay1 = Vin_err_Q15;

	duty_output = ((long long int) v_comp_out >> 5);
	duty = ((unsigned int) duty_output);

	EPwm1Regs.CMPA.half.CMPA = duty;
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	EINT;
	EPwm1Regs.ETCLR.bit.INT = 0x1;  			//Clear the Interrupt Flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  	//Acknowledge the interrupt
	return;
}

void pwm_setup()
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
	EDIS;

	EPwm1Regs.ETSEL.bit.INTEN = 0x1;
	EPwm1Regs.ETSEL.bit.INTSEL = 0x1;
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

void initialize_mppt_timer(void)
{
	//Load the pre-scaler to 60000 clock cycles (1ms)
	CpuTimer2Regs.TPR.bit.TDDR = 0xEA60;
}

void initVariables (void)
{
	rising_edge_delay = DB_RED;
	falling_edge_delay = DB_FED;
	overlap = 0;
	duty = 0;
	period = 1000;
	first_run = 1;
	Input_Voltage_Q15 = 0;
	Bus_Voltage_Q15 = 0;
	Input_Current_Q15 = 0;
	VIN_SCALE = 508;
	VBUS_SCALE = 667;
	I_SCALE = 150;
	I_OFFSET = 0;
	control_gain = 0x8000;
	v_temporary = 0;
	Vin_reference_Q15 = 0x8000;
	Vin_reference_Q15 = 100*Vin_reference_Q15;
	Min_Startup_Voltage_Q15 = MIN_STARTUP_VOLTAGE;
	Min_Operating_Voltage_Q15 = MIN_OPERATING_VOLTAGE;
	startup_flag = 0;
}

