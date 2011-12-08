#include "DSP2802x_Device.h"
#include "DSP2802x_Examples.h"
#include "easy2802x_sci_v7.3.h"
#include "DSP2802x_GlobalPrototypes.h"
#include "Piccolo_PWM.h"
#include "IQmathLib.h"

#define VIN_SCALE_INIT	430		//Scaling for Q15 Format
#define VIN_OFFSET_INIT 10
#define VBUS_SCALE_INIT	667		//Scaling for Q15 Format
#define	I_SCALE_INIT	147		//Scaling for Q15 Format
#define I_OFFSET_INIT		114

#define GLOBAL_Q	15

//#define v_b0 2418
//#define v_b1 -4190
//#define v_b2 1814

#define v_b0 256
#define v_b1 -444
#define v_b2 230

#define INITIAL_DEADTIME_AFTER_Q1_OFF	0
#define INITIAL_DEADTIME_AFTER_Q2_OFF	1
#define INITIAL_DEADTIME_AFTER_Q1_OFF_HI_RES	30
#define INITIAL_DEADTIME_AFTER_Q2_OFF_HI_RES	30
#define INITIAL_EARLY_TURN_ON_Q2	2

#define MIN_OPERATING_VOLTAGE 491520 //Minimum Operating Voltage of 15V
#define MIN_STARTUP_VOLTAGE 655360 //Minimum Startup Voltage of 20V
#define MAX_OPERATING_VOLTAGE 1474560 //Maximum Operating Voltage of 45V

#define MPPT_UPDATE_PERIOD_MS	500
#define MAX_POWER_SAMPLES		100
#define INITIAL_POWER_SAMPLES	10

#define OUT_MAX 0x599A
#define DELAY_MAX 45876 //(+1.4)
#define DELAY_MIN -45876 //(-1.4)

#pragma CODE_SECTION(pwm_int, "ramfuncs");
#pragma CODE_SECTION(mppt_int, "ramfuncs");
#pragma CODE_SECTION(__IQmpy, "ramfuncs");
#pragma CODE_SECTION(_IQ15int, "ramfuncs");
#pragma CODE_SECTION(_IQ15frac, "ramfuncs");
#pragma CODE_SECTION(__IQsat, "ramfuncs");
#pragma CODE_SECTION(_IQ15div, "ramfuncs");

//ADC Scaling/Offset Variables
unsigned int VIN_SCALE;
unsigned int VBUS_SCALE;
unsigned int I_SCALE;
unsigned int I_OFFSET;
unsigned int VIN_OFFSET;
int input_voltage_prescale;

//MPPT Variables
unsigned int step_direction;
unsigned int startup_flag;
long int Min_Startup_Voltage_Q15;
long int Min_Operating_Voltage_Q15;
long int Max_Operating_Voltage_Q15;
long int Previous_Power_Q15;
long int MPPT_Step_Size_Q15;
long int Input_Power_Q15;
int step_dir;
int input_current_prescale;

//System Status Variables
unsigned int Power_Good;
unsigned int Output_Over_Voltage;

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

unsigned int deadtime_after_Q1_off;
unsigned int deadtime_after_Q2_off;
unsigned int deadtime_after_Q1_off_hi_res;
unsigned int deadtime_after_Q2_off_hi_res;
unsigned int early_turn_on_Q2;

long int Power_Samples_Q15[MAX_POWER_SAMPLES];
unsigned int Num_Power_Samples;
unsigned int Power_Sample_Counter;
long int Power_Sample_Sum_Q15;

unsigned int i;

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
	InitEPwm2();
	InitEPwm3();
	pwm_setup();
	initialize_mppt_timer();
	EALLOW;
	PieVectTable.TINT2 = &mppt_int;
	PieVectTable.EPWM2_INT = &pwm_int;
	PieCtrlRegs.PIEIER3.bit.INTx2 = 0x1;
	EDIS;
	IER |= M_INT3;
	IER |= M_INT14;
	EINT;
	ms_delay(1);
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	ERTM;
	for(;;)
	{
////		Bus_Voltage_Q15 = ((long int) AdcResult.ADCRESULT1*VBUS_SCALE);

	}
}

interrupt void mppt_int()
{
	GpioDataRegs.GPASET.bit.GPIO3 = 1;
	Power_Good = GpioDataRegs.GPADAT.bit.GPIO16;
	Output_Over_Voltage = GpioDataRegs.GPADAT.bit.GPIO17;
	Power_Sample_Sum_Q15 = 0;
	for (i = 0; i < Num_Power_Samples; i++)
	{
		Power_Sample_Sum_Q15 += Power_Samples_Q15[i];
	}
	Input_Power_Q15 = _IQ15div(Power_Sample_Sum_Q15, (Num_Power_Samples << 15));
	if (!startup_flag && Input_Voltage_Q15 > Min_Startup_Voltage_Q15)
	{
		Vin_reference_Q15 = Input_Voltage_Q15 - MPPT_Step_Size_Q15;
		step_dir = 0;
		startup_flag = 1;
	}
	else if (Input_Voltage_Q15 <= Min_Operating_Voltage_Q15)
	{
		Vin_reference_Q15 = Min_Operating_Voltage_Q15 + MPPT_Step_Size_Q15;
		step_direction = 1;
	}
	else if (Input_Voltage_Q15 >= Max_Operating_Voltage_Q15)
	{
		Vin_reference_Q15 = Max_Operating_Voltage_Q15 - MPPT_Step_Size_Q15;
		step_direction = 0;
	}
	else
	{
		if (startup_flag)
		{
			if (Input_Power_Q15 > Previous_Power_Q15)
			{
				if (step_direction)
				{
					Vin_reference_Q15 += MPPT_Step_Size_Q15;
				}
				else
				{
					Vin_reference_Q15 -= MPPT_Step_Size_Q15;
				}
			}
			else
			{
				if (step_direction)
				{
					step_direction = !step_direction;
					Vin_reference_Q15 -= MPPT_Step_Size_Q15;
				}
				else
				{
					step_direction = !step_direction;
					Vin_reference_Q15 += MPPT_Step_Size_Q15;
				}
			}
		}
		else
		{
			Vin_reference_Q15 = 3276800; //100V
		}
	}
	Previous_Power_Q15 = Input_Power_Q15;
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	return;
}

interrupt void pwm_int()
{
	DINT;
	//GpioDataRegs.GPASET.bit.GPIO3 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC0 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC1 = 1;
	AdcRegs.ADCSOCFRC1.bit.SOC2 = 1;
	input_voltage_prescale = ((int) AdcResult.ADCRESULT0 - VIN_OFFSET);
	Input_Voltage_Q15 = ((long int) input_voltage_prescale*VIN_SCALE);
	input_current_prescale = ((int) AdcResult.ADCRESULT2 - I_OFFSET);
	Input_Current_Q15 = ((long int) (input_current_prescale )*I_SCALE);

	if(Power_Sample_Counter >= Num_Power_Samples)
	{
		Power_Sample_Counter = 0;
	}
	Power_Samples_Q15[Power_Sample_Counter] = _IQmpy(Input_Voltage_Q15, Input_Current_Q15);
	Power_Sample_Counter++;

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
	//duty = ((unsigned int) duty_output);

	EPwm2Regs.CMPA.half.CMPA = duty - deadtime_after_Q1_off;
	EPwm2Regs.CMPA.half.CMPAHR = deadtime_after_Q1_off_hi_res << 8;

	if (duty <= early_turn_on_Q2)
	{
		duty = early_turn_on_Q2;
	}
	EPwm3Regs.CMPB = duty + early_turn_on_Q2;
	EPwm3Regs.CMPA.half.CMPA = deadtime_after_Q2_off;
	EPwm3Regs.CMPA.half.CMPAHR = deadtime_after_Q2_off_hi_res << 8;
	//GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	EINT;
	EPwm2Regs.ETCLR.bit.INT = 0x1;  			//Clear the Interrupt Flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  	//Acknowledge the interrupt
	return;
}

void pwm_setup()
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
	EDIS;

	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;
	EDIS;

	EPwm2Regs.ETSEL.bit.INTEN = 0x1;
	EPwm2Regs.ETSEL.bit.INTSEL = 0x1;
	EPwm2Regs.ETPS.bit.INTPRD = 0x1;
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
	CpuTimer2Regs.TPRH.all = 0x00EA;
	CpuTimer2Regs.TPR.all = 0x0060;
	CpuTimer2Regs.PRD.all = MPPT_UPDATE_PERIOD_MS;
	CpuTimer2Regs.TCR.bit.TRB = 1;
	CpuTimer2Regs.TCR.all = 0x4000; //Initialize to Interrupt Enabled and Free-Running
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
	VIN_SCALE = VIN_SCALE_INIT;
	VBUS_SCALE = VBUS_SCALE_INIT;
	I_SCALE = I_SCALE_INIT;
	I_OFFSET = I_OFFSET_INIT;
	control_gain = 0x8000;
	v_temporary = 0;
	Vin_reference_Q15 = 0x8000;
	Vin_reference_Q15 = 100*Vin_reference_Q15;
	Min_Startup_Voltage_Q15 = MIN_STARTUP_VOLTAGE;
	Min_Operating_Voltage_Q15 = MIN_OPERATING_VOLTAGE;
	Max_Operating_Voltage_Q15 = MAX_OPERATING_VOLTAGE;
	startup_flag = 0;
	step_dir = 0;
	input_current_prescale = 0;
	VIN_OFFSET = VIN_OFFSET_INIT;
	input_voltage_prescale = 0;
	Power_Good = 0;
	Output_Over_Voltage = 1;
	deadtime_after_Q1_off = INITIAL_DEADTIME_AFTER_Q1_OFF;
	deadtime_after_Q2_off = INITIAL_DEADTIME_AFTER_Q2_OFF;
	deadtime_after_Q1_off_hi_res = INITIAL_DEADTIME_AFTER_Q1_OFF_HI_RES;
	deadtime_after_Q2_off_hi_res = INITIAL_DEADTIME_AFTER_Q2_OFF_HI_RES;
	early_turn_on_Q2 = INITIAL_EARLY_TURN_ON_Q2;
	duty = INITIAL_EARLY_TURN_ON_Q2;
	Num_Power_Samples = INITIAL_POWER_SAMPLES;
	for (i = 0; i < MAX_POWER_SAMPLES; i++)
	{
		Power_Samples_Q15[i] = 0;
	}
	Power_Sample_Counter = 0;
	Power_Sample_Sum_Q15 = 0;
}

