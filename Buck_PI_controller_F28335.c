#include "DSP28x_Project.h"
#include "DSP2833x_Examples.h"
#include "DSP2833x_Device.h" //"DSP2802x_Device.h"
#include <stdio.h>
#include <stdlib.h>
#include "IQmathLib.h"
#include "math.h"

#if (CPU_FRQ_150MHZ)   // Default - 150 MHz SYSCLKOUT
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3) = 25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
#define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2) = 25.0 MHz
#endif
#define ADC_CKPS 0x0  // ADC module clock = HSPCLK/1 = 25.5MHz/(1) = 25.0 MHz
#define ADC_SHCLK 0x1 // S/H width in ADC module periods = 2 ADC cycle
#define AVG 1000      // Average sample limit*/
#define ZOFFSET 0x00  // Average Zero offset
#define BUF_SIZE 1024 // Sample buffer size

void InitEPwm1Example(void);
interrupt void epwm1_isr(void);
interrupt void cpu_timer0_isr(void);
void Gpio_select(void);
interrupt void cpu_timer0_isr(void);
Uint32 EPwm1TimerIntCount;
Uint16 EPwm1_DB_Direction;

float Voltage_VR3, IL;
float Voltage_VR4, Vo;
float Voltage_VR5, Vdc;

// PI Paramer//
float kii = 405.0643, kpi = 1.1282, kiv = 2.5266, kpv =-0.0296;

float err_v,Upv,Uiv,Uiv_1=0,Upi_v,Ts_PI=1.65e-5;
float err_i,Upi,Uii,Uii_1=0,Upi_i;
float Vo, IL, setpoint;
float Ueq;
float count;

#define EPWM1_MAX_DB 0x03FF
#define EPWM1_MIN_DB 0
#define DB_UP 1
#define DB_DOWN 0

void main(void)
{
    InitSysCtrl();
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;
    Gpio_select();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    EDIS;
    InitAdc();
    AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; // 1 Cascaded mode
    AdcRegs.ADCTRL1.bit.CONT_RUN = 1; // Setup continuous run
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1; // Enable Sequencer override feature
    AdcRegs.ADCTRL2.all = 0x2000;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0xf;
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // A0
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // A1
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // A2
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; // A3
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; // A4
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5; // A5
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6; // A6
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7; // A7
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

    InitEPwm1Example();
    DINT;

    InitCpuTimers(); // For this example, only initialize the Cpu Timers

#if (CPU_FRQ_150MHZ)
    ConfigCpuTimer(&CpuTimer0, 10, 150);
#endif

    CpuTimer0Regs.TCR.all = 0x4001;
    EINT;
    ERTM;

    for (i = 0; i <= 2; i++)
    {
        Vo_FIL_PAST = 0;
        Vdc_FIL_PAST = 0;
        IL_FIL_PAST = 0;

            //////////////////////Sensor////////////////////////////////////////////////////////////
            GpioDataRegs.GPADAT.bit.GPIO12 = !GpioDataRegs.GPADAT.bit.GPIO12;
            Voltage_VR3 = (AdcRegs.ADCRESULT2 >> 4); // IL
            IL = ((Voltage_VR3 * 3 / 4095) * (9.902)) + 0.0118;
            Voltage_VR4 = (AdcRegs.ADCRESULT3 >> 4); // V output
            Vo = ((Voltage_VR4 * 3 / 4095) * (135.32)) + 0.0739;
            Voltage_VR5 = (AdcRegs.ADCRESULT4 >> 4); // V input
            Vdc = (((Voltage_VR5 * 3 / 4095) * (136.06)) + 1.1729);

            ///////////////////////////////////////
            setpoint=20;
            // PI controller voltage loop----------->>>
            err_v = (setpoint-Vo);
            Upv = kpv*err_v;
            Uiv = (kiv*Ts_PI*err_v)+Uiv_1;
            Upi_v = Upv+Uiv;
            // PI controller current loop----------->>>
            err_i = Upi_v-IL;
            Upi = kpi*err_i;
            Uii = (kii*Ts_PI*err_i)+Uii_1;
            Upi_i = Upi+Uii;
            Ueq = Upi_i;

            if(Ueq>=1){Ueq = 1;}
            if(Ueq<=0){Ueq = 0;}

            EPwm1Regs.CMPA.half.CMPA = 1875-1875*Ueq;

            Uiv_1=Uiv;
            Uii_1=Uii;

    }
}

void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = 1875;                 //Fs = 100kHz
    EPwm1Regs.TBPHS.half.TBPHS = 0;                // Set Phase register to zero
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm1Regs.TBCTR = 0x0000;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Master module
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;   // Sync down-stream module
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
    EPwm1Regs.DBFED = 20;                     // 10 = 1usec
    EPwm1Regs.DBRED = 20;                     // 10 = 1usec
}
interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    GpioDataRegs.GPBTOGGLE.bit.GPIO32 = 1;
    EALLOW;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
void Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0x0000;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // ePWM1A active
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0; // ePWM1B active
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0; // ePWM2A active
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0; // ePWM2B active
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0; // ePWM2A active
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0; // ePWM2B active
    GpioCtrlRegs.GPBMUX1.all = 0x0000;  // GPIO functionality GPIO32-GPIO47
    GpioCtrlRegs.GPCMUX1.all = 0x0000;  // GPIO functionality GPIO64-GPIO79
    GpioCtrlRegs.GPADIR.all = 0xFFFF;
    GpioCtrlRegs.GPBDIR.all = 0xFFFF; // GPIO32-GPIO47 are output
    GpioCtrlRegs.GPCDIR.all = 0xFFFF; // GPIO64-GPIO79 are output
    EDIS;
}
