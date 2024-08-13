/*boost close-loop code by ee.rohan*/

#include "F28x_Project.h"
#include "math.h"
float sampling_time = 100e-6;
float v_act = 0, v_ref = 24, v_error, duty, duty_pre = 0, v_error_pre = 0;
float kp = 100, ki = 100, b0, b1;
extern interrupt void epwm1_isr(void);
void Gpio_select(void);
void epwm1_int_setup(void);
void setup_epwm(void);
//void initdaca(void);
void configureADC(void);
void setupADC(void);
void main(void){
    InitSysCtrl();
    DINT;
    Gpio_select();
    setup_epwm();
    epwm1_int_setup();
//    initdaca();
    configureADC();
    setupADC();
    b0 = (2*kp + ki*sampling_time)/2;
    b1 = (ki*sampling_time - 2*kp)/2;
    while(1){
    }
}

void Gpio_select(void){
    EALLOW;
    //Enable pin gpio 0 as epwm 1a
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    EDIS;
}

void setup_epwm(void){
    EPwm1Regs.TBCTL.bit.CLKDIV = 1;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBPRD = 2500;
    EPwm1Regs.CMPA.bit.CMPA = 1250;
    EPwm1Regs.AQCTLA.all = 0x0012;
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;
    EPwm1Regs.ETCLR.bit.SOCA = 1;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETSEL.bit.INTSEL = 1;
    EPwm1Regs.ETPS.bit.INTPRD = 1;
    EPwm1Regs.ETCLR.bit.INT = 1;

    EPwm2Regs.TBCTL.bit.CLKDIV = 1;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;
    EPwm2Regs.TBPRD = 5000;
    EPwm2Regs.AQCTLA.all = 0x0090;
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm2Regs.DBCTL.bit.POLSEL = 2;
    EPwm2Regs.DBFED.all = 50;
    EPwm2Regs.DBRED.all = 50;
}

void epwm1_int_setup(void){
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;
    IER |= M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    EINT;
    ERTM;
}

//void initdaca(void){
//    EALLOW;
//    DacaRegs.DACCTL.bit.DACREFSEL = 1;
//    DacaRegs.DACCTL.bit.LOADMODE = 0;
//    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
//    DacaRegs.DACVALS.bit.DACVALS = dac1;
//    DELAY_US(10);
////    DacbRegs.DACCTL.bit.DACREFSEL = 1;
////    DacbRegs.DACCTL.bit.LOADMODE = 0;
////    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
////    DacbRegs.DACVALS.bit.DACVALS = dac2;
////    DELAY_US(10);
//    EDIS;
//}
void configureADC(void){
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;
//    DELAY_US(1000);
}
void setupADC(void){
    Uint16 acqps;
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;
    }
    else{
        acqps = 63;
    }
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    EDIS;
}
extern interrupt void epwm1_isr(void){
//    EPwm2Regs.CMPA.bit.CMPA = 2500*(AdcaResultRegs.ADCRESULT0)/4095;
    v_act = 36.0*AdcaResultRegs.ADCRESULT0/4095.0;
    v_error = v_ref - v_act;
    duty = duty_pre + b0*v_error + b1*v_error_pre;
    if(duty > 4000){
            duty = 4000;
        }
    if(duty < 0){
        duty = 0;
    }
    duty_pre = duty;
    v_error_pre = v_error;
    EPwm2Regs.CMPA.bit.CMPA = duty;
//    DacaRegs.DACVALS.bit.DACVALS = dac1;
//    DacbRegs.DACVALS.bit.DACVALS = dac2;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    EPwm1Regs.ETCLR.bit.INT = 1;
}
