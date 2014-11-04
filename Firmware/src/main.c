/*!
 * \mainpage main.c
 * \author   Mauro Soligo -->mauro.soligo@gmail.com<--
 * \version 0.0.1
 * \date 12/2011
 * \details This is a software :)

-------------------------------------------------------------------------------

\copyright 2011 Mauro Soligo
mauro.soligo@gmail.com

RoboController is free software derived from Guido Ottaviano dsPID33
you can redistribute it and/or modify it under the terms of ......

-------------------------------------------------------------------------------
 */

unsigned char Ver[] = "RoboController Test V1.0 Mauro Soligo 2011"; // 42+1 char

// standard include
#define VAR_INC     // Solo nel Main per definire le 

#include "p33Fxxxx.h"
#include <stdio.h>
#include <dsp.h>
#include <pwm12.h>
#include <uart.h>
#include <qei.h>
#include <adc.h>
#include <timer.h>
#include <ports.h>
#include <dma.h>
#include <math.h>
#include <stdlib.h>
#include <libq.h>

#include "def.h"
#include "ptype.h"
#include "var.h"
#include "macro.h"


//unsigned int TestBreakpoint;
//float a,b,c,d;
//long int Valori[1000];
//unsigned int IndiceValori = 0;

int main(int argc, char** argv)
{
    unsigned int test;

    Settings();


   //long E1, E2; // E = [( 2π * 10^9 ) / EncoderResolution]

    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    /* Inizializzazione da EEPROM                                                                               */
    /* Recupero i dati salvati in EEPROM ed eseguo un controllo di correttezza su di essi, questo serve         */
    /* principalmente per il default dopo la programmazione                                                     */
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    InitializationEEPROM();

    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    /* Inizializzazione variabili                                                                               */
    /*                                                                                                          */
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */

    LED1 = LED_OFF;
    LED2 = LED_OFF;

    MotorControlEnable(MOTORE1, MOTOR_DEACTIVE);
    MotorControlEnable(MOTORE2, MOTOR_DEACTIVE);

    DmaBuffer = 0;

    VarModbus[INDICE_ENC1_TICK] = 0;
    VarModbus[INDICE_ENC2_TICK] = 0;
    VarModbus[INDICE_ENC1_PERIOD] = 0;
    VarModbus[INDICE_ENC2_PERIOD] = 0;
    VarModbus[INDICE_ENC1_SPEED] = 0;
    VarModbus[INDICE_ENC2_SPEED] = 0;
    //Motore1.L_WheelSpeed = 0;
    //Motore2.L_WheelSpeed = 0;

    VarModbus[INDICE_STATUSBIT1] = 0;
    //VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_JOYMODE);     // Al reset disattivo la modalità JoyStick
    //VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_PID_EN);      // Al reset disattivo la modalità PID
    VarModbus[INDICE_STATUSBIT1] |= FLG_STATUSBI1_PID_EN; // Al reset attivo la modalità PID
    VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_EEPROM_RAMP_EN); // Al reset disattivo RAMPE

    OLD_INDICE_STATUSBIT1 = 0; // Se parto con il PID abilitato deve valere 0.
    //OLD_INDICE_STATUSBIT1 = 1;  // Se parto con il PWM abilitato deve valere 1.

    VarModbus[INDICE_STATUSBIT1] |= FLG_STATUSBI1_COMWATCHDOG; // Al reset attivo il WatchDog sulla comunicazione
    //VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_COMWATCHDOG); // Al reset disattivo il WatchDog sulla comunicazione
    VarModbus[INDICE_STATUSBIT1] |= FLG_STATUSBI1_EEPROM_SAVE_EN; // Al reset attivo il Salvataggio automatico dei parametri in EEPROM
    //VarModbus[INDICE_STATUSBIT1] &= ~(FLG_STATUSBI1_EEPROM_SAVE_EN);   // Al reset disattivo il Salvataggio automatico dei parametri in EEPROM

    
    VarModbus[INDICE_FLAG_TARATURA] = 0;
    //VarModbus[INDICE_PWM_CH1] = MOTORE_STOP;                      // Motori fermi all'accensione
    //VarModbus[INDICE_PWM_CH2] = MOTORE_STOP;                      // Motori fermi all'accensione
    VarModbus[INDICE_PWM_CH1] = MOTORE_STOP_PID; // Motori fermi all'accensione
    VarModbus[INDICE_PWM_CH2] = MOTORE_STOP_PID; // Motori fermi all'accensione

    InitMotorStructure();
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */
    /* ******************************************************************************************************** */

    /* Inizializzo i timer SW */
    Timer1mSec.T_flag = FALSE;
    Timer1mSec.T_initial_value = 1; // 1 = 1mSec
    Timer1mSec.T_count_time = Timer1mSec.T_initial_value;
    Timer10mSec.T_flag = FALSE;
    Timer10mSec.T_initial_value = 10; // 10 = 10mSec
    Timer10mSec.T_count_time = Timer10mSec.T_initial_value;

    /* Inizializzazione porte seriali */
    InizializzaSeriale(PORT_COM1);
    InizializzaSeriale(PORT_COM2);

    InitADC();

    MotorControlEnable(MOTORE1, MOTOR_ACTIVE);
    MotorControlEnable(MOTORE2, MOTOR_ACTIVE);

    //  TEST SEGNALAZIONI LED, sarà da eseguire dopo il controllo dei relativi FLAG...
    SetLedErrorCode(&Led1Segnalazione, LED_ERRORCODE_05_CHECKERRORIOK, 1, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
    SetLedErrorCode(&Led2Segnalazione, LED_POWERON_05_POWERTEST, 1, SEGNALAZIONELED_TON * 2, SEGNALAZIONELED_TOFF * 2, SEGNALAZIONELED_TPAUSE);

    test = INTCON1;

    ISR_Settings(); //  Configures and enables ISRs

    //    /* DEBUG */
    //    SetpointRPM_M1 = Motore1.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)(100)));
    //    SetpointRPM_M2 = Motore2.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)(0)));
    //    if(!Motore1.UC_Fail) PID1.Setpoint = (long)(SetpointRPM_M1);
    //    if(!Motore2.UC_Fail) PID2.Setpoint = (long)(SetpointRPM_M2);
    //    /* ***** */



    /* test */
    //E1, E2; // ( 2π * 10^9 ) / EncoderResolution
    // TWOPINSEC  = 6283185307.2	// 360° ( 2 Pigreco * 10^9  per la conversione da nSec a rad/Sec )
    /*

     */
    E1 = TWOPI_decNSEC / ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT];
    E2 = TWOPI_decNSEC / ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT];
    /* **** */

    while (1)
    { // ----------------------  Gestione protocollo ModBus ---------------------- //
        ModbusRoutine(PORT_COM1);
        ModbusRoutine(PORT_COM2);

        // -----------------  PID and speed calculation every 1ms ----------------- //
        if (PID1_CALC_FLAG) Pid1(); // Ogni 1mSec ricalcola il prescaler, avvia un ciclo
        if (PID2_CALC_FLAG) Pid2(); // di lettura dell'IC e esegue il PID sul dato prec.

        // ----------------------  Task eseguito ogni 1mSec  ---------------------- //
        if (Timer1mSec.T_flag)
        {
            Timer1mSec.T_flag = FALSE;
//            GestioneWatchdog(); //  GESTIONE WATCHDOG COMUNICAZIONE
            GestioneSicurezzaMotore(); //  Gestisco situazioni di FAIL dei motori
        }

        // ----------------------  Task eseguito ogni 10mSec  ---------------------- //
        if (Timer10mSec.T_flag)
        {
            Timer10mSec.T_flag = FALSE;
            //            GestioneWatchdog();             //  GESTIONE WATCHDOG COMUNICAZIONE
            //            GestioneSicurezzaMotore();      //  Gestisco situazioni di FAIL dei motori
            GestioneLed1ErrorCode(&Led1Segnalazione);
            GestioneLed2ErrorCode(&Led2Segnalazione);
        }

        GestioneAllarmi();
        GestioneSetpoint();
        //AggiornaDatiVelocita();
        AggiornaVariabiliModbus();
    }
    return (EXIT_SUCCESS);
}

/*! \brief Questa funzione prende il dato presente nei registri WORD_PWM_CH1 e WORD_PWM_CH2
 * e lo interpreta in relazione allo stato del bit FLG_STATUSBI1_PID_EN.
 *
 * FLG_STATUSBI1_PID_EN = 0    :
 * Il dato passato alla robocontroller è un puro PWM, con 2048 il relativo motore
 * è fermo, con 0 il motore gira a massima velocità in un senso e con 4096 nel senso opposto.
 *
 * FLG_STATUSBI1_PID_EN = 1    :
 * Nelle word WORD_PWM_CH1 e WORD_PWM_CH2 passo un dato espresso come velocità in mm/Sec.
 * con 32768 la velocità è pari a 0.
 * Se voglio mandare il robot a 1m/Sec dovrò scrivere 32768+1000 = 33768
 * Se voglio mandare il robot a -1m/Sec dovrò scrivere 32768-1000 = 31768
 *
 */

/*!
  \param void
  \return void
 */
void GestioneSetpoint(void)
{
    //float Setpoint_M1, Setpoint_M2;
    long Setpoint_M1, Setpoint_M2;



    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    {
        LED2 = LED_ON;
    }
    else
    {
        LED2 = LED_OFF;
    }


    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    { //Funzionamento in modalità PID

        if (OLD_INDICE_STATUSBIT1 == 1) // Esco dalla modalità PWM e passo a quella PID
        {
            OLD_INDICE_STATUSBIT1 = 0;
            // Disattivo il PID
            PidReset(&PID1, &Motore1);
            PidReset(&PID2, &Motore2);
        }

        /* **************************************************************** */
        /* ************ SONO IN MODALITA' PID, ROUTINE "MAIN"  ************ */
        /* **************************************************************** */

        //VarModbus[INDICE_PWM_CH1] = 1000;
        // E1 = TWOPINSEC / ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_LEFT];
        // E2 = TWOPINSEC / ParametriEEPROM[EEPROM_MODBUS_ROBOT_ENCODER_CPR_RIGHT];
        Setpoint_M1 = (long)E1 / (long)VarModbus[INDICE_PWM_CH1];
        Setpoint_M2 = (long)E2 / (long)VarModbus[INDICE_PWM_CH2];

        //Setpoint_M1 = Motore1.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)VarModbus[INDICE_PWM_CH1]));
        //Setpoint_M2 = Motore2.FL_Costante_Conversione_Vlin_to_Vang * ((float)((int)VarModbus[INDICE_PWM_CH2]));
        VarModbus[INDICE_RD_PWM_CH1] = VarModbus[INDICE_PWM_CH1]; // aggiorno PWM in lettura
        VarModbus[INDICE_RD_PWM_CH2] = VarModbus[INDICE_PWM_CH2]; // aggiorno PWM in lettura

    }
    else
    {
        /* **************************************************************** */
        /* ************ SONO IN MODALITA' PWM, ROUTINE "MAIN"  ************ */
        /* **************************************************************** */

        if (OLD_INDICE_STATUSBIT1 == 0) // Esco dalla modalità PID e passo a quella PWM
        {
            OLD_INDICE_STATUSBIT1 = 1;
            // Disattivo il PID
            PidReset(&PID1, &Motore1);
            PidReset(&PID2, &Motore2);
        }

        // In modalità PWM il dato delle word INDICE_PWM_CHx lo mando direttamente 
        // al modulo PWM del micro perchè rappresenta già un PWM.
        SetDCMCPWM1(1, 2048 + (int) (VarModbus[INDICE_PWM_CH1]), 0); // setta il PWM  del motore
        SetDCMCPWM1(2, 2048 + (int) (VarModbus[INDICE_PWM_CH2]), 0); // setta il PWM  del motore
        VarModbus[INDICE_RD_PWM_CH1] = VarModbus[INDICE_PWM_CH1]; // aggiorno PWM in lettura
        VarModbus[INDICE_RD_PWM_CH2] = VarModbus[INDICE_PWM_CH2]; // aggiorno PWM in lettura
    }

    // A prescindere che sia in modalità PID o PWM chiamo sempre le funzioni PID
    // che mi servono in ogni caso per il calcolo dei dati di velocità.
    if (!Motore1.fail)
        PID1.setpoint = (long) (Setpoint_M1);
    if (!Motore2.fail)
        PID2.setpoint = (long) (Setpoint_M2);

}

/*---------------------------------------------------------------------------*/
/* Interrupt Service Routines                                                */
/*---------------------------------------------------------------------------*/
#ifndef TIMER_OFF

void _ISR_PSV _T1Interrupt(void)
{
    //InterruptTest4++;
    // Timer 1	1ms
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.T1IF = 0; // interrupt flag reset

    /*
     * Gestione del calcolo del PID, due strade percorribili:
     * Setto dei FLAG PID1_CALC_FLAG e PID2_CALC_FLAG che poi
     * nel main intercetto.
     *
     * Eseguo direttamente il calcolo nell'interrupt a 1mSec
     * richiamando le funzioni Pid1() e Pid2().
     */

    PID1_CALC_FLAG = 1; // PID1 and speed calculation enabled
    PID2_CALC_FLAG = 1; // PID2 and speed calculation enabled
    //    Pid1(); // Ogni 1mSec ricalcola il prescaler, avvia un ciclo
    //    Pid2(); // di lettura dell'IC e esegue il PID sul dato prec.

    /* **************************** TIMER SOFTWARE ******************************/
    GestioneTimerSW(&Timer1mSec);
    GestioneTimerSW(&Timer10mSec);
    /* **************************************************************************/

    /* ******************************** MODBUS **********************************/
    if (TimerRitardoModbus[0]) TimerRitardoModbus[0]--;
    if (TimerOutRxModbus[0]) TimerOutRxModbus[0]--; //time-out dei dati in ricezione
    if (TimerRitardoModbus[1]) TimerRitardoModbus[1]--;
    if (TimerOutRxModbus[1]) TimerOutRxModbus[1]--; //time-out dei dati in ricezione
    /* ************************************************************************* */

    DISICNT = 0; //re-enable interrupts
    //InterruptTest4--;

}

void __attribute__((interrupt, auto_psv, shadow)) _T2Interrupt(void)
{
    //InterruptTest3++;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles

    IFS0bits.T2IF = 0;

    Motore1.overFlowCounter++;
    // reset Vel M1
    if (Motore1.overFlowCounter > 4)
    {
        Motore1.overFlowCounter = 4;
        //Motore1.I_MotorAxelSpeed = 0;

    }
    //InterruptTest3--;
    DISICNT = 0; //re-enable interrupts
    return;
}

void __attribute__((interrupt, auto_psv, shadow)) _T3Interrupt(void)
{
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    //InterruptTest2++;
    IFS0bits.T3IF = 0;
    Motore2.overFlowCounter++;

    //reset Vel M2
    if (Motore2.overFlowCounter > 4)
    {
        Motore2.overFlowCounter = 4;
        //Motore2.I_MotorAxelSpeed = 0;
    }
    DISICNT = 0; //re-enable interrupts

    //InterruptTest2--;
}


#endif

/*  ***************************************************************************
 *  ***************************************************************************
 *  ***************************************************************************
 */
void __attribute__((interrupt, auto_psv, shadow)) _IC1Interrupt(void)
{
    long tmp = 0;
    unsigned int ActualIC1BUF;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.IC1IF = 0;

    ActualIC1BUF = IC1BUF;

    if (Motore1.first_IC_Interrupt_Done == 0)
    {
        Motore1.old_Capture = ActualIC1BUF; // 1st interrupt, acquire start time
        Motore1.overFlowCounter = 0; // reset overflow
        Motore1.first_IC_Interrupt_Done = 1; // next interrupt valid acquire
        Motore1.overFlowCounter = 0; // reset overflow
    }
    else
    { // 2nd interrupt
        tmp = TMR2_VALUE;
        tmp *= Motore1.overFlowCounter; // overflow offset
        tmp += ActualIC1BUF; // capture
        tmp -= Motore1.old_Capture; // click period

        //Motore1.UI_Period = tmp << Motore1.UC_PrescalerDivisor;
        Motore1.period = tmp * Motore1.encoderTimeBase;
        if (!QEI1CONbits.UPDN)
        {
            Motore1.period *= -1;
        }


//        Motore1.UI_Period = (unsigned int) ((long) Motore1.L_RpmConversion / tmp);
//        Motore1.I_MotorAxelSpeed = Motore1.UI_Period;
//        // CCW or CW
//        if (!QEI1CONbits.UPDN)
//        {
//            Motore1.I_MotorAxelSpeed *= -1;
//        }
        Motore1.first_IC_Interrupt_Done = 0;
        IC1CONbits.ICM = 0; // Disable Input Capture 1 module,
        // re-enabled after PID computation on 1mSec Interrupt Timer
    }

    DISICNT = 0; //re-enable interrupts
    Motore1.overFlowCounter = 0; // reset overflow
}

void __attribute__((interrupt, auto_psv, shadow)) _IC2Interrupt(void)
{
    long tmp = 0;
    unsigned int ActualIC2BUF;
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IFS0bits.IC2IF = 0;

    ActualIC2BUF = IC2BUF;

    if (Motore2.first_IC_Interrupt_Done == 0)
    {
        Motore2.old_Capture = ActualIC2BUF; // 1st interrupt, acquire start time
        Motore2.overFlowCounter = 0; // reset overflow
        Motore2.first_IC_Interrupt_Done = 1; // next interrupt valid acquire
    }
    else
    {
        // 2nd interrupt

        tmp = TMR2_VALUE;
        tmp *= Motore2.overFlowCounter; // overflow offset
        tmp += ActualIC2BUF; // capture
        tmp -= Motore2.old_Capture; // click period

        //Motore1.UI_Period = tmp << Motore1.UC_PrescalerDivisor;
        Motore2.period = tmp * Motore2.encoderTimeBase;
        if (!QEI2CONbits.UPDN)
        {
            Motore2.period *= -1;
        }


//        tmp = TMR2_VALUE;
//        tmp *= Motore2.UC_OverFlowCounter; // overflow offset
//        tmp += ActualIC2BUF; // capture
//        tmp -= Motore2.UI_Old_Capture; // click period
//        Motore2.UI_Period = (unsigned int) ((long) Motore2.L_RpmConversion / tmp); // Valore istantaneo di periodo.
//        Motore2.I_MotorAxelSpeed = Motore2.UI_Period;
//
//        // CCW or CW
//        if (!QEI2CONbits.UPDN)
//        {
//            Motore2.I_MotorAxelSpeed *= -1;
//        }

        Motore2.first_IC_Interrupt_Done = 0;
        IC2CONbits.ICM = 0; // Disable Input Capture 1 module,
        // re-enabled after PID computation on 1mSec Interrupt Timer
    }
    DISICNT = 0; //re-enable interrupts
    Motore1.overFlowCounter = 0; // reset overflow
}

void AggiornaVariabiliModbus(void)
{
    /* **************************************************************** */
    /* Aggiorno tutte le variabili per il protocollo modbus, le variabili
     * modbus vengono lette al max ogni 10mSec e quindi posso aggiornarle
     * con calma
     *                                                                  */
    /* **************************************************************** */

    // ----------------------  ADC value average calculus ----------------------//
    VarModbus[INDICE_TENSIONE_ALIM] = LeggiADC(PIC_AN0); // AN0 del micro = VMOT
    VarModbus[INDICE_AN1] = LeggiADC(PIC_AN1); // AN1 del PCB; AN5 del micro
    VarModbus[INDICE_AN2] = LeggiADC(PIC_AN2); // AN2 del PCB; AN5 del micro
    VarModbus[INDICE_AN3] = LeggiADC(PIC_AN3); // AN3 del PCB; AN6 del micro
    VarModbus[INDICE_AN4] = LeggiADC(PIC_AN4); // AN4 del PCB; AN7 del micro

    /*
     *     NOTA RELAZIONI PERIFERICHE e VARIABILI
     *     TIMER2 <=> IC1 <=> QEI1 <=> LEFT <=> 1
     *     TIMER3 <=> IC2 <=> QEI2 <=> RIGHT <=> 0
     */

    VarModbus[INDICE_PID_ERROR_RIGHT] = (int) PID1.error_T_0;
    VarModbus[INDICE_PID_ERROR_LEFT] = (int) PID2.error_T_0;




    /* */
    //Struttura per accedere ad un dato o come Long o come due Integer ( per inviare il dato via Modbus )
    //    typedef struct
    //    {   union
    //        {   struct
    //            {   unsigned int high_part;
    //                unsigned int low_part;
    //            };
    //            long LongVal;
    //       };
    //    } lvalue;
    //      extern volatile lvalue TmpSplitLongToWord;


    TmpSplitLongToWord.LongVal = 0;
    TmpSplitLongToWord.high_part = 0;
    TmpSplitLongToWord.low_part = 0;
    TmpSplitLongToWord.LongVal = Motore1.period;
    VarModbus[INDICE_DEBUG_00] = TmpSplitLongToWord.low_part;
    VarModbus[INDICE_DEBUG_01] = TmpSplitLongToWord.high_part;

    TmpSplitLongToWord.LongVal = 0;
    TmpSplitLongToWord.high_part = 0;
    TmpSplitLongToWord.low_part = 0;
    TmpSplitLongToWord.LongVal = Motore2.period;
    VarModbus[INDICE_DEBUG_02] = TmpSplitLongToWord.low_part;
    VarModbus[INDICE_DEBUG_03] = TmpSplitLongToWord.high_part;


    TmpSplitLongToWord.LongVal = 0;
    TmpSplitLongToWord.high_part = 0;
    TmpSplitLongToWord.low_part = 0;
    TmpSplitLongToWord.LongVal = PID1.setpoint;
    VarModbus[INDICE_DEBUG_04] = TmpSplitLongToWord.low_part;
    VarModbus[INDICE_DEBUG_05] = TmpSplitLongToWord.high_part;

    TmpSplitLongToWord.LongVal = 0;
    TmpSplitLongToWord.high_part = 0;
    TmpSplitLongToWord.low_part = 0;
    TmpSplitLongToWord.LongVal = PID1.setpoint;
    VarModbus[INDICE_DEBUG_06] = TmpSplitLongToWord.low_part;
    VarModbus[INDICE_DEBUG_07] = TmpSplitLongToWord.high_part;
}

void __attribute__((interrupt, auto_psv, shadow)) _StackError(void)
{
    /* Sicurezza , fermo i motori!*/
    LED1 = LED_ON;
    LED2 = LED_ON;

    MotorControlEnable(MOTORE1, MOTOR_DEACTIVE);
    MotorControlEnable(MOTORE2, MOTOR_DEACTIVE);

    while (1);
}

void __attribute__((interrupt, auto_psv, shadow)) _OscillatorFail(void)
{
    while (1);
}

void __attribute__((interrupt, auto_psv, shadow)) _MathError(void)
{
    while (1);
}

void __attribute__((interrupt, auto_psv, shadow)) _DMACError(void)
{
    while (1);
}

