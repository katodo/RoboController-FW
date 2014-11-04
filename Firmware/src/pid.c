// standard include
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

#include "limits.h"

void Pid1(void)
{ //__builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    PID1_CALC_FLAG = 0; // Attivato sotto interrupt ogni 1mSec

    /*  *************************************************************************** */
    // Calcolo il prescaler per la prossima misura
    /*
     * Il nuovo valore del prescaler viene determinato dal valore dell'ultima lettura espresso in nSec.
     * Con base tempi 25nSec il periodo può variare in questi range:
     * - PRESCALER x1   : 0-65535 = 0-1638375 nSec ( 0 -    1,638375mSec ) -> Risoluzione 25nSec
     * - PRESCALER x4   : 0-65535 = 0-6553500 nSec ( 0 -    6,553500mSec ) -> Risoluzione 100nSec
     * - PRESCALER x16 : 0-65535 = 0-26214000 nSec ( 0 -  26,214000mSec ) -> Risoluzione 400nSec
     *
     * Ovviamente, per avere la maggior precisione, devo cercare di usare per quanto possibile la base tempi
     * di 25nSec tenendomi però un certo lasco per evitare falsi cambi.
     *
     * La condizione di if è la condizione di "Ritorno in quel valore di prescaler".
     * if(Motore1.period < 1600000 )
     * if(Motore1.period > 1630000 && Motore1.period < 6500000)
     * if (Motore1.period > 6550000)
    */

    //if (Motore1.I_MotorAxelSpeed < 1800) // UI_Period
    if(Motore1.period < 1600000 )
    {   //PRESCALER x1  : used from 0 to 1600000nSec period ( 0  1,6mSec )
        // return in this situation with Period below 1,6mSec
        Motore1.ICM_Restart_Value = 0b011;
        Motore1.captureEventDivisor = 1; // con IC1CONbits.ICM = 0b011;
        //Motore1.UC_CaptureEventDivisor = 4; // con IC1CONbits.ICM = 0b100;
        //Motore1.UC_CaptureEventDivisor = 16; // con IC1CONbits.ICM = 0b101;

        //Motore1.I_Prescaler_IC = 1; //  bit2-0:     Generate capture event on every 1st rising
        //Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_1; // every rising edge
    }
    //if (Motore1.I_MotorAxelSpeed > 2200 && Motore1.I_MotorAxelSpeed < 3800)
    if (Motore1.period > 1630000 && Motore1.period < 6500000)
    {   //PRESCALER x4   : used from 1630000nSec to 6500000nSec period ( 1,63 to 6,5mSec )
        // return in this situation with Period greater than 1,63mSec and lower than 6,5mSec
        Motore1.ICM_Restart_Value = 0b100;
        //Motore1.UC_CaptureEventDivisor = 1; // con IC1CONbits.ICM = 0b011;
        Motore1.captureEventDivisor = 4; // con IC1CONbits.ICM = 0b100;
        //Motore1.UC_CaptureEventDivisor = 16; // con IC1CONbits.ICM = 0b101;

        //Motore1.I_Prescaler_IC = 4; //  bit2-0:     Generate capture event on every 4th rising
        //Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_2; // every 4th rising edge
    }
    //if (Motore1.I_MotorAxelSpeed > 4200)
    if (Motore1.period > 6550000)
    {   // PRESCALER x16 :used from 6550000nSec and 26214000nSec period ( 6,55mSec to 26,214000mSec )
        Motore1.ICM_Restart_Value = 0b101;
        //Motore1.UC_CaptureEventDivisor = 1; // con IC1CONbits.ICM = 0b011;
        //Motore1.UC_CaptureEventDivisor = 4; // con IC1CONbits.ICM = 0b100;
        Motore1.captureEventDivisor = 16; // con IC1CONbits.ICM = 0b101;

        //Motore1.I_Prescaler_IC = 16; //  bit2-0:     Generate capture event on every 16th rising
        //Motore1.L_RpmConversion = Motore1.T_FattoreConversioneRPM_3; // every 16th rising edge
    }
    Motore1.encoderTimeBase = (long)Motore1.prescaler_TIMER * (long)Motore1.captureEventDivisor;
    /*  *************************************************************************** */

    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    { // In modalità PID calcolo il PID altrimenti sono in modalità PWM e chiamo questa funzione solo per i calcoli della velocità.
        Pid(&PID1, &Motore1); // T = 6.9uSec MEDI misurati (5.8 typ )
        SetDCMCPWM1(1, PID1.outPid, 0); // setta il PWM  del motore
        VarModbus[INDICE_RD_PWM_CH1] = PID1.outPid; // aggiorno PWM in lettura
    }
    IC1CONbits.ICM = Motore1.ICM_Restart_Value;
}

void Pid2(void)
{
    PID2_CALC_FLAG = 0; // Attivato sotto interrupt ogni 1mSec
    /*  *************************************************************************** */
    if(Motore2.period < 1600000 )
    {   Motore2.ICM_Restart_Value = 0b011;
        Motore2.captureEventDivisor = 1; // con IC1CONbits.ICM = 0b011;
    }
    if(Motore2.period > 1630000 && Motore2.period < 6500000)
    {   Motore2.ICM_Restart_Value = 0b100;
        Motore2.captureEventDivisor = 4; // con IC1CONbits.ICM = 0b100;
    }
    if(Motore2.period > 6550000)
    {   Motore2.ICM_Restart_Value = 0b101;
        Motore2.captureEventDivisor = 16; // con IC1CONbits.ICM = 0b101;
    }
    Motore2.encoderTimeBase = (long)Motore2.prescaler_TIMER * (long)Motore2.captureEventDivisor;
    /*  *************************************************************************** */

    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_PID_EN)
    { // In modalità PID calcolo il PID altrimenti sono in modalità PWM e chiamo questa funzione solo per i calcoli della velocità.
        Pid(&PID2, &Motore2); // T = 6.9uSec MEDI misurati (5.8 typ )
        SetDCMCPWM1(2, PID2.outPid, 0); // setta il PWM  del motore
        VarModbus[INDICE_RD_PWM_CH2] = PID2.outPid; // aggiorno PWM in lettura
    }
    IC2CONbits.ICM = Motore2.ICM_Restart_Value;
}

void Pid(volatile Pid_t *PID, volatile Motor_t *MOTORE)
{
    long Setpoint = PID->setpoint; // Dato da mantenere/raggiungere ( velocità di crociera )
    long Processo = MOTORE->period;

    //    __builtin_disi(0x3FFF); /* disable interrupts, vedere pg 181 di MPLAB_XC16_C_Compiler_UG_52081.pdf */

    /*
     *  PWM varia da 0 a 4095 con centro a 2048, modalita LAP
     *  Tutti i calcoli sono a tre decimali, interi moltiplicati per 1000
     *
     *  _SETPOINT è la velocità in RPM dell'asse motore da raggiungere
     *  _AXELSPEED è la velocità in RPM istantanea dell'asse motore.
     *  
     * Tutto quello che riguarda il riduttore e la ruota viene calcolato a parte.
     *  
     * Da _SETPOINT e _AXELSPEED derivo:
     *      L_ScaledSetpoint  :   Velore di velocità da raggiungere, dato moltiplicato per 1000
     *      L_ScaledProcesso  :   Velore di velocità istantaneo, dato moltiplicato per 1000
     */


    /* ***************************************************************************************
     *  ************************************   __builtin_  ************************************
     *  ***************************************************************************************
     *  I calcoli sono ottimizzati mediante l'utilizzo delle funzioni __builtin_
     *  descritte nel documento "MPLAB_XC16_C_Compiler_UG_52081.pdf" presente nella directory
     *  del compilatore XC16
     *
     *  __builtin_mulss   :   MPLAB_XC16_C_Compiler_UG_52081.pdf pg294
     *  ***************************************************************************************
     */

    int saturazione; // Indica se il controllo è in saturazione.
    // Da usare per l'anti-windup

    if ((PID->oldContrValue == 4095) || PID->oldContrValue == 1) // Il controllo è saturo
        saturazione = 1;
    else
        saturazione = 0;

    // Rampa è il dato effettivo di "Setpoint" da raggiungere in ciascun ciclo.
    // Tende a raggiungere il valore di SetPoint in base all'ampiezza dello Step.
    if (VarModbus[INDICE_STATUSBIT1] & FLG_STATUSBI1_EEPROM_RAMP_EN)
    { //  Modalità rampa
        if (PID->ramp < Setpoint)
        { // Rampa in salita

            if ((PID->ramp > -DEAD_ZONE) && (PID->ramp < DEAD_ZONE))
            { //  Se _RAMPA cade nell'interno di +/- DEAD_ZONE
                //  Salto all'esterno.
                PID->ramp = DEAD_ZONE;
            }

            PID->ramp += PID->rampStep;
            if (PID->ramp > Setpoint)
                PID->ramp = Setpoint;
            if (PID->ramp > MOTORE->motorRpmMax)
                PID->ramp = MOTORE->motorRpmMax;
        }


        if (PID->ramp > Setpoint)
        { //  Rampa in discesa


            if ((PID->ramp > -DEAD_ZONE) && (PID->ramp < 0))
            {
                PID->ramp = -DEAD_ZONE;
            }
            else if ((PID->ramp >= 0) && (PID->ramp < DEAD_ZONE))
            {
                PID->ramp = DEAD_ZONE;
            }
            else if ((PID->ramp > -DEAD_ZONE) && (PID->ramp < DEAD_ZONE))
            {
                PidReset(PID, MOTORE);
            }


            PID->ramp -= PID->rampStep;
            if (PID->ramp < Setpoint)
                PID->ramp = Setpoint;
            if (PID->ramp < MOTORE->motorRpmMin)
                PID->ramp = MOTORE->motorRpmMin;
        }
    }
    else
    { // Modalità senza rampa
        if ((Setpoint < DEAD_ZONE) & (Setpoint > -DEAD_ZONE))
        {
            Setpoint = 0;
            PidReset(PID, MOTORE); // All'interno della banda morta resetto il PID
        }

        PID->ramp = Setpoint;
    }

    PID->error_T_0 = (PID->ramp - Processo); // calcolo errore tra il setpoint e il Current

    //    PID->ComponenteFeedForward = RescaledErrore * 2;
    //    if (PID->ComponenteFeedForward >  2045 )
    //        PID->ComponenteFeedForward =  2045;    // limiti componente feed forward
    //
    //    if (PID->ComponenteFeedForward < -2045 )
    //        PID->ComponenteFeedForward = -2045;

    // Y[n] = Y[n-1] + P*(X[n] - X[n-1] ) + I*X[n] + D*(X[n] - 2*X[n-1] + X[n-2])

    // CONTRIBUTO PROPORZIONALE
    PID->propContrib = PID->Kp * (PID->error_T_0 - PID->error_T_1);

    // CONTRIBUTO INTEGRALE
    if (saturazione == 1 || // Controllo saturo -> Anti-WindUp
            PID->Ki == 0) // Nessun contributo integrale
        PID->integrContrib = 0;
    else
        PID->integrContrib = PID->Ki * PID->error_T_0;

    // CONTRIBUTO DERIVATIVO
    if (PID->Kd == 0) // Nessun contributo derivativo
        PID->derivContrib = 0;
    else
        PID->derivContrib = PID->Kd * (PID->error_T_0 - 2 * PID->error_T_1 + PID->error_T_2);

    // Aggiornamento errori
    PID->error_T_2 = PID->error_T_1;
    PID->error_T_1 = PID->error_T_0;

    // Ora che è differenziale ci va il "+=" by Walt
    PID->sum += (PID->propContrib + PID->integrContrib + PID->derivContrib); // sommatoria errori

    if (PID->sum > (LONG_MAX - 1000))
        PID->sum = LONG_MAX;

    if (PID->sum < (LONG_MIN + 1000))
        PID->sum = LONG_MIN;

    if (PID->ramp == 0)
    {
        PID->outPid = 2048;
        PID->integral = 0;
        PID->ramp = 0;
    }
    else
    { // sommatore per il calcolo del reale PWM da inviare al motore
        PID->outPid = 2048 + PID->sum;
    }

    if (PID->outPid > 4095) PID->outPid = 4095;
    if (PID->outPid < 1) PID->outPid = 1;

    PID->oldContrValue = PID->outPid;

    //    __builtin_disi(0x0000); /* enable interrupts, vedere pg 181 di MPLAB_XC16_C_Compiler_UG_52081.pdf */
}

void PidReset(volatile Pid_t *PID, volatile Motor_t *MOTORE)
{
    PID->rampStep = 0;
    PID->integral = 0;
    PID->integrContrib = 0;
    PID->propContrib = 0;
    PID->derivContrib = 0;

    PID->error_T_0 = 0;
    PID->error_T_1 = 0;
    PID->error_T_2 = 0;

    PID->sum = 0;
    PID->oldContrValue = 0;

    PID->outPid = 0;
}
