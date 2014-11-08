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

#define _MOTOR_FAIL MOTORE->UC_Fail

/*  FUNZIONE DA RICHIAMARE SOLO AL RESET
*/
void InitMotorStructure()
{
    Motore1.overFlowCounter = 1;
    Motore2.overFlowCounter = 1;

    Motore1.first_IC_Interrupt_Done = 0;
    Motore2.first_IC_Interrupt_Done = 0;
    Motore1.IC_idx = 0;
    Motore2.IC_idx = 0;
    Motore1.ICM_Restart_Value = 3;
    Motore2.ICM_Restart_Value = 3;

    Motore1.motorNumber = 1;     // Mi serve nelle funzioni a cui passo la struttura come argomento per
    Motore2.motorNumber = 2;     // sapere su che struttura sto lavorando

    Motore1.fail = 0;    //  I motori non sono in errore.
    Motore2.fail = 0;

    UpdateMotorStructure();
}

/* FUNZIONE DA RICHIAMARE OGNI VOLTA CHE VENGONO MODIFICATI DEI PARAMETRI STRUTTURALI DEL ROBOT
*/
void UpdateMotorStructure()
{  
    #warning: Value not used in current software revision for internal calculation but can be used by ROS...
    Motore1.motorRpmMax = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT];
    Motore1.motorRpmMin = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_LEFT] * -1;
    Motore2.motorRpmMax = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT];
    Motore2.motorRpmMin = ParametriEEPROM[EEPROM_MODBUS_ROBOT_MOTOR_RPMMAX_RIGHT] * -1;
}




/*
 *  Funzione da chiamare continuamente sotto Main, dopo aver
 *  richiamato GestioneWatchdog()
 *
 * Gestisce le sicurezze legate ai motori, si occupa di :
 * - verificare eventuali condizioni di allarme
 * - attivare/disattivare il driver del motore
 *
 */
void MotorAlarmRoutine(void) //volatile Motor_t *MOTORE)
{
    Motore1.fail ? MotorControlEnable(MOTORE1,MOTOR_DEACTIVE) : MotorControlEnable(MOTORE1,MOTOR_ACTIVE);
    Motore2.fail ? MotorControlEnable(MOTORE2,MOTOR_DEACTIVE) : MotorControlEnable(MOTORE2,MOTOR_ACTIVE);
}

/*! \brief Questa funzione abilita/disabilita il driver motori tenendo conto del flag modbus FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY
 * per determinarne la polarità, ovvero se il driver è attivo col pin a "1" o a "0".
 * Esempi:
 * MotorControlEnable(MOTORE1,MOTOR_ACTIVE);
 * MotorControlEnable(MOTORE2,MOTOR_ACTIVE);
 * MotorControlEnable(MOTORE1,MOTOR_DEACTIVE);
 * MotorControlEnable(MOTORE2,MOTOR_DEACTIVE);
 */
/*!
  \param Motor Indica su quale uscita motore agire, può valere MOTORE1 o MOTORE2
  \param Status Indica se attivare (MOTOR_ACTIVE) o disattivare (MOTOR_DEACTIVE) il motore selezionato.
  \return void
*/
void MotorControlEnable(unsigned char Motor, unsigned char Status)
{
    if( Motor == MOTORE1  )
    {   if ( Status == MOTOR_ACTIVE )
        {   // Attivo il motore 1
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE1 = 1;
            else
                MOTOR_ENABLE1 = 0;
         }
        else
        {   // Disattivo il motore 1
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE1 = 0;
            else
                MOTOR_ENABLE1 = 1;
         }
    }

    if( Motor == MOTORE2  )
    {   if ( Status == MOTOR_ACTIVE )
        {   // Attivo il motore 2
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE2 = 1;
            else
                MOTOR_ENABLE2 = 0;
         }
        else
        {   // Disattivo il motore 2
            if( ParametriEEPROM[EEPROM_MODBUS_STATUSBIT2] & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY)
                MOTOR_ENABLE2 = 0;
            else
                MOTOR_ENABLE2 = 1;
         }
    }
}



