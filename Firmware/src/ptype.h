#ifndef PTYPE_INC
#define PTYPE_INC


/*
 *          NOTA RELAZIONI PERIFERICHE e VARIABILI
 *     TIMER2 <=> IC1 <=> QEI1 <=> LEFT <=> 1
 *     TIMER3 <=> IC2 <=> QEI2 <=> RIGHT <=> 0
 */


/* //////////////////////////////////////////////////////////////////////////*/
/* Included	in "dsPid_definitions.h", it contains functions prototypes       */
/* //////////////////////////////////////////////////////////////////////////*/

//! Struttura dati usata per i timer Software
//! \sa Funzione GestioneTimerSW() per ulteriori dettagli
//! \sa Funzione _T1Interrupt() per la chiamata della funzione sotto interrupt
typedef struct _timer_
{	unsigned	T_flag:1;           //!< Flag che diventa TRUE nel momento in cui il tempo è trascorso
	unsigned int	T_count_time;       //!< Varibile che viene decrementata ogni xmSec per contare il tempo trascorso
	unsigned int	T_initial_value;    //!< Valore con cui caricare T_count_time per un nuovo conteggio
} Timer_t;

//! Struttura usata per i motori, contiene tutte le informazioni di controllo relative al motore e
//! al relativo encoder.
typedef struct _motor_t_
{   
    /* VARIABILI INTERNE ALLA ROBOCONTROLLER            */
    volatile unsigned char  fail;                // Se a "1" il motore è in Fail, azzero il PID e fermo i ponti.
    volatile unsigned char  motorNumber;         // Indica a quale motore fa riferimento la struttura in uso ( vedi in funzione PID )
    volatile unsigned char  overFlowCounter;         // contatori overflow Timer 2, uno per ogni input capture
    volatile unsigned char  overFlowErrorCounter;         // contatori overflow Timer 2, uno per ogni input capture
    volatile unsigned char  ICM_Restart_Value;       //  Valore di ICM con cui riattivare l'interrupt
    volatile unsigned char  first_IC_Interrupt_Done; // indici per arry e controllo primo interrupt
    volatile unsigned char  IC_idx;
    volatile unsigned int   old_Capture;             // variabili per il primo interrupt
    volatile int            motorRpmMax;
    volatile int            motorRpmMin;
    volatile int            prescaler_TIMER;      // Prescaler usato dal TIMER2/3, mi serve a calcolare la base tempi
    volatile unsigned char  captureEventDivisor; // Value: 1,4,16 , see IC2CONbits.ICM bit status
    volatile long           encoderTimeBase;
    volatile long           period;               // nSec vale of time from 2 tick of encoder
    //volatile float          costante_Conversione_Vlin_to_Vang;
} Motor_t;

//! Struttura usata per la gestione dell'InputCapture e le misure di velocità
typedef struct _inputCapture_t_
{   volatile long   oldMeasure;
    volatile int    errorCounter;
} InputCapture_t;

typedef struct _pid_t_
{   volatile long   Kp;
    volatile long   Ki;
    volatile long   Kd;
    volatile long   integral;       // ??????????????
    volatile long   sum;            // error sum
    volatile long   oldContrValue;  // Last control value
    volatile long   setpoint;       // setpoint :)
    volatile long   ramp;           // ramp setpoint
    volatile long   rampStep;       // step of the ramp
    volatile long   error_T_0;      // current error
    volatile long   error_T_1;      // error at T-1
    volatile long   error_T_2;      // error at T-2
    volatile long   propContrib;    // proportional contribution
    volatile long   integrContrib;  // integral contribution
    volatile long   derivContrib;   // derivative contribution
    volatile long   outPid;          // LAst output value
} Pid_t;


//! Struttura usata per gestire la segnalazione del codice di errore tramite i LED
//!
typedef struct _led_t_
{   volatile unsigned char  busy;               // If "1" there is a signal sequence
    volatile unsigned char  errorCode;
    volatile unsigned char  phase;              // Indicates the signal phase (Pause, LedOn, LedOff, EndSequence )
    volatile unsigned char  blinkDone;          // Counts the number of blinks (Led On - Led Off )
    volatile unsigned char  repetition;         // Repetition counter, 0 = infinite
    volatile unsigned char  repetitionDone;     // Repetition done
    volatile unsigned int   ton;
    volatile unsigned int   toff;
    volatile unsigned int   tpause;
    volatile unsigned int   timer;
} Led_t;

typedef struct _bit_
{   unsigned bit0:1;
    unsigned bit1:1;
    unsigned bit2:1;
    unsigned bit3:1;
    unsigned bit4:1;
    unsigned bit5:1;
    unsigned bit6:1;
    unsigned bit7:1;
    unsigned bit8:1;
    unsigned bit9:1;
    unsigned bit10:1;
    unsigned bit11:1;
    unsigned bit12:1;
    unsigned bit13:1;
    unsigned bit14:1;
    unsigned bit15:1;
} Bit_t;

//Struttura per accedere ad un dato o come Float o come due Integer ( per salvare il float in EEPROM )
typedef struct _fvalue_t_
{   union
    {   struct
        {   unsigned int high_part;
            unsigned int low_part;
        };
        float fval;
   };
} fvalue;

//Struttura per accedere ad un dato o come Long o come due Integer ( per inviare il dato via Modbus )
typedef struct _lvalue_t_
{   union
    {   struct
        {   unsigned int high_part;
            unsigned int low_part;
        };
        long LongVal;
   };
} lvalue;

void PidPwmSwitchRoutine(void);
void SetpointUpdate(void);
void MotorControlEnable(unsigned char Motor, unsigned char Status);
void UpdateModbusVar(void);
void Usart1Setting(void);
void Usart2Setting(void);
void TxString(unsigned char *Punt, unsigned char NCar, unsigned char Port);

void Settings(void);

void ISR_Settings(void);
void _ISR _INT1Interrupt(void);
void _ISR _U1RXInterrupt(void);
void _ISR _U1TXInterrupt(void);
void _ISR _IC1Interrupt(void);
void _ISR _IC2Interrupt(void);
void _ISR _T1Interrupt(void);
void _ISR _T2Interrupt(void);
void _ISR _CNInterrupt(void);

//TIMER SOFTWARE
void TimerSW_Reload(volatile Timer_t *Timer);
void TimerSW_Routine(volatile Timer_t *Timer);

//Modbus
unsigned int WordRead(unsigned int Address);
unsigned char WordWrite(unsigned int Address,unsigned int Word);
unsigned char BitRead(unsigned int Address);
unsigned char BitWrite(unsigned int Address,unsigned char Bit);
void ModbusRoutine(unsigned char Port);
void ModbusRxRoutine(unsigned char Code, unsigned char Port);
void ModbusReadBit(unsigned char Port);
void ModbusReadWord(unsigned char Port);
void ModbusWriteSingleBit(unsigned char Port);
void ModbusWriteSingleWord(unsigned char Port);
void ModbusWriteMultipleBits(unsigned char Port);
void ModbusWriteMultipleWords(unsigned char Port);
void ModbusErroreResponse(unsigned char NumeroErrore, unsigned char Port);
unsigned int ModbusCheckCRC16(unsigned char *P,unsigned char NByte);
void FreeRxBuffer(unsigned char Port);
void ModbusSerialInit(unsigned char Port);


// Gestioni analogiche
void InitADC(void);
int LeggiADC(int NumeroIngresso);
void MediaADC(unsigned int * OutputIngressiMediati, unsigned int DmaAdc[][SAMP_BUFF_SIZE]);
unsigned int TaraturaAnalogiche(unsigned int FlagTaratura);

// EPROM.C
void InitializationEEPROM(void);
void RestoreEEPROMData(void);

// PID.C
void Pid1(void);
void Pid2(void);
void PidReset(volatile Pid_t *PID, volatile Motor_t *MOTORE);
void Pid(volatile Pid_t *PID, volatile Motor_t *MOTORE);
void InitMotorStructure();
void UpdateMotorStructure();

// Alarm.c
void WatchdogRoutine(void);
void MotorAlarmRoutine(void);

// Led.c
void AlarmRoutine();
void SetLedErrorCode(   volatile Led_t *LED,
                        unsigned char ErrorCode, unsigned char Ripetizioni,
                        unsigned int Ton, unsigned int Toff, unsigned int Tpausa );
void ErrorCodeRoutine_Led1(volatile Led_t *LED);
void ErrorCodeRoutine_Led2(volatile Led_t *LED);

#endif
