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

/*
 *  Funzione da chiamare continuamente sotto main.
 *
 *  Questa funzione intercetta i segnali di allarme dal programma e li visualizza in
 *  base alla loro priorità.
 *
 *
 */
void AlarmRoutine()
{
    // Se compilo in debug i LED li uso in giro per il programma per test e non
    // li muovo da questa funzione
#ifndef DEBUG

// Segnalazioni su LED1 : ERRORI
//    #define LED_ERRORCODE_00_NOERROR    0   // Nessun errore da gestire
//    #define LED_ERRORCODE_01_WATCHDOG   1   // Errore comunicazione seriale ( Watchdog comunicazione )
//    #define LED_ERRORCODE_02_FAILMOTOR1 2   // Motore 1 in fail
//    #define LED_ERRORCODE_03_FAILMOTOR2 3   // Motore 2 in fail
//    #define LED_ERRORCODE_04_FAILMOTORI 4   // Tutti i motori in errore
//    #define LED_ERRORCODE_05_CHECKERRORIOK 5   // Usato all'accensione per segnalare che non vi sono errori di sistema( EEPROM E SIMILI )


    // Gli errori sono gestiti con priorità, LED_ERRORCODE_00_NOERROR ha priorità minima, LED_ERRORCODE_01_WATCHDOG ha priorità massima
    if(!Led1Segnalazione.busy)
    {
        if(!ComunicationWatchDogTimer)
        {   //Priorità massima...
            SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_01_WATCHDOG, 0, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
        }
        else if (Motore1.fail && Motore2.fail)
        { SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_04_FAILMOTORI, 0, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
        }
        else if (Motore1.fail)
        { SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_02_FAILMOTOR1, 0, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
        }
        else if (Motore2.fail)
        { SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_03_FAILMOTOR2, 0, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
        }
        else
        {   // Nessun allarme... priorità minima
            //SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_00_NOERROR, 0, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
            SetLedErrorCode( &Led1Segnalazione, LED_ERRORCODE_00_NOERROR, 0, SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, 0);
        }
    }
    
// Segnalazioni su LED2 : STATUS
//    #define LED_STATUSCODE_01_NORMAL_RUN    1
//    #define LED_STATUSCODE_02_WATCHDOGFAIL  2

    if(!Led2Segnalazione.busy)
    {
        if (!ComunicationWatchDogTimer)
        {   //Priorità massima...
            SetLedErrorCode(    &Led2Segnalazione, LED_STATUSCODE_02_WATCHDOGFAIL, 0,
                                SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
        }
        else if (LedStatusSignal.bit0)
        {   // Entro in modalità PID
            SetLedErrorCode(    &Led2Segnalazione, LED_STATUSCODE_03_ENTER_IN_PIDMODE, 2,
                                SEGNALAZIONELED_TON_FAST, SEGNALAZIONELED_TOFF_FAST, SEGNALAZIONELED_TPAUSE );
            LedStatusSignal.bit0 = 0;
        }
        else if (LedStatusSignal.bit1)
        {   // Entro in modalità PWM
            SetLedErrorCode(    &Led2Segnalazione, LED_STATUSCODE_04_ENTER_IN_PWMMODE, 2,  // Infinite Loop
                                SEGNALAZIONELED_TON_FAST, SEGNALAZIONELED_TOFF_FAST, SEGNALAZIONELED_TPAUSE );
            LedStatusSignal.bit1 = 0;
        }
        else
        {   //SetLedErrorCode( &Led2Segnalazione, LED_STATUSCODE_01_NORMAL_RUN, 0, 20, 20, 0);
            SetLedErrorCode(    &Led2Segnalazione, LED_STATUSCODE_01_NORMAL_RUN, 0,
                                SEGNALAZIONELED_TON, SEGNALAZIONELED_TOFF, SEGNALAZIONELED_TPAUSE);
        }

    }
#endif
}



/*
 *  I codici di errore vengono segnalati all'utente con una sequenza di lampeggi
 *  pari al numero del codice di errore segueita da una pausa.
 *  La sequenza è gestita dalle funzioni GestioneLed1 e GestioneLed2 sotto timer
 *  nel main.
 *
 *  I lampeggi sono generati da uno stato LedOn da 100mSec, seguito da uno
 *  stato LedOff di 200mSec il tutto ripetuto per ErrorCode volte e poi seguito
 *  da una pausa di 600mSec.
 *
 */

/*
 * Quando chiamata sovrascrive la precedente procedura di segnalazione
 * Il codice di errore "0" spegne il led.
 */
void SetLedErrorCode(   volatile Led_t *LED,
                        unsigned char ErrorCode, unsigned char Ripetizioni,
                        unsigned int Ton, unsigned int Toff, unsigned int Tpausa )
{   //Inizializzo la segnalazione dell'errore, interrompe eventuali segnalazioni precedenti
    
    LED->ton          = Ton; //SEGNALAZIONELED_TON;
    LED->toff         = Toff; //SEGNALAZIONELED_TOFF;
    LED->tpause       = Tpausa; //SEGNALAZIONELED_TPAUSE;
    LED->errorCode       = ErrorCode;
    LED->phase           = 0;
    LED->repetition      = Ripetizioni;
    LED->repetitionDone  = 0;
    LED->busy            = 1;
}

/*
 * Chiamata ciclicamente ogni 10mSec
 */
void ErrorCodeRoutine_Led1(volatile Led_t *LED)
{
#ifndef DEBUG
    switch(LED->phase)
    {   case 0  :   //Inizio Sequenza Inizializzo il timer
                    LED->timer   = LED->ton;
                    LED->phase = 1;
                    LED->blinkDone = 0;
                    if(LED->errorCode == 0)
                    {   // Nel caso di codice 0 non gestisco nemmeno un lampeggio,
                        // Spengo il led ed esco.
                        LED1 = LED_OFF;
                        LED->phase = 4;
                    }
                    break;
        case 1  :   LED1 = LED_ON;

                    if(LED->timer > 0)
                    {   LED->timer--;
                    }
                    else
                    {   LED->timer   = LED->toff;
                        LED->phase = 2;
                    }
                    break;

        case 2  :   LED1 = LED_OFF; 
                    if(LED->timer > 0)
                    {   LED->timer--;
                    }
                    else
                    {   LED->timer   = LED->toff;
                        LED->blinkDone++;   // Ciclo completo

                        if( LED->blinkDone < LED->errorCode)
                        {   // Devo eseguire ancora dei blink
                            LED->timer   = LED->ton;
                            LED->phase = 1;
                        }
                        else
                        {   // Ho completato i lampeggi, salto nella pausa
                            LED->timer   = LED->tpause;
                            LED->phase = 3;
                        }
                    }
                    break;

        case 3  :   LED1 = LED_OFF;                    
                    if(LED->timer > 0)
                    {   LED->timer--;
                    }
                    else    //    if(LED->timer == 0)
                    {   //Pausa terminata
                        LED->repetitionDone++;
                        // Verifico se devo ripetere ancora la sequenza
                        if( (LED->repetitionDone < LED->repetition) || (LED->repetition == 0) )
                        {   LED->phase = 0;
                            if(LED->repetition == 0)
                            {   // In loop infinito esco dallo stato di busy appena è stata
                                // completata una sequenza di segnalazione
                                LED->busy = 0;  // Posso gestire una nuova sequenza di segnalazione
                            }

                        }
                        else
                        {   LED->phase = 4;
                        }
                    }
                    break;
                    
          case 4  : // Attendo all'infinito...
                    LED->busy = 0;  // Posso gestire una nuova sequenza di segnalazione
                    break;
    }
#endif
}

void ErrorCodeRoutine_Led2(volatile Led_t *LED)
{

#ifndef DEBUG

    switch(LED->phase)
    {   case 0  :   //Inizio Sequenza Inizializzo il timer
                    LED->timer   = LED->ton;
                    LED->phase = 1;
                    LED->blinkDone = 0;
                    if(LED->errorCode == 0)
                    {   // Nel caso di codice 0 non gestisco nemmeno un lampeggio,
                        // Spengo il led ed esco.
                        LED2 = LED_OFF;
                        LED->phase = 4;
                    }
                    break;
        case 1  :   LED2 = LED_ON;
                    if(LED->timer > 0)
                    {   LED->timer--;
                    }
                    else
                    {   LED->timer   = LED->toff;
                        LED->phase = 2;
                    }
                    break;

        case 2  :   LED2 = LED_OFF;
                    if(LED->timer > 0)
                    {   LED->timer--;
                    }
                    else
                    {   LED->timer   = LED->toff;
                        LED->blinkDone++;   // Ciclo completo

                        if( LED->blinkDone < LED->errorCode)
                        {   // Devo eseguire ancora dei blink
                            LED->timer   = LED->ton;
                            LED->phase = 1;
                        }
                        else
                        {   // Ho completato i lampeggi, salto nella pausa
                            LED->timer   = LED->tpause;
                            LED->phase = 3;
                        }
                    }
                    break;

        case 3  :   LED2 = LED_OFF;
                    if(LED->timer > 0)
                    {   LED->timer--;
                    }
                    else
                    {   //Pausa terminata
                        LED->repetitionDone++;
                        // Verifico se devo ripetere ancora la sequenza
                        if( (LED->repetitionDone < LED->repetition) || (LED->repetition == 0) )
                        {   LED->phase = 0;
                            if(LED->repetition == 0)
                            {   // In loop infinito esco dallo stato di busy appena è stata
                                // completata una sequenza di segnalazione
                                LED->busy = 0;  // Posso gestire una nuova sequenza di segnalazione
                            }

                        }
                        else
                        {   LED->phase = 4;
                        }
                    }
                    break;

          case 4  : // Attendo all'infinito...
                    LED->busy = 0;  // Posso gestire una nuova sequenza di segnalazione
                    break;
    }
#endif
}


