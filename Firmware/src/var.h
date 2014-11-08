#ifdef VAR_INC

/*
 *     NOTE: Relation Scheme from LogicName ( LEFT/RIGHT ) and peripherical
 *           name "1" or "2"
 *
 *     TIMER2 <=> IC1 <=> QEI1 <=> LEFT
 *     TIMER3 <=> IC2 <=> QEI2 <=> RIGHT
 *
 */

const long              Tcy = 1000000/(float)(FCY)* 100000000;

//ADC
unsigned int DmaBuffer;
volatile fvalue CostanteTaraturaAN0;
volatile fvalue CostanteTaraturaAN1;
volatile fvalue CostanteTaraturaAN2;
volatile fvalue CostanteTaraturaAN3;
volatile fvalue CostanteTaraturaAN4;

/* PID */
volatile Pid_t   PID1;
volatile Pid_t   PID2;

unsigned char SampleICSwitcher;

// Timer software
volatile Timer_t        Timer10mSec;
volatile Timer_t        Timer1mSec;

volatile Motor_t        Motore1;
volatile Motor_t        Motore2;

//! Struttura usata per la gestione dell'InputCapture e le misure di velocità
volatile InputCapture_t InputCapture1;
volatile InputCapture_t InputCapture2;

/* LED SEGNALAZIONE ERRORI*/
volatile Led_t  Led1Segnalazione;
volatile Led_t  Led2Segnalazione;
volatile Bit_t  LedStatusSignal;

// Strutture
volatile Bit_t VOLbits1;
Bit_t Old_Indice_Status_Bit1;
Bit_t VARbits1;

// Configurazioni/tarature
unsigned int ParametriEEPROM[NUMERO_PARAMETRI_EEPROM];
unsigned int IngressiAnalogici[NUMERO_INGRESSI_ANALOGICI];

// UART1 & UART2
volatile unsigned int ComunicationWatchDogTimer;
volatile int Uart1RxStatus =0;			//!< index for command decoding status
unsigned char ChkSum1=0;			//!< checksum
volatile int Uart2RxStatus =0;			//!< index for command decoding status
unsigned char ChkSum2=0;                        //!< checksum
volatile unsigned char TxNByte_UART2 = 0;                  // Trasmissione byte senza DMA
unsigned char *TxPointer_UART2;

//  MODBUS
volatile lvalue TmpSplitLongToWord;
unsigned int    VarModbus[N_PARAMETRI];
unsigned int    StatoSeriale[NUMERO_PORT_SERIALI];
unsigned char   RxNByte[NUMERO_PORT_SERIALI];
unsigned char   *RxPointer[NUMERO_PORT_SERIALI];
unsigned char   *TxPointer[NUMERO_PORT_SERIALI];
unsigned char   TimerOutRxModbus[NUMERO_PORT_SERIALI], TimerRitardoModbus[NUMERO_PORT_SERIALI];
unsigned char   TxComplete[NUMERO_PORT_SERIALI];

// DMA buffers
unsigned char Uart1TxBuff[MAX_TX_BUFF] __attribute__((space(dma),aligned(128))); //!< TX Buffer for Serial Port 1
//unsigned char Uart2TxBuff[MAX_TX_BUFF] __attribute__((space(dma),aligned(128))); //!< TX Buffer for Serial Port 2

unsigned int DmaAdc_A[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));
unsigned int DmaAdc_B[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));

// Buffer per protocollo modbus
const char TabMaskBitModbus[8]      =   {   0x01,   0x02,   0x04,   0x08,   0x10,   0x20,   0x40,   0x80    };
const unsigned int TabMaskBitIO[]   = 	{   0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                                            0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000	};

unsigned char ModbusTxBuff[NUMERO_PORT_SERIALI][MODBUS_N_BYTE_TX];  //!< TX Buffer for modbus packet
unsigned char ModbusRxBuff[NUMERO_PORT_SERIALI][MODBUS_N_BYTE_RX];  //!< RX Buffer for modbus packet
volatile unsigned char UartRxBuff[MAX_RX_BUFF][2];//serial communication buffer

// Rad to nSec conversion
long E1, E2; // E = [( 2π * 10^9 ) / EncoderResolution]


#else
    // ridefinisco le variabili come extern

//ADC
extern unsigned int DmaBuffer;
extern volatile fvalue CostanteTaraturaAN0;
extern volatile fvalue CostanteTaraturaAN1;
extern volatile fvalue CostanteTaraturaAN2;
extern volatile fvalue CostanteTaraturaAN3;
extern volatile fvalue CostanteTaraturaAN4;

/* PID */
extern volatile Pid_t   PID1;
extern volatile Pid_t   PID2;

extern unsigned char SampleICSwitcher;
// Timer software
extern volatile Timer_t    Timer10mSec;
extern volatile Timer_t    Timer1mSec;

extern volatile Motor_t        Motore1;
extern volatile Motor_t        Motore2;

extern volatile Led_t  Led1Segnalazione;
extern volatile Led_t  Led2Segnalazione;
extern volatile Bit_t  LedStatusSignal;

// Strutture
extern volatile Bit_t VOLbits1;
extern Bit_t Old_Indice_Status_Bit1;
extern Bit_t VARbits1;

// Configurazioni/tarature
extern unsigned int ParametriEEPROM[NUMERO_PARAMETRI_EEPROM];
extern unsigned int IngressiAnalogici[NUMERO_INGRESSI_ANALOGICI];

// UART1 & UART2
extern volatile unsigned int ComunicationWatchDogTimer;
extern volatile int Uart1RxStatus;			//!< index for command decoding status
extern unsigned char ChkSum1;			//!< checksum
extern volatile int Uart2RxStatus;			//!< index for command decoding status
extern unsigned char ChkSum2;                        //!< checksum
extern volatile unsigned char TxNByte_UART2;                  // Trasmissione byte senza DMA
extern unsigned char *TxPointer_UART2;

//  MODBUS
extern volatile lvalue TmpSplitLongToWord;

extern unsigned int    VarModbus[N_PARAMETRI];
extern unsigned int    StatoSeriale[NUMERO_PORT_SERIALI];
extern unsigned char   RxNByte[NUMERO_PORT_SERIALI];
extern unsigned char   *RxPointer[NUMERO_PORT_SERIALI];
extern unsigned char   *TxPointer[NUMERO_PORT_SERIALI];
extern unsigned char   TimerOutRxModbus[NUMERO_PORT_SERIALI], TimerRitardoModbus[NUMERO_PORT_SERIALI];
extern unsigned char   TxComplete[NUMERO_PORT_SERIALI];

// DMA buffers
extern unsigned char Uart1TxBuff[MAX_TX_BUFF] __attribute__((space(dma),aligned(128))); //!< TX Buffer for Serial Port 1
//extern unsigned char Uart2TxBuff[MAX_TX_BUFF] __attribute__((space(dma),aligned(128))); //!< TX Buffer for Serial Port 2

extern unsigned int DmaAdc_A[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));
extern unsigned int DmaAdc_B[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(128)));

// Buffer per protocollo modbus
extern const char TabMaskBitModbus[8]; //      =   {   0x01,   0x02,   0x04,   0x08,   0x10,   0x20,   0x40,   0x80    };
extern const unsigned int TabMaskBitIO[]; //   = 	{   0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                                          //  0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000	};

extern unsigned char ModbusTxBuff[NUMERO_PORT_SERIALI][MODBUS_N_BYTE_TX];  //!< TX Buffer for modbus packet
extern unsigned char ModbusRxBuff[NUMERO_PORT_SERIALI][MODBUS_N_BYTE_RX];  //!< RX Buffer for modbus packet
extern volatile unsigned char UartRxBuff[MAX_RX_BUFF][2];//serial communication buffer

extern long E1, E2; // E = [( 2π * 10^9 ) / EncoderResolution]
#endif

