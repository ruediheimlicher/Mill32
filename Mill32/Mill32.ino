///
/// @mainpage	Stepper32
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		06.05.2020 21:02
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2020
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Stepper32.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		06.05.2020 21:02
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2020
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#include "Arduino.h"

#include "gpio_MCP23S17.h"
#include <SPI.h>
#include "lcd.h"
#include "settings.h"
//#include <Wire.h>
#include <i2c_t3.h>
#include <LiquidCrystal_I2C.h> // auch in Makefile angeben!!!
#include <TeensyThreads.h>
// Set parameters


// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
uint8_t loopLED;
#define USB_DATENBREITE 64

#define TEST 0

int8_t r;

// USB
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
volatile uint16_t          usb_recv_counter=0;
volatile uint16_t          cnc_recv_counter=0;
// end USB


elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMillis sinceusb;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;

// Prototypes

static volatile uint8_t buffer[USB_DATENBREITE]={};
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

// Ringbuffer
uint8_t                    CNCDaten[RINGBUFFERTIEFE][USB_DATENBREITE];
uint8_t                    CDCStringArray[RINGBUFFERTIEFE];

volatile uint16_t          abschnittnummer=0;
volatile uint16_t          endposition= 0xFFFF;
volatile uint16_t           ladeposition=0;

volatile uint16_t          globalaktuelleladeposition = 0;
volatile uint8_t           ringbufferstatus=0x00;   

uint16_t                   Abschnitte=0;
uint16_t                   AbschnittCounter=0;
volatile uint8_t           liniencounter= 0;
// end Ringbuffer
volatile uint16_t           steps= 0;

volatile uint16_t           loadtime= 0;


volatile uint8_t           timer0startwert=TIMER0_STARTWERT;

volatile uint16_t          timer2Counter=0;
volatile uint8_t           cncstatus=0x00;
volatile uint8_t           sendstatus=0x00;


volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
static volatile uint8_t    anschlagstatus=0x00;

volatile uint8_t           timerstatus=0;

volatile uint8_t           status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;

// CNC

volatile uint16_t          CounterA=0;         // Zaehler fuer Delay von Motor A 
volatile uint16_t          CounterB=0;         // Zaehler fuer Delay von Motor B
volatile uint16_t          CounterC=0;         // Zaehler fuer Delay von Motor C 
volatile uint16_t          CounterD=0;         // Zaehler fuer Delay von Motor D

volatile uint32_t          DelayA=24;         // Delay von Motor A 
volatile uint32_t          DelayB=24;         // Delay von Motor B 
volatile uint32_t          DelayC=24;         // Delay von Motor C 
volatile uint32_t          DelayD=24;         // Delay von Motor D 

volatile uint32_t          StepCounterA=0;   // Zaehler fuer Schritte von Motor A 
volatile uint32_t          StepCounterB=0;   // Zaehler fuer Schritte von Motor B
volatile uint32_t          StepCounterC=0;   // Zaehler fuer Schritte von Motor C 
volatile uint32_t          StepCounterD=0;   // Zaehler fuer Schritte von Motor D

volatile uint8_t           richtung=0;

volatile uint8_t           parallelcounter=0;
volatile uint8_t           parallelstatus=0; // Status des Thread

volatile uint16_t          timerintervall = TIMERINTERVALL;
volatile uint16_t          timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t          timerintervall_FAST = 0; // Intervall gross


// Ramp

volatile uint16_t          ramptimerintervall = TIMERINTERVALL;

volatile uint8_t           rampstatus=0;
volatile uint8_t           RampZeit = RAMPZEIT;
volatile uint8_t           RampFaktor = RAMPFAKTOR;
volatile uint32_t          rampstepstart=0; // Stepcounter am Anfang
volatile uint32_t          ramptimercounter=0;  // laufender counter  fuer Rampanpassung
volatile uint32_t          ramptimerdelay = 200;  // Takt fuer Rampanpassung
volatile uint16_t          rampbreite = 0;  // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t          rampendstep = 0; // Beginn der Endtramp. Wird in Abschnittladen bestimmt


// Create an IntervalTimer object 
IntervalTimer              delayTimer;

// Utilities


// Functions

void OSZI_A_LO(void)
{
   if (TEST)
   digitalWriteFast(OSZI_PULS_A,LOW);
}

void OSZI_A_HI(void)
{
   if (TEST)
   digitalWriteFast(OSZI_PULS_A,HIGH);
}

void OSZI_A_TOGG(void)
{
   if (TEST)
   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZI_B_LO(void)
{
   if (TEST)
   digitalWriteFast(OSZI_PULS_B,LOW);
}

void OSZI_B_HI(void)
{
   if (TEST)
   digitalWriteFast(OSZI_PULS_B,HIGH);
}



void startTimer2(void)
{
   timerstatus |= (1<<TIMER_ON);
}
void stopTimer2(void)
{
   timerstatus &= ~(1<<TIMER_ON);
}

void timerfunction() 
{ 
   if (timerstatus & (1<<TIMER_ON))
   {
      if (PWM) // Draht soll heiss sein. 
      {
      }
      else
      {
         pwmposition =0;
      }
      
      //  if (timer2Counter >= 14) 
      {
         
         if(CounterA)
         {
            CounterA-=1;
         }
         if(CounterB)
         {
            CounterB-=1;
         }
         if(CounterC)
         {
            CounterC-=1;
         }
         if(CounterD)
         {
            CounterD-=1;
         }
         
         if (PWM)
         {
            pwmposition ++;
         }
         else
         {
            pwmposition =0;
         }
         
         //      timer2Counter = 0; 
         //OSZI_B_TOGG ;
      } 
   } // if timerstatus
//   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
  // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
}

void delaytimerfunction(void) // 1us ohne ramp
{ 
  // digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
   if (timerstatus & (1<<TIMER_ON))
   {
      //OSZI_A_LO();
      {
         
         if(CounterA)
         {
            CounterA-=1;
         }
         if(CounterB)
         {
            CounterB-=1;
         }
         if(CounterC)
         {
            CounterC-=1;
         }
         if(CounterD)
         {
            CounterD-=1;
         }
         
         if (PWM)
         {
            pwmposition ++;
         }
         else
         {
            pwmposition =0;
         }
         
         //      timer2Counter = 0; 
         //OSZI_B_TOGG ;
      } 
      //OSZI_A_HI();
      ramptimercounter += 1;
      
      rampstatus = 0;
      
      if (ramptimercounter > ramptimerdelay) // Teiler, 200us
      {
         ramptimercounter = 0;
         if (rampstatus & (1<<RAMPSTARTBIT))
         {
            if (ramptimerintervall > timerintervall_FAST) // noch nicht auf max speed
            {
             //  Serial.printf("start ramptimerintervall: %d\n",ramptimerintervall);
               ramptimerintervall -= RAMPSCHRITT;
               delayTimer.update(ramptimerintervall);
               rampbreite++;
            }
            else
            {
               
               rampstatus &= ~(1<<RAMPSTARTBIT);
               rampendstep = rampstepstart - max(StepCounterA, StepCounterB);
               rampstatus |= (1<<RAMPENDBIT);
               rampstatus |= (1<<RAMPEND0BIT);
              // Serial.printf("start rampstepstart: %d rampendstep: %d ramptimerintervall: %d timerintervall: %d\n",rampstepstart,rampendstep, ramptimerintervall,timerintervall);
               
            }
         }
         if (rampstatus & (1<<RAMPENDBIT))
         {
            
            if (max(StepCounterA, StepCounterB) < rampendstep)
            {
               //Serial.printf("end StepCounterA: %d\n",StepCounterA);
               // ramptimerintervall ist timerintervall_FAST
               if (rampstatus & (1<<RAMPEND0BIT))
                   {
                      //Serial.printf("rampend0:  rampendstep: %d StepCounterA: %d StepCounterB: %d ramptimerintervall: %d timerintervall: %d timerintervall_SLOW: %d\n",rampendstep, StepCounterA,StepCounterB,ramptimerintervall,timerintervall,timerintervall_SLOW);
                      
                      rampstatus &= ~(1<<RAMPEND0BIT);
                    }
               if (ramptimerintervall < timerintervall_SLOW)
               {
                  //Serial.printf("end StepCounterA: %d ramptimerintervall: %d\n",StepCounterA,ramptimerintervall);
                  ramptimerintervall += RAMPSCHRITT;
                  
                  delayTimer.update(ramptimerintervall);
                  //rampbreite++;
               }

            }
            
         } // if RAMPENDBIT
      }
      
      
   } // if timerstatus
   
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
    
}


uint8_t  AbschnittLaden_4M(const uint8_t* AbschnittDaten) // 22us
{
   //OSZI_A_LO();
   uint8_t l = sizeof(AbschnittDaten);
 //  Serial.printf("\n*********                   AbschnittLaden_4M:  AbschnittDaten len: %d Motor: %d\n\n", l,AbschnittDaten[28]);
   stopTimer2();
   uint8_t returnwert=0;
   parallelstatus  |= (1<<THREAD_COUNT_BIT);
//   Serial.printf("AbschnittDaten\n");
 //  for(int i=0;i<32;i++) // 5 us ohne printf, 10ms mit printf
   { 
 //         Serial.printf("%d \t",AbschnittDaten[i]);
    }
   //OSZI_A_HI();
   rampstatus = 0;
//   Serial.printf("\n            end Abschnittdaten\n");

#  pragma mark Reihenfolge der Daten
   /*         
    Rehenfolge der Daten   
    
    0   schrittexA
    1   schrittexB
    2   schrittexC
    3   schrittexD
    4   delayxA
    5   delayxB
    6   delayxC
    7   delayxD
    8   schritteyA
    9   schritteyB
    10   schritteyC
    11   schritteyD
    12   delayyA
    13   delayyB
    14   delayyC
    15   delayyD
    16   schrittezA
    17   schrittezB
    18   schrittezC
    19   schrittezD
    20   delayzA
    21   delayzB
    22   delayzC
    23   delayzD
    24   code
    25   position lage im Schnittpolygom
    26   indexh
    27   indexl
    */         
   int lage = 0;

   lage = AbschnittDaten[25]; // Start: 1, innerhalb: 0, Ende: 2
   if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
   // Motor A
//   
    
   StepCounterA = AbschnittDaten[0] | (AbschnittDaten[1]<<8) | (AbschnittDaten[2]<<16) | ((AbschnittDaten[3] & 0x7F)<<24);
   //uint32_t a  = AbschnittDaten[0] + (AbschnittDaten[1]<<8) + (AbschnittDaten[2]<<16) + ((AbschnittDaten[3] & 0x7F)<<24);
   //Serial.printf("l: %d\n",a);
   DelayA = AbschnittDaten[4] | (AbschnittDaten[5]<<8) | (AbschnittDaten[6]<<16) | ((AbschnittDaten[7] & 0x7F)<<24);
   
//   Serial.printf("StepCounterA: %d DelayA: %d\n",StepCounterA,DelayA);
//   Serial.printf("dataL: %d dataH: %d  delayL: %d delayH: %d\n",dataL,dataH, delayL,delayH);
   
   //lcd_gotoxy(17,0);
   if (AbschnittDaten[3] & 0x80) // Bit 7 gesetzt, negative zahl
   {
//      Serial.printf("rueckwaerts\n");
      richtung |= (1<<RICHTUNG_A); // Rueckwarts
      //STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
      digitalWriteFast(MA_RI, LOW);
      //lcd_putc('r');
   }
   else 
   {
//      Serial.printf("vorwaerts\n");
      richtung &= ~(1<<RICHTUNG_A);
      //STEPPERPORT_1 |= (1<< MA_RI);
      digitalWriteFast(MA_RI,HIGH);
      //lcd_putc('v');   // Vorwaerts
   }
 
   CounterA = DelayA;
   // STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   digitalWriteFast(MA_EN,LOW);

//   Serial.printf("CounterA: %d\n",CounterA);
   
   
   // Motor B
   //CounterB=0;
   //STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   digitalWriteFast(MB_EN,LOW);
 
   StepCounterB = AbschnittDaten[8] | (AbschnittDaten[9]<<8) | (AbschnittDaten[10]<<16) | ((AbschnittDaten[11] & 0x7F)<<24);
   DelayB= AbschnittDaten[12] | (AbschnittDaten[13]<<8) | (AbschnittDaten[14]<<16) | ((AbschnittDaten[15] & 0x7F)<<24);
//   Serial.printf("StepCounterB: %d DelayB: %d\n",StepCounterB,DelayB);

   if (AbschnittDaten[11] & 0x80) // Bit 7 gesetzt, negative zahl
   {
//      Serial.printf("rueckwaerts\n");
      richtung |= (1<<RICHTUNG_B); // Rueckwarts
      //STEPPERPORT_1 &= ~(1<< MB_RI);
      digitalWriteFast(MB_RI,LOW);
      //lcd_putc('r');
   }
   else 
   {
//      Serial.printf("vorwaerts\n");
      richtung &= ~(1<<RICHTUNG_B);
      //STEPPERPORT_1 |= (1<< MB_RI);
      digitalWriteFast(MB_RI,HIGH);
      //lcd_putc('v');
   }
    
       CounterB = DelayB;
       
       // Motor C
       //STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
       digitalWriteFast(MC_EN,LOW);
       //CounterC=0;
       StepCounterC = AbschnittDaten[16] | (AbschnittDaten[17]<<8) | (AbschnittDaten[18]<<16) | ((AbschnittDaten[19] & 0x7F)<<24);
       DelayC= AbschnittDaten[20] | (AbschnittDaten[21]<<8) | (AbschnittDaten[22]<<16) | ((AbschnittDaten[23] & 0x7F)<<24);
       if (AbschnittDaten[11] & 0x80) // Bit 7 gesetzt, negative zahl
           {
              richtung |= (1<<RICHTUNG_B); // Rueckwarts
              //STEPPERPORT_1 &= ~(1<< MB_RI);
              digitalWriteFast(MB_RI,LOW);
              //lcd_putc('r');
           }
           else 
           {
              richtung &= ~(1<<RICHTUNG_B);
              //STEPPERPORT_1 |= (1<< MB_RI);
              digitalWriteFast(MB_RI,HIGH);
              //lcd_putc('v');
           }
           
           CounterB = DelayB;
           
           //richtung=0;
   if (AbschnittDaten[19] & 0x80) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //STEPPERPORT_2 &= ~(1<< MC_RI);
      digitalWriteFast(MC_RI,LOW);
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //STEPPERPORT_2 |= (1<< MC_RI);
      digitalWriteFast(MC_RI,HIGH);
   }

   CounterC = DelayC;
   
   
   /* 
   // Motor D
   //STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
   digitalWriteFast(MD_EN,LOW);
   //CounterD=0;
   dataL=0;
   dataH=0;
   
   delayL = 0;
   delayH = 0;
   
   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      //STEPPERPORT_2 &= ~(1<< MD_RI);
      digitalWriteFast(MD_RI,LOW);
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_D);
      //STEPPERPORT_2 |= (1<< MD_RI);
      digitalWriteFast(MD_RI,HIGH);
   }
   
   dataH &= (0x7F);
   StepCounterD= dataH;      // HByte
   StepCounterD <<= 8;      // shift 8
   StepCounterD += dataL;   // +LByte
   
   delayL=AbschnittDaten[14];
   delayH=AbschnittDaten[15];
   
   DelayD = delayH;
   DelayD <<= 8;
   DelayD += delayL;
   
   CounterD = DelayD;
   
   // pwm-rate
   PWM = AbschnittDaten[20];
    */
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[28];
//   Serial.printf("*** Abschnittladen motorstatus %d\n",motorstatus); 
   
   if (motorstatus > 3)
   {
      Serial.printf("*** Abschnittladen motorstatus korr\n"); 
      //motorstatus = 1;
   }
 
#pragma mark Ramp
   // STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   digitalWriteFast(MA_EN,LOW);
   
   if (rampstatus > 0)
   {
      rampstepstart =  max(StepCounterA,StepCounterB); // maximalwert
      timerintervall_FAST = TIMERINTERVALL;
      timerintervall_SLOW = RampFaktor * TIMERINTERVALL; // Verlaengerung der delayzeit
      
      ramptimerintervall = timerintervall_SLOW;
      delayTimer.update(ramptimerintervall);
      
      if (rampstepstart > RampZeit) // 5mm
      {
         //rampendstep = max(StepCounterA,StepCounterB) -
         rampstatus |= (1<<RAMPSTARTBIT);
         Serial.printf("*** Ramp\n");
      }
      ramptimercounter = 0;
      rampbreite = 0;
   }
   startTimer2();
   
   Serial.printf("*                    *** Abschnittladen motorstatus %d StepCounterA: %d StepCounterB: %d \n",motorstatus,StepCounterA, StepCounterB);
   
   return returnwert;
 
   //OSZI_A_HI;
  
}


void AnschlagVonMotor(const uint8_t motor)
{
   // return;
   //lcd_gotoxy(0,1);
   //lcd_putc('A');
   //lcd_gotoxy(2+2*motor,1);
   //lcd_puthex(motor);
   Serial.printf("*** AnschlagvonMotor: %d\n",motor);
   
   uint8_t endPin;
   switch (motor)
   {
      case 0:
      {
         endPin = END_A0;
      }break;
      case 1:
      {
         endPin = END_B0;
      }break;
      case 2:
      {
         endPin = END_C0;
      }break;
      case 3:
      {
         endPin = END_D0;
      }break;
         
   }// switch motor
   
   if (digitalRead(endPin)) // Eingang ist HI, Schlitten nicht am Anschlag A0
   {
      if (anschlagstatus &(1<< endPin)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         anschlagstatus &= ~(1<< endPin); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {         
      //   AnschlagVonMotor(0); // Bewegung anhalten
      
      
      if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung ist auf Anschlag A0 zu         
      {
         if (!(anschlagstatus &(1<< (END_A0 + motor))))
         {
            //cli();
            anschlagstatus |= (1<< (END_A0 + motor));      // Bit fuer Anschlag A0+motor setzen
            //lcd_putc('A');
            if (cncstatus & (1<<GO_HOME)) // nur eigene Seite abstellen
            {
               // Paralleler Schlitten gleichzeitig am Anschlag?
               
               //lcd_putc('B');
               sendbuffer[0]=0xB5 + motor;
               
               cncstatus |= (1<<motor);
               
               if (motor<2) // Stepperport 1
               {
                  //STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
                  digitalWriteFast(MA_EN + motor,HIGH);
                  //STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
                  StepCounterA=0;
                  StepCounterB=0;
                  //               CounterA=0xFFFF;
                  //              CounterB=0xFFFF;
                  
               }
               else // Stepperport 2
               {
                  //    STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
                  digitalWriteFast(MC_EN + motor,HIGH);
                  //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
                  StepCounterC=0;
                  StepCounterD=0;
                  //               CounterC=0xFFFF;
                  //               CounterD=0xFFFF;
               }
               //cncstatus &= ~(1<<GO_HOME);
               
            }
            else           // beide Seiten abstellen
            {    
               cncstatus=0;
               sendbuffer[0]=0xA5 + motor;
               
               if (motor<2) // Stepperport 1
               {
                  //STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
                  digitalWriteFast(MA_EN + motor,HIGH);
                  //STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
                  digitalWriteFast(MA_EN + motor + 2,HIGH);
               }
               else // Stepperport 2
               {
                  //STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
                  digitalWriteFast(MC_EN + motor,HIGH);
                  //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
                  digitalWriteFast(MC_EN + motor + 2,HIGH);
               }
               
               // Alles abstellen
               StepCounterA=0;
               StepCounterB=0;
               StepCounterC=0;
               StepCounterD=0;
               
               /*
                CounterA=0xFFFF;
                CounterB=0xFFFF;
                CounterC=0xFFFF;
                CounterD=0xFFFF;
                */
               
               ladeposition=0;
               motorstatus=0;
               
            }
            
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;

            sendbuffer[8]=ladeposition & 0x00FF;
            //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
            sendbuffer[20]=cncstatus;
            usb_rawhid_send((void*)sendbuffer, 50);
             
            //ladeposition=0;
            // motorstatus=0;
            
            richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten
            
            interrupts();
         } // NOT END_A0
         else
         {
            
         }
      }
      else 
      {
         if (!(anschlagstatus &(1<< (END_A0 + motor))))
         {
            anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag B0 zuruecksetzen
         }
      }
   }
}







gpio_MCP23S17     mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20
//delay(1000); 
// Add setup code



void thread_func(int inc) 
{
   /*
   lcd.setCursor(0,0);
   lcd.print("A: ");
   
   lcd.setCursor(0,1);
   lcd.print("A:");
   lcd.setCursor(6,1);
   lcd.print("B:");
   
   lcd.setCursor(5,0);
   lcd.print("PWM:");
*/
   while (1)
   {
      if (parallelstatus & (1<<THREAD_COUNT_BIT))
      {
         parallelcounter += 1;
         
         lcd.setCursor(0,1);
         //String s = "D";
         lcd.setCursor(13,0);
         
         uint16_t rest = Abschnitte - AbschnittCounter;
         lcd.print(String(rest));
         
         
       //  s.append(rest);
         /*
         s.append("n ");
         
         s.append(StepCounterA);
         s.append("\n");
         
         lcd.print(s);
        */
         /*
         //lcd.print(String(parallelcounter));
         lcd.setCursor(9,0);
         lcd.print(String(PWM));
         lcd.setCursor(2,1);
         lcd.print(String(StepCounterA));
         lcd.setCursor(8,1);
         lcd.print(String(StepCounterB));
          */
         parallelstatus  &= ~(1<<THREAD_COUNT_BIT);
     }
   }
   /*
   if (sincelastthread >= 500)
   {
      sincelastthread = 0;
      parallelcounter += 2;
 //     lcd.setCursor(12,0);
  //    lcd.print(String(parallelcounter));
   }
   */
}



void setup()
{
   Serial.begin(9600);
   pinMode(LOOPLED, OUTPUT);


   pinMode(DC_PWM, OUTPUT);
   digitalWriteFast(DC_PWM, HIGH); // OFF
   
   pinMode(STROM, OUTPUT);
   digitalWriteFast(STROM, LOW); // LO, OFF
   
   
   // init Pins
   // Stepper A
   pinMode(MA_STEP, OUTPUT); // 
   pinMode(MA_RI, OUTPUT); // 
   pinMode(MA_EN, OUTPUT); // 

   digitalWriteFast(MA_STEP, HIGH); // HI
   digitalWriteFast(MA_RI, HIGH); // HI
   digitalWriteFast(MA_EN, HIGH); // HI
   
   
 
   
   // Stepper B
   pinMode(MB_STEP, OUTPUT); // HI
   pinMode(MB_RI, OUTPUT); // HI
   pinMode(MB_EN, OUTPUT); // HI
   
   digitalWriteFast(MB_STEP, HIGH); // HI
   digitalWriteFast(MB_RI, HIGH); // HI
   digitalWriteFast(MB_EN, HIGH); // HI
   
   
   
   // Stepper C
   pinMode(MC_STEP, OUTPUT); // HI
   pinMode(MC_RI, OUTPUT); // HI
   pinMode(MC_EN, OUTPUT); // HI
   
   digitalWriteFast(MC_STEP, HIGH); // HI
   digitalWriteFast(MC_RI, HIGH); // HI
   digitalWriteFast(MC_EN, HIGH); // HI
  
   // Stepper D
   pinMode(MD_STEP, OUTPUT); // HI
   pinMode(MD_RI, OUTPUT); // HI
   pinMode(MD_EN, OUTPUT); // HI
   
   digitalWriteFast(MD_STEP, HIGH); // HI
   digitalWriteFast(MD_RI, HIGH); // HI
   digitalWriteFast(MD_EN, HIGH); // HI
   
   pinMode(END_A0_PIN, INPUT); // 
   pinMode(END_B0_PIN, INPUT); // 
   pinMode(END_C0_PIN, INPUT); // 
   pinMode(END_D0_PIN, INPUT); // 

   pinMode(END_A0_PIN, INPUT_PULLUP); // HI
   pinMode(END_B0_PIN, INPUT_PULLUP); // 
   pinMode(END_C0_PIN, INPUT_PULLUP); // 
   pinMode(END_D0_PIN, INPUT_PULLUP); // 
   
   if (TEST)
   {
      pinMode(OSZI_PULS_A, OUTPUT);
      digitalWriteFast(OSZI_PULS_A, HIGH); 
   }
   
   delay(100);
   lcd.init();
   delay(100);
   lcd.backlight();
   
   //lcd.setCursor(0,0);
   //lcd.print("hallo");
   delayTimer.begin(delaytimerfunction,timerintervall);
   delayTimer.priority(0);
   
//   threads.addThread(thread_func, 1);
   
   lcd.setCursor(0,0);
   lcd.print("CNC");
//   lcd.setCursor(0,1);
//   lcd.print("PWM:");
   /*
   lcd.setCursor(0,1);
   lcd.print("A:");
   lcd.setCursor(6,1);
   lcd.print("B:");

   lcd.setCursor(5,0);
   lcd.print("PWM:");
*/
}

// Add loop code
void loop()
{
//   Serial.println(steps);
//   threads.delay(1000);
   
   
   if (sinceblink > 1000) 
   {  
      //scanI2C(100000);
      loopLED++;
      sinceblink = 0;
      
//      lcd.setCursor(0,1);
//      lcd.print(String(loopLED));

      if (digitalRead(LOOPLED) == 1)
      {
         
         //Serial.printf("LED ON\n");
         digitalWriteFast(LOOPLED, 0);
         /*
         //Serial.printf("blink\t %d\n",loopLED);
         lcd.setCursor(0,0);
         //lcd.print("hallo");
         lcd.print(String(timer2Counter));
         lcd.setCursor(10,0);
         lcd.print(String(usb_recv_counter));

         lcd.setCursor(16,0);
         lcd.print(String(abschnittnummer));
         lcd.setCursor(0,1);
         lcd.print(String(CounterA&0xFF));
         lcd.setCursor(4,1);
         lcd.print(String(CounterB&0xFF));
         */

      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
      }
      parallelcounter += 2;
//      lcd.setCursor(14,0);
//      lcd.print(String(parallelcounter));

   }// sinceblink
   
   if (sincelaststep > 500)
   {
      //Serial.printf("sincelaststep\n");
      sincelaststep = 0;
 //     timerfunction();
      
   //   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   }
   
   
   
   
   #pragma mark start_usb
   
   r = usb_rawhid_recv((void*)buffer, 0); // 1.5us
  
   if (r > 0) // 
   {
      noInterrupts();
      uint8_t code = 0x00;
      code = buffer[24];
  //    Serial.printf("----------------------------------->    rawhid_recv code: %02X\n",code);
      usb_recv_counter++;
 //     lcd.setCursor(10,1);
 //     lcd.print(String(usb_recv_counter));
 //     lcd.setCursor(14,1);
 //     lcd.print(String(code));
      switch (code)
      {   
      case 0xE0: // Man: Alles stoppen
         {
            Serial.printf("E0 Stop\n");
            ringbufferstatus = 0;
            motorstatus=0;
            anschlagstatus = 0;
            cncstatus = 0;
            sendbuffer[0]=0xE1;
            
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;

            sendbuffer[8]=ladeposition & 0x00FF;
           // sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
            
            
            
            usb_rawhid_send((void*)sendbuffer, 50);
            
            sendbuffer[0]=0x00;
            sendbuffer[5]=0x00;
            sendbuffer[6]=0x00;
            sendbuffer[8]=0x00;
            
            ladeposition=0;
            sendbuffer[8]=ladeposition;
            endposition=0xFFFF;
            
            AbschnittCounter=0;
            PWM = sendbuffer[29];
            //CMD_PORT &= ~(1<<DC_PWM);
            digitalWriteFast(DC_PWM,HIGH);
            
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            /*
            STEPPERPORT_1 |= (1<<MA_EN); // Pololu OFF
            STEPPERPORT_1 |= (1<<MB_EN); // Pololu OFF
            STEPPERPORT_2 |= (1<<MC_EN); // Pololu OFF
            STEPPERPORT_2 |= (1<<MD_EN); // Pololu OFF
            */
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            digitalWriteFast(MD_EN,HIGH);
            lcd.setCursor(0,1);
            lcd.print("HALT");
           // lcd_gotoxy(0,1);
           // lcd_puts("HALT\0");
            
         }break;
         
         
      case 0xE2: // DC_PWM ON_OFF: Temperatur Schneiddraht setzen
         {
            
            PWM = buffer[20];
            Serial.printf("E2 setPWM: %d\n",PWM);
            if (PWM==0) // OFF
            {
               //CMD_PORT &= ~(1<<DC_PWM);
              digitalWriteFast(DC_PWM,LOW);
            }
            parallelstatus |= (1<<THREAD_COUNT_BIT);
            
            sendbuffer[0]=0xE3;
  //          usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            //sendbuffer[5]=0x00;
            //sendbuffer[8]=0x00;
            
         }break;
         
         
      case 0xE4: // Stepperstrom ON_OFF
         {
            Serial.printf("E4 ON\n");
            if (buffer[8])
            {
               //CMD_PORT |= (1<<STROM); // ON
               digitalWriteFast(STROM,HIGH);
               PWM = buffer[29];
            }
            else
            {
               //CMD_PORT &= ~(1<<STROM); // OFF
               digitalWriteFast(STROM,LOW);
               PWM = 0;
            }
            
            if (PWM==0)
            {
               //CMD_PORT &= ~(1<<DC_PWM);
               digitalWriteFast(DC_PWM,LOW);
            }
            
            
            sendbuffer[0]=0xE5;
  //          usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            //sendbuffer[5]=0x00;
            //sendbuffer[6]=0x00;
            
         }break;
         
      case 0xF1: // reset
         {
            Serial.printf("F1 reset\n");
            uint8_t i=0, k=0;
            for (k=0;k<RINGBUFFERTIEFE;k++)
            {
               for(i=0;i<USB_DATENBREITE;i++)
               {
                  CNCDaten[k][i]=0;  
               }
            }
            
            ringbufferstatus = 0;
            motorstatus=0;
            anschlagstatus = 0;
            
            cncstatus = 0;
            ladeposition=0;
            endposition=0xFFFF;
            
            AbschnittCounter=0;
            PWM = 0;
            //CMD_PORT &= ~(1<<DC_PWM);
            digitalWriteFast(DC_PWM,LOW);
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            lcd.setCursor(0,1);
            lcd.print("reset\n");
            //cli();
            //usb_init();
            /*
             while (!usb_configured()) // wait  ;
             
             // Wait an extra second for the PC's operating system to load drivers
             // and do whatever it does to actually be ready for input
             _delay_ms(1000);
             */
            //sei();
            //sendbuffer[0]=0xF2;
            //usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            
         }break;
#pragma mark default
      default:
         {
  //          Serial.printf("\n---  usb_recv_counter %d\t default \nringbufferstatus: %02X position(buffer17): %02X\n",usb_recv_counter,ringbufferstatus, buffer[17]);
            // Abschnittnummer bestimmen
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            //OSZI_A_LO();
            Serial.printf("\n\ndefault abschnittnummer: %d  lage: %d code: %d\n",abschnittnummer,lage,code); // 50 us
            for(int i=0;i<32;i++)
            {
    //           Serial.printf("%d\t",buffer[i]);
            }
            Serial.printf("\n****************************************\n");
            Serial.printf("default Abschnitt lage: %d abschnittnummer: %d\n",lage,abschnittnummer);
            Serial.printf("****************************************\n");

    //        Serial.printf("\n");
            //Serial.printf("schritteX: \t%d \tschritteY: \t%d\n",schritteX,schritteY);
            //OSZI_A_HI();
   //         lcd.setCursor(5,1);
   //         lcd.print(String(abschnittnummer));
            

  //          sendbuffer[0]=0x33;
  //          sendbuffer[5]=abschnittnummer;
  //          sendbuffer[6]=buffer[16];
            
  //           usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
#pragma mark default abschnittnummer 0
            if (abschnittnummer==0)  // Start
            {
               noInterrupts();
               
                 
  //             Serial.printf("count: %d\n",buffer[22]);
   //            lcd.setCursor(12,0);
               //lcd.print("Abschnitt: ");
   //            Abschnitte = (buffer[26] << 8) | buffer[27] ;
               
    //           lcd.print(String(Abschnitte));
    //           lcd.setCursor(5,1);
               //lcd.print("Abschnitt: ");
               PWM= buffer[29];
 //              lcd.print(String(PWM));
               
               ladeposition=0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               ringbufferstatus=0x00;
               anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               //lcd_gotoxy(0,0);
               sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               sendbuffer[0]=0xD1;
               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
              
               /*
               if (code == 0xF0) // cncstatus fuer go_home setzen
               {
                  
                  sendbuffer[0]=0x45;
                  
                  cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                  usb_rawhid_send((void*)sendbuffer, 50);
               }
               else if (code == 0xF1)
               {
                  sendbuffer[0]=0x44;
                  cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                  usb_rawhid_send((void*)sendbuffer, 50);
               }
                */
               // Abschnitt 0 melden
               usb_rawhid_send((void*)sendbuffer, 50);
               
//               startTimer2();
//               interrupts();
               
            }
            else // Abschnittnummer > 0
            {
               // Ablauf schon gestartert
     //          Serial.printf("  -----                   Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
               //lcd.setCursor(12,0);
               //lcd.print(String(abschnittnummer));
            
            }
            
            //             if (buffer[9]& 0x02)// letzter Abschnitt
            
   //         Serial.printf("------------------------                buffer[17]: %d\n",buffer[17]);
            
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
                        
            // empfangene Daten vom buffer in CNCDaten laden
          //  if (abschnittnummer == 0)
            {
               
               uint8_t pos=(abschnittnummer);
               
               pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
               Serial.printf("default: load  CNC-Daten. abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
               //if (abschnittnummer>8)
               
               {
                  //lcd_putint1(pos);
               }
               
               // Daten laden in ringbuffer an Position pos
               uint8_t i=0;
               /*
                for(i=0;i<10;i++)
                {
                   Serial.printf("%d\t",i);
                }
                */
                //Serial.printf("\n");
                //OSZI_A_LO();
               Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
               //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
               for(i=0;i<32;i++) // 5 us ohne printf, 10ms mit printf
               { 
                  Serial.printf("%d \t",buffer[i]);

                  CNCDaten[pos][i]=buffer[i];  
               }
               //OSZI_A_HI();
               Serial.printf("\n");
               
            }
            
            
            // Erster Abschnitt, Beim Start naechsten Abschnitt ebenfalls laden, Anfrage an host
            
            
            /*
            if (ringbufferstatus & (1<<LASTBIT)) // nur 1 Abschnitt
                {
                   Serial.printf("erster Abschnitt nur 1 Abschnitt ladeposition: %d\n",ladeposition);
                   //lcd_putc('*');
                   
                   sendbuffer[5]=abschnittnummer;
             sendbuffer[6]=ladeposition & 0x00FF;
             sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                   sendbuffer[0]=0xD0; // end
                   uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 50);
                   Serial.printf("nur 1 Abschnitt senderfolg; %d\n",senderfolg);
                 
                }
            
            else 
             */
             
            if ((abschnittnummer == 0)&&(endposition)) // endposition ist > 0, weitere daten
                {
                   {
                      Serial.printf("erster Abschnitt, mehr Abschnitte ladeposition: %d endposition: %d\n",ladeposition,endposition);
                      //lcd_putc('*');
                      
                      sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                      sendbuffer[6]=abschnittnummer & 0x00FF;
                      
                      
                      sendbuffer[8]=ladeposition;
                      
                     sendbuffer[0]=0xAF; // next
         // AF nicht mehr gemeldet             
         //             uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 50);
 //                     Serial.printf("mehr Abschnitte senderfolg; %d\n",senderfolg);
                      sei();
                      //  sendbuffer[0]=0x00;
                      //  sendbuffer[5]=0x00;
                      //  sendbuffer[6]=0x00;
                      
                      
                   }  
                }
            
            ringbufferstatus &= ~(1<<FIRSTBIT);
  //          ringbufferstatus |= (1<<STARTBIT);
            // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
            Serial.printf("end USB abschnittnummer: %d ringbufferstatus: %d\n",abschnittnummer,ringbufferstatus);
            Serial.printf("\n****************************************\n");
            Serial.printf("Run Abschnitt %d\n",abschnittnummer);
            Serial.printf("****************************************\n");
            
            // if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
            if (((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
            
            {
               {
                  Serial.printf("abschnittnummer 1\n");
                  ringbufferstatus &= ~(1<<LASTBIT);
                  ringbufferstatus |= (1<<STARTBIT);
                  
               }
            }
            if (abschnittnummer == 0)
            {
//               startTimer2();
               //interrupts();

            }
            
         } // default
         
      } // switch code
      interrupts();
      code=0;
   }// r > 0
   /**   End USB-routinen   ***********************/
   
#pragma mark CNC-routinen   
   /*>
    #define STARTBIT   2       // Buffer ist geladen
    #define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
    #define LASTBIT   4         // Letzter Abschnitt  ist geladen
    #define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
    #define STOPBIT   6        // Ablauf stoppen
    #define FIRSTBIT   7

    */
   /**   Start CNC-routinen   ***********************/
   if (ringbufferstatus & (1<<STARTBIT)) // Buffer ist in Ringbuffer geladen, Schnittdaten von Abschnitt 0 laden
   {
      noInterrupts();
      
      Serial.printf("\n\n                 Abschnitt 0 laden ringbufferstatus: %d\n",ringbufferstatus);
      ringbufferstatus &= ~(1<<STARTBIT);  // Startbit entfernen      
      ladeposition=0;  // laufender Zaehler fuer Ringbuffer, gefiltert mit Ringbuffertiefe
      AbschnittCounter=0;
      
      // Abschnitt 0 laden
      uint8_t l = sizeof(CNCDaten[ladeposition]);
#pragma mark default Ersten Abschnitt laden
      Serial.printf("+++ Ersten Abschnitt laden AbschnittLaden_4M len: %d ringbufferstatus: %d\n",l,ringbufferstatus);
      uint8_t lage=AbschnittLaden_4M(CNCDaten[ladeposition]); // erster Wert im Ringbuffer
      
      ladeposition++;
      if (lage==2) // nur ein Abschnitt
      {
         Serial.printf("Abschnitt 0 laden nur 1 Abschnitt\n");
         ringbufferstatus |=(1<<ENDBIT); // unbenutzt
         ringbufferstatus |=(1<<LASTBIT);
      }
      AbschnittCounter+=1;
      
      interrupts();
//      startTimer2();
      
   }
   
#pragma mark Anschlag
   
   
   
   // Anschlagsituation abfragen
   
   // ********************
   // * Anschlag Motor A *
   // ********************
    //AnschlagVonMotor(0); 
   /*
   if (digitalRead(END_A0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A0
   {
      if (anschlagstatus &(1<< END_A0)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         anschlagstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {         
      AnschlagVonMotor(0); // Bewegung anhalten
   }
   */
   // **************************************
   // * Anschlag Motor B *
   // **************************************
   //AnschlagVonMotor(1);
   /*
   
   if (digitalRead(END_B0_PIN)) // Schlitten nicht am Anschlag B0
   {
      if (anschlagstatus &(1<< END_B0))
      {
         anschlagstatus &= ~(1<< END_B0); // Bit fuer Anschlag B0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag B0
   {
      AnschlagVonMotor(1);
   } // end Anschlag B0
   
   // End Anschlag B
   */
   
   // ********************
   // * Anschlag Motor C *
   // ********************
   //AnschlagVonMotor(2);
   /*
   // Anschlag C0
   if (digitalRead(END_C0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag C0
   {
      if (anschlagstatus &(1<< END_C0))
      {
         anschlagstatus &= ~(1<< END_C0); // Bit fuer Anschlag C0 zuruecksetzen
      }         
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag C0
   {
      AnschlagVonMotor(2);
   }
   */
   // ***************
   // * Anschlag Motor D *
   // ***************
   //AnschlagVonMotor(3);
   /*
   // Anschlag D0
   if (digitalRead(END_D0_PIN)) // Schlitten nicht am Anschlag D0
   {
      if (anschlagstatus &(1<< END_D0))
      {
         anschlagstatus &= ~(1<< END_D0); // Bit fuer Anschlag D0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag D0
   {
      AnschlagVonMotor(3);
   }
*/
#pragma mark Motor A    
   // Begin Motor A
   // **************************************
   // * Motor A *
   // **************************************
   /*
   if ((StepCounterA) && (StepCounterA%100 == 0))
   {
      Serial.printf("StepCounterA %d\n",StepCounterA);
      sendbuffer[0]=0xBA;
//      steps = StepCounterA%10;
      sendbuffer[7] = ((StepCounterA & 0xFF00) >> 8); // hi
      sendbuffer[8] = StepCounterA & 0x00FF; // lo
      usb_rawhid_send((void*)sendbuffer, 50);
      //Serial.printf("\nMotor A BA %d",StepCounterA);
   }
*/
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // Es hat noch Steps, CounterA ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if ((StepCounterA > 0)  && (CounterA == 0) &&(!(anschlagstatus & (1<< END_A0))))//||(cncstatus & (1<< END_B0)))))//   
   {
      
      noInterrupts();
      
      // Impuls starten
      //STEPPERPORT_1 &= ~(1<<MA_STEP);   // Impuls an Motor A LO -> ON
      digitalWriteFast(MA_STEP,LOW);
      CounterA = DelayA;                     // CounterA zuruecksetzen fuer neuen Impuls
 //     Serial.printf("Motor A StepCounterA: %d\n",StepCounterA);
//      Serial.printf("Motor A CounterA: %d StepCounterA: %d \n",CounterA, StepCounterA);
      if (StepCounterA > 0)
      {
         StepCounterA--; // ein Step weniger
      }
      
      // Wenn StepCounterA jetzt abgelaufen und relevant: next Datenpaket abrufen
      if ((StepCounterA == 0 ) && (motorstatus & (1<< COUNT_A)))    // StepCounterA abgelaufen, Motor A ist relevant fuer Stepcount
      {
         sendstatus |= (1<<COUNT_A);
         if (StepCounterB > 0) // Motor B ist noch nicht fertig
         {
            motorstatus |=  (1<< COUNT_B); // Relevanz zuteilen Motor B soll noch abzaehlen
            motorstatus &= ~(1<< COUNT_A); 
         }
         else
         {
            sendstatus |= (1<<COUNT_B); // Motor B auch markieren
         }
         Serial.printf("\nMotor A StepCounterA abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterB: %d sendstatus: %d\n", abschnittnummer, endposition, ringbufferstatus, StepCounterB, sendstatus);
         //        Serial.printf("Rampbreite: %d rampendstep: %d",rampbreite, rampendstep);
         //
         
         // Begin Ringbuffer-Stuff
         //if (ringbufferstatus & (1<<ENDBIT))
         
         if ((abschnittnummer==endposition))// && ((ringbufferstatus & (1<<ENDBIT)) || (ringbufferstatus & (1<<LASTBIT)))) // Ablauf fertig
         {  
            Serial.printf("-----------------------> Motor A endpos\n");
            noInterrupts();
            ringbufferstatus = 0;

            sendstatus |= (1<<COUNT_LAST);
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
   //         motorstatus=0;
            sendbuffer[0]=0xAD;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            sendbuffer[8]=ladeposition & 0x00FF;
            //         sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
 //           usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            interrupts();
         }
         else 
         {
            
            
            
          //  if ((sendstatus == 3) || (StepCounterB == 0))
   //         if (StepCounterB == 0)
            {
               uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
               
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               globalaktuelleladeposition = ladeposition;
               Serial.printf("\tMotor A globalaktuelleladeposition: %d\n",globalaktuelleladeposition);

               aktuelleladeposition &= 0x03;

               //          Serial.printf("\nMotor A StepCounterA abgelaufen aktuelleladeposition: %d\n", aktuelleladeposition);
               
               
               // aktuellen Abschnitt laden
               aktuellelage = CNCDaten[aktuelleladeposition][25];
          //     aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               //sendstatus = 0;
               //          }
               
               
               
               //          Serial.printf("+++ AbschnittLaden_4M Motor A aktuellelage: %d aktuelleladeposition: %d\n",aktuellelage, aktuelleladeposition);
               
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  
                  Serial.printf("\tMotor A last Abschnitt\n");
          //        
                  
                  
                  noInterrupts();
                  
                  endposition=abschnittnummer; // letzter Abschnitt
                  
                  /*
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                  sendbuffer[8]=ladeposition & 0x00FF;
                  //   sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  ringbufferstatus |= (1<<ENDBIT);
                  ringbufferstatus |= (1<<LASTBIT);
                  interrupts();
                  //endposition = 0xFFFF;
                  */
                  
               }
               else
               {
                  //  if (sendstatus == 3)
                  {
                     //            Serial.printf("Motor A neuen Abschnitt abrufen mit A1\n");
                     // neuen Abschnitt abrufen
                     Serial.printf("\tMotor A next abschnittnummer: %d\n",abschnittnummer);
                     
                     /*
                     sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
                     sendbuffer[6]=abschnittnummer & 0x00FF;
                     
                     sendbuffer[8]=ladeposition & 0x00FF;
                     //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                     sendbuffer[0]=0xA1;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     //sendstatus = 0;
                      */
                  }
               }
            
            
  //           ladeposition++;
  //             AbschnittCounter++;
            }
              
            Serial.printf("\tMotor A Abschnittcounter: %d\n",AbschnittCounter);
         }
         //         loadtime = sinceload - loadtime;
         //       Serial.printf("%d\n",loadtime);
         
      }
      
      interrupts();
   }
   else
   {
      //Serial.printf("A");
      //STEPPERPORT_1 |= (1<<MA_STEP);               // Impuls an Motor A HI -> OFF
   //forever
 //    Serial.printf("Impuls an Motor A HI -> OFF\n");
      //OSZI_A_LO();
      if (digitalReadFast(MA_STEP) == 0) //100 ns
      {
         digitalWriteFast(MA_STEP,HIGH);
         
         if (StepCounterA ==0)                     // Keine Steps mehr fuer Motor A
         {
            //OSZI_A_LO();
            //forever
 //           Serial.printf("Motor A OFF\n");
            //STEPPERPORT_1 |= (1<<MA_EN);                     // Motor A ist OFF
            digitalWriteFast(MA_EN,HIGH);
            //OSZI_A_HI();
         }
      }
      //OSZI_A_HI();
      interrupts();
   }
   
 #pragma mark Motor B
   // **************************************
   // * Motor B *
   // **************************************
   
   if ((StepCounterB > 0) && (CounterB == 0)&&(!(anschlagstatus & (1<< END_B0))))
   {
      //Serial.printf("Motor B");
      noInterrupts();
      //lcd_putc('B');
      
      //STEPPERPORT_1 &= ~(1<<MB_STEP);               // Impuls an Motor B LO ON
      digitalWriteFast(MB_STEP,LOW);
      
      CounterB= DelayB;
      if (StepCounterB > 0)
      {
         StepCounterB--;
      }
      if ((StepCounterB == 0)  && (motorstatus & (1<< COUNT_B))) // StepCounterB abgelaufen, Motor B ist relevant fuer Stepcount 
      {
         sendstatus |= (1<<COUNT_B);
         
         if (StepCounterA > 0)
         {
            motorstatus |=  (1<< COUNT_A);
            motorstatus &= ~(1<< COUNT_B);
         }
         else
         {
            sendstatus |= (1<<COUNT_A); // Motor A abgelaufen, auch markieren
         }

         
         //            STEPPERPORT_1 |= (1<<MB_EN);               // Motor B OFF
         Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterA: %d sendstatus: %d\n", abschnittnummer, endposition, ringbufferstatus,StepCounterA,sendstatus);
         
         
         //StepCounterA=0;
         //lcd_putc('-');
         // Begin Ringbuffer-Stuff
         
         if (abschnittnummer==endposition)
         {  
            //cli();
            Serial.printf("-----------------------> Motor B endpos\n");
            
            ringbufferstatus = 0;
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
       //     motorstatus=0;
 
            sendstatus |= (1<<COUNT_LAST); // beenden abzeigeb

            sendbuffer[0]=0xAD;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            sendbuffer[8]=ladeposition & 0x00FF;
 //           usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            
         }
         else 
         { 
            uint8_t aktuellelage=0;
            uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
            aktuelleladeposition &= 0x03;
            
            // aktuellen Abschnitt laden
            aktuellelage = CNCDaten[aktuelleladeposition][25];
            globalaktuelleladeposition = ladeposition;
            Serial.printf("\tMotor B  globalaktuelleladeposition: %d\n",globalaktuelleladeposition);
            //aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
            //               Serial.printf("+++ AbschnittLaden_4M Motor B aktuellelage: %d aktuelleladeposition: %d\n",aktuellelage, aktuelleladeposition);
            //sendstatus = 0;
            
            
            if (aktuellelage==2) // war letzter Abschnitt
            {
               endposition=abschnittnummer; // letzter Abschnitt
               Serial.printf("\tMotor B last Abschnitt\n");
              // sendstatus |= (1<<COUNT_LAST);
               
               
               // Neu: letzten Abschnitt melden
               /*
               sendbuffer[0]=0xD0;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               sendbuffer[8]=ladeposition & 0x00FF;
               //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
               usb_rawhid_send((void*)sendbuffer, 50);
               */
               
            }  
            else 
            {
               {
                  Serial.printf("\tMotor B next abschnittnummer: %d\n",abschnittnummer);
                  /*
                  // neuen Abschnitt abrufen
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                  
                  sendbuffer[8]=ladeposition & 0x00FF;
                  //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[0]=0xA1;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  */
               }
            }
            
//            ladeposition++;
//            AbschnittCounter++;
              Serial.printf("\tMotor B Abschnittcounter: %d\n",AbschnittCounter);
         }
          
      }
      
      
      interrupts();
   }
   else// if (CounterB)
   {
      //STEPPERPORT_1 |= (1<<MB_STEP);
      digitalWriteFast(MB_STEP,HIGH);
      if (StepCounterB ==0)                     // Keine Steps mehr fuer Motor B
      {
         //Serial.printf("Motor B OFF\n");
         //STEPPERPORT_1 |= (1<<MB_EN);               // Motor B OFF
         digitalWriteFast(MB_EN,HIGH);
      
      }
      
      interrupts();
      
   }
   
   
   // End Motor B
   
   // Begin Motor C
#pragma mark Motor C
   // **************************************
   // * Motor C *
   // **************************************
   
   // Es hat noch Steps, CounterC ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if (StepCounterC &&(CounterC == 0) &&(!(anschlagstatus & (1<< END_C0))))//||(cncstatus & (1<< END_D0)))))//   
   {
      noInterrupts();
      // Impuls starten
      //STEPPERPORT_2 &= ~(1<<MC_STEP);   // Impuls an Motor C LO -> ON
      digitalWriteFast(MC_STEP,LOW);
      CounterC=DelayC;                     // CounterC zuruecksetzen fuer neuen Impuls
      
      
      StepCounterC--;
      
      // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
      if (StepCounterC ==0 && (motorstatus & (1<< COUNT_C)))    // Motor A ist relevant fuer Stepcount 
      {
         
         //            STEPPERPORT_2 |= (1<<MC_EN);                          // Motor C OFF
         //StepCounterD=0; 
         
         // Begin Ringbuffer-Stuff
         //if (ringbufferstatus & (1<<ENDBIT))
         if (abschnittnummer==endposition)
         {  
            //cli();
            Serial.printf("\nMotor C endpos\n");
             sendbuffer[0]=0xAD;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;

            sendbuffer[8]=ladeposition;
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
            motorstatus=0;
           
            
            usb_rawhid_send((void*)sendbuffer, 50);
            ringbufferstatus = 0;

            ladeposition=0;
            interrupts();
         }
         else 
         { 
            uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
            {
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               if (ladeposition>8)
               {
                  //lcd_putint1(ladeposition);
               }
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;

                  sendbuffer[8]=ladeposition & 0x00FF;
                  //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
                  
               }  
               else
               {
                  // neuen Abschnitt abrufen
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;

                  sendbuffer[8]=ladeposition & 0x00FF;
            //      sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[0]=0xA2;
                  usb_rawhid_send((void*)sendbuffer, 50);  
                  
               }
               
               ladeposition++;
               
            }
            
            
            AbschnittCounter++;
            
         }
         
      }
      
      interrupts();
   }
   else
   {
      //STEPPERPORT_2 |= (1<<MC_STEP);               // Impuls an Motor C HI -> OFF
      digitalWriteFast(MC_STEP,HIGH);
      if (StepCounterC ==0)                     // Keine Steps mehr fuer Motor C
      {
         //STEPPERPORT_2 |= (1<<MC_EN);                     // Motor C OFF
         digitalWriteFast(MC_EN,HIGH);
      }
   }
   
#pragma mark Motor D
   // **************************************
   // * Motor D *
   // **************************************
   
   if (StepCounterD && (CounterD == 0)&&(!(anschlagstatus & (1<< END_D0))))
   {
      noInterrupts();
      
      //STEPPERPORT_2 &= ~(1<<MD_STEP);               // Impuls an Motor D LO: ON
      digitalWriteFast(MD_STEP,LOW);
      CounterD= DelayD;
      StepCounterD--;
      
      if (StepCounterD ==0 && (motorstatus & (1<< COUNT_D))) // Motor D ist relevant fuer Stepcount 
      {
         //            STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
         
         //StepCounterC=0;
         // Begin Ringbuffer-Stuff
         if (abschnittnummer==endposition)
         {  
            //cli();
            Serial.printf("\nMotor D endpos\n");
            ringbufferstatus = 0;
            cncstatus=0;
            motorstatus=0;
            sendbuffer[0]=0xAD;
            sendbuffer[1]=abschnittnummer;
            usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            
         }
         else 
         { 
            uint8_t aktuellelage=0;
            {
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;

                  sendbuffer[8]=ladeposition & 0x00FF;
              //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
                  
               }  
               else
               {
                  // neuen Abschnitt abruffen
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                 
                  sendbuffer[8]=ladeposition & 0x00FF;
               //   sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[0]=0xA3;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
               }
               
               ladeposition++;
               
            }
            AbschnittCounter++;
            
         }
      }

      
      interrupts();
   }
   else// if (CounterB)
   {
      //STEPPERPORT_2 |= (1<<MD_STEP);
      digitalWriteFast(MD_STEP,HIGH);
      if (StepCounterD ==0)                     // Keine Steps mehr fuer Motor D
      {
         //STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
         digitalWriteFast(MD_EN,HIGH);
      }
       
   }
   
#pragma mark sendstatus
   //if (sendstatus >= 3)
   if (sendstatus > 0)
   {
      
      Serial.printf("\n++++++++++++++++++++++++++++++\nsendstatus: %d abschnittnummer: %d endposition: %d globalaktuelleladeposition: %d: StepCounterA: %d StepCounterB: %d\n", sendstatus,abschnittnummer,endposition,globalaktuelleladeposition,StepCounterA, StepCounterB);
      if ((sendstatus == 3) ) //|| ((StepCounterA == 0) && (StepCounterB == 0)) )
      {
         if (sendstatus == 3)
         {
            Serial.printf("\tsendstatus 3\n");
         }
         else 
         {
            Serial.printf("\tsendstatus (StepCounterA == 0) && (StepCounterB == 0): sendstatus: %d\n",sendstatus);
         }
         
         Serial.printf("\nabschnittnummer: %d endposition: %d \n",abschnittnummer, endposition);
         if (abschnittnummer == endposition)
         {
            Serial.printf("\n****************************************\n");
            Serial.printf("\n sendstatus 3  abschnittnummer = endposition\n");
            Serial.printf("\n****************************************\n");
         }
         
         motorstatus=0;
         sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
         sendbuffer[6]=abschnittnummer & 0x00FF;
         
         sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
         //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
         
         
         {
            sendbuffer[0]=0xA1;
            usb_rawhid_send((void*)sendbuffer, 50);
            
   /*         
            if (abschnittnummer == endposition)
            {
      //         Serial.printf("\tsendstatus LAST setzen\n\n");
      //         sendstatus |= (1<<COUNT_LAST);
            }
            else
    */
            {
               Serial.printf("\tsendstatus next Abschnitt\n\n");
            uint8_t aktuellelage = AbschnittLaden_4M(CNCDaten[(globalaktuelleladeposition & 0x03)]);
               
            Serial.printf("\n****************************************\n");
            Serial.printf("Load Abschnitt %d\n",globalaktuelleladeposition);
            Serial.printf("****************************************\n");
            
            Serial.printf("\n\tsendstatus next Abschnitt geladen aktuellelage: \n",aktuellelage);
            ladeposition++;
               
            AbschnittCounter=0;
            }
            /*
             if (abschnittnummer == endposition)
             {
             sendbuffer[0]=0xD0;
             sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
             sendbuffer[6]=abschnittnummer & 0x00FF;
             
             sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
             //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
             usb_rawhid_send((void*)sendbuffer, 50);
             
             }
             */
         }
         
      }
      
      if ((sendstatus & (1<<COUNT_LAST)) && ((StepCounterA == 0) && (StepCounterB == 0)))
      {
         Serial.printf("\t+++   sendstatus last   +++\n");
         Serial.printf("\t COUNT LAST StepCounterA: %d StepCounterB: %d\n", StepCounterA, StepCounterB);
         if (abschnittnummer == endposition)
         {
            Serial.printf("\n****************************************\n");
            Serial.printf("\n sendstatus COUNT_LAST  abschnittnummer = endposition\n");
            Serial.printf("\n****************************************\n");
         }

         //       sendbuffer[0]=0xD0;
         //      motorstatus=0;
         sendbuffer[0]=0xAD;
         sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
         sendbuffer[6]=abschnittnummer & 0x00FF;
         
         sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
         //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
         usb_rawhid_send((void*)sendbuffer, 50);
         
      }
      // ladeposition++;
      AbschnittCounter++;
      sendstatus = 0;
   }
   
   if (sendstatus)
   {
 //     Serial.printf("\nsendstatus: %d abschnittnummer: %d globalaktuelleladeposition: %d\n", sendstatus,abschnittnummer,globalaktuelleladeposition);
 //     sendstatus = 0;
   }

  interrupts();
   // End Motor D
   
   
 
   
   
   

   /**   End CNC-routinen   ***********************/
}
