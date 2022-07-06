

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
//#include <util/delay.h>

#include <Wire.h>
//#include <i2c_t3.h>
#include <LiquidCrystal_I2C.h> // auch in Makefile angeben!!!
//#include <TeensyThreads.h>
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

volatile uint8_t buffer[USB_DATENBREITE]={};
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

// Ringbuffer
uint8_t                    CNCDaten[RINGBUFFERTIEFE][USB_DATENBREITE];
uint8_t                    CDCStringArray[RINGBUFFERTIEFE];

volatile uint16_t          abschnittnummer=0;
volatile uint16_t          endposition= 0xFFFF;
volatile uint16_t           ladeposition=0;

//volatile uint16_t          globalaktuelleladeposition = 0;
volatile uint16_t          aktuelleladeposition = 0;
volatile uint8_t           ringbufferstatus=0x00;   

uint16_t                   Abschnitte=0;
uint16_t                   AbschnittCounter=0;
volatile uint8_t           liniencounter= 0;
// end Ringbuffer
volatile uint16_t           steps= 0;

volatile uint16_t korrekturintervallx = 0;
volatile uint16_t korrekturintervally = 0;

volatile uint16_t korrekturintervallcounterx = 0;
volatile uint16_t korrekturintervallcountery = 0;

volatile uint16_t korrekturcounterx = 0;
volatile uint16_t korrekturcountery = 0;


volatile uint8_t vorzeichen = 0;



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
volatile uint8_t           homestatus=0;

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

volatile uint32_t          rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt


// Create an IntervalTimer object 
IntervalTimer              delayTimer;

// Utilities
// Ganssle


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
     
      {
         ramptimercounter += 1;
         
         //      Serial.printf("start rampstatus: %d",rampstatus);
         //      rampstatus = 0;
         
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
      }
      
   } // if timerstatus
   
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
    
}


uint8_t  AbschnittLaden_4M(const uint8_t* AbschnittDaten) // 22us
{
   stopTimer2();
 //  lcd_gotoxy(15,0);
 //  lcd_puts("    ");
   
   uint8_t returnwert=0;
// MARK: mark Reihenfolge der Daten
   /*         
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    
    22   Strom ON
    */         
   int lage = 0;
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
   Serial.printf("\n\nAbschnittLaden_4M lage: %d\n",lage);
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
   digitalWriteFast(MA_EN,LOW); // Pololu ON
   
   uint8_t dataL=0;
   uint8_t dataH=0;
   
   uint8_t delayL=0;
   uint8_t delayH=0;
   
   dataL=AbschnittDaten[0];
   dataH=AbschnittDaten[1];
   
   //lcd_gotoxy(17,0);
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_A); // Rueckwarts
      digitalWriteFast(MA_RI, LOW); // PIN fuer Treiber stellen
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_A);
      digitalWriteFast(MA_RI, HIGH);
      //lcd_putc('v');   // Vorwaerts
   }
   
   dataH &= (0x7F);
   StepCounterA = dataH;      // HByte
   StepCounterA <<= 8;        // shift 8
   StepCounterA += dataL;     // +LByte
   
   delayL=AbschnittDaten[4];
   delayH=AbschnittDaten[5];
   
   
   DelayA = delayH;
   DelayA <<= 8;
   DelayA += delayL;
   
   CounterA = DelayA;
   
   // Motor B
   //CounterB=0;
   digitalWriteFast(MB_EN,LOW);   
   dataL=AbschnittDaten[2];
   dataH=AbschnittDaten[3];
   //lcd_gotoxy(19,1);
   
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_B); // Rueckwarts
      digitalWriteFast(MB_RI, LOW);      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_B);
      digitalWriteFast(MB_RI, LOW);
      //lcd_putc('v');
   }
   
   dataH &= (0x7F);
   StepCounterB = dataH;      // HByte
   StepCounterB <<= 8;      // shift 8
   StepCounterB += dataL;   // +LByte
   
   DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
   CounterB = DelayB;
   
   // Motor C
   digitalWriteFast(MC_EN,LOW);    // Pololu ON
   //CounterC=0;
   dataL=0;
   dataH=0;
   
   delayL=0;
   delayH=0;
   
   dataL=AbschnittDaten[8];
   dataH=AbschnittDaten[9];
   
   //richtung=0;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      digitalWriteFast(MC_RI, LOW);
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      digitalWriteFast(MA_RI, HIGH);
   }
   
   dataH &= (0x7F);
   StepCounterC = dataH;      // HByte
   StepCounterC <<= 8;      // shift 8
   StepCounterC += dataL;   // +LByte
   
   
   delayL=AbschnittDaten[12];
   delayH=AbschnittDaten[13];
   
   DelayC = delayH;
   DelayC <<=8;
   DelayC += delayL;
   
   CounterC = DelayC;

   
   // Motor D
   digitalWriteFast(MD_EN,LOW);   //CounterD=0;
   dataL=0;
   dataH=0;
   
   delayL = 0;
   delayH = 0;
   
   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      digitalWriteFast(MD_RI, LOW);
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_D);
      digitalWriteFast(MD_RI, HIGH);
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
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   
   startTimer2();
   
   Serial.printf("\n\nAbschnittLaden_4M end\n\n");
    return returnwert;
 
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
//            usb_rawhid_send((void*)sendbuffer, 50);
             
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
   
//   rampstatus |=(1<<RAMPOKBIT);
   
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
      
      usb_recv_counter++;
      //     lcd.setCursor(10,1);
      //     lcd.print(String(usb_recv_counter));
      //     lcd.setCursor(14,1);
      //     lcd.print(String(code));
      uint8_t device = buffer[32];
      
      Serial.printf("----------------------------------->    rawhid_recv code: %02X device: %d\n",code, device);
      for(uint8_t i=0;i<36;i++) // 5 us ohne printf, 10ms mit printf
      { 
 //        Serial.printf("%d %d \n",i,buffer[i]);
      }

      Serial.printf("\n");
      
      sendbuffer[24] =  buffer[32];
      switch (code)
      {   
         case 0xA4:
         {
            Serial.printf("A4 clear\n");
         }break;
            
#pragma mark A5           
         case 0xA5: // 
         {
    //        Serial.printf("A5\n");
            uint8_t i=0;
            for(i=0;i<33;i++) // 5 us ohne printf, 10ms mit printf
            { 
 //              Serial.printf("%d \t",buffer[i]);
            }
            Serial.printf("\n");
            //Serial.printf("A5 setStepCounter\n");
            if (StepCounterA)
            {
               uint8_t vorzeichenx = buffer[4];
               uint16_t dx = buffer[0] | ((buffer[1]<<8) & 0x7F);
               //StepCounterA += dx;
               // Serial.printf("buffer0: %d buffer1: %d\n",buffer[0],buffer[1]);
     //          Serial.printf("setStepCounter dx: %d vorzeichen: %d\n",dx,vorzeichenx);
            }
            if (StepCounterB)
            {
               uint8_t vorzeicheny = buffer[12];
               uint16_t dy = buffer[8] | ((buffer[9]<<8) & 0x7F);
               //StepCounterB += dy;
    //           Serial.printf("setStepCounter dy: %d vorzeichen: %d\n",dy,vorzeicheny);
            }
            AbschnittLaden_4M(buffer);
         }break;
#pragma mark B1    
         case 0xB1:
         {
            Serial.printf("B1 PCB\n");
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            Serial.printf("B1 abschnittnummer: %d\n",abschnittnummer);
            
            AbschnittLaden_4M(buffer);
            if (abschnittnummer == 0)
            {
               
            }
            
            sendbuffer[8]=ladeposition & 0x00FF;
            sendbuffer[8]=ladeposition & 0x00FF;
            
            sendbuffer[10]=(endposition & 0xFF00) >> 8;
            sendbuffer[11]=(endposition & 0x00FF);
            
            
            sendbuffer[0]=0xB2;
            //          usb_rawhid_send((void*)sendbuffer, 50);
            
         }break;
#pragma mark B3       
         case 0xB3: // sendTextdaten
         {
            Serial.printf("*B3 Joystick*\n");
            uint8_t i=0;
            for(i=0;i<33;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
            }
            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            
            Serial.printf("B3 abschnittnummer: %d\tbuffer25 lage: %d \t device: %d\n",abschnittnummer,lage,buffer[32]);
            //             Serial.printf("count: %d\n",buffer[22]);
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               Serial.printf("B3 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
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
               //               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
               
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
               //              usb_rawhid_send((void*)sendbuffer, 50);
               
               //               startTimer2();
               //               interrupts();
               // Serial.printf("------------------------------------->  first abschnitt end\n");
            }
            else // Abschnittnummer > 0
            {
               Serial.printf("XX\n");
               // Ablauf schon gestartert
               //          Serial.printf("  -----                  B3 Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
               //lcd.setCursor(12,0);
               //lcd.print(String(abschnittnummer));
               
            }
            
            
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //              Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //               Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            uint8_t pos=(abschnittnummer);
            
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
            //              Serial.printf("default: load  CNC-Daten. abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
            //if (abschnittnummer>8)
            
            {
               //lcd_putint1(pos);
            }
            
            // Daten laden in ringbuffer an Position pos
            //          uint8_t i=0;
            /*
             for(i=0;i<10;i++)
             {
             Serial.printf("%d\t",i);
             }
             */
            //Serial.printf("\n");
            //OSZI_A_LO();
            //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
            //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
            for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               
               CNCDaten[pos][i]=buffer[i];  
            }
            interrupts();
         }break;

#pragma mark B5               
         case 0xB5: // PCB neu
         {
            Serial.printf("B5\n");
            sendbuffer[0]=0xB6;
            usb_rawhid_send((void*)sendbuffer, 50);
            uint8_t i=0;
            for(i=0;i<33;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               CNCDaten[0][i]=buffer[i];  
            }
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
         //   Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
         //   Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               Serial.printf("B5 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
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
               {
               sendbuffer[0]=0xB6;
               }
               
            }
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               Serial.printf("------------------------ B5  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                     Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //                      Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            
            
         
         }break;
            
         case 0xCA:
         {
            Serial.printf("case CA\n");
         }break;
            
            
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
            digitalWriteFast(DC_PWM,HIGH);
            
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            digitalWriteFast(MD_EN,HIGH);
            lcd.setCursor(0,1);
            lcd.print("HALT");
            
            // lcd_gotoxy(0,1);
            // lcd_puts("HALT\0");
            Serial.printf("E0 Stop END\n");
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
            
            
            
#pragma mark F1 reset 
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
            digitalWriteFast(DC_PWM,LOW);
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            korrekturcounterx = 0;
            korrekturcountery = 0;
            
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            
            digitalWriteFast(MA_STEP,HIGH);
            digitalWriteFast(MB_STEP,HIGH);
            
            //           lcd.setCursor(0,1);
            //           lcd.print("reset\n");
            //cli();
            //usb_init();
            /*
             while (!usb_configured()) // wait  ;
             
             // Wait an extra second for the PC's operating system to load drivers
             // and do whatever it does to actually be ready for input
             _delay_ms(1000);
             */
            //sei();
            sendbuffer[0]=0xF2;
            //         usb_rawhid_send((void*)sendbuffer, 50);
            sendbuffer[0]=0x00;
            
         }break;
            
   // MARK: F0
      case 0xF0:// cncstatus fuer go_home setzen
         {
          //  lcd_cls();
          //  lcd_gotoxy(0,0);
          //  lcd_puts("HOME");
            
          //  gohome();
          //  break;
            abschnittnummer = 0; // diff 220520
            
            ladeposition=0;
            endposition=0xFFFF;
            cncstatus = 0;
            motorstatus = 0;
            ringbufferstatus=0x00;
            anschlagstatus=0;
            ringbufferstatus |= (1<<FIRSTBIT);
  //          ringbufferstatus |= (1<<STARTBIT); // diff 220520

            AbschnittCounter=0;
            //sendbuffer[8]= versionintl;
            //sendbuffer[8]= versioninth;

            sendbuffer[0]=0xF0;
            //sendbuffer[0]=0x45;
            cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
            sendbuffer[22] = cncstatus;
            
            ringbufferstatus |= (1<<LASTBIT);
            // Daten vom buffer in CNCDaten laden
            {
               uint8_t pos=0;
               pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
               //if (abschnittnummer>8)
               {
                  //lcd_putint1(pos);
               }
               uint8_t i=0;
               for(i=0;i<USB_DATENBREITE;i++)
               {
                  if (i<5)
                  {
                   //  lcd_puthex(buffer[i]);
                  }
                  CNCDaten[pos][i]=buffer[i];  
               }
               
            }
            startTimer2();
            
            // F0 melden
//            usb_rawhid_send((void*)sendbuffer, 50);
           
            sei();

         }break;
     
            
            
#pragma mark default 
         default: // CNC 
         {
            Serial.printf("\n---  default   usb_recv_counter %d\t \nringbufferstatus: %02X position(buffer17): %02X device: %d \n",usb_recv_counter,ringbufferstatus, buffer[17],device);
            // Abschnittnummer bestimmen
            
            sendbuffer[24] =  buffer[32];
            
            // Abschnittnummer bestimmen
            uint8_t indexh=buffer[18];
            uint8_t indexl=buffer[19];
            uint8_t position = buffer[17];
            Serial.printf("default position: %d\n",position);
            
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            sendbuffer[0]=0x33;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=buffer[16];            
            // Lage:
            
            uint8_t lage = buffer[25];
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            //OSZI_A_LO();
            //            Serial.printf("\n\ndefault abschnittnummer: %d  lage: %d code: %d\n",abschnittnummer,lage,code); // 50 us
            //            for(int i=0;i<33;i++)
            {
               //              Serial.printf("%d\t",buffer[i]);
            }
            //           Serial.printf("\n");
            
            Serial.printf("\n****************************************\n");
            Serial.printf("default Abschnitt lage: %d abschnittnummer: %d\n",lage,abschnittnummer);
            Serial.printf("****************************************\n");
                          
                        //              usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
                        
                        if (abschnittnummer==0)
                        {
                           //anschlagstatus &= ~(1<< END_A0); // 220518 diff
                           //  lcd_clr_line(1);
                           cli();
                           /*
                            uint8_t i=0,k=0;
                            for (k=0;k<RINGBUFFERTIEFE;k++)
                            {
                            for(i=0;i<USB_DATENBREITE;i++)
                            {
                            CNCDaten[k][i]=0;  
                            }
                            }
                            */
                           //CNCDaten = {};
                           
                           
                           ladeposition=0;
                           endposition=0xFFFF;
                           cncstatus = 0;
                           motorstatus = 0;
                           ringbufferstatus=0x00;
                           anschlagstatus=0;
                           ringbufferstatus |= (1<<FIRSTBIT);
                           AbschnittCounter=0;
                           //sendbuffer[8]= versionintl;
                           //sendbuffer[8]= versioninth;
                           sendbuffer[5]=0x00;
                           
                           //in teensy3.2: timerintervall
                           //                sendbuffer[8] = (TIMERINTERVALL & 0xFF00)>>8;
                           //                sendbuffer[9] = (TIMERINTERVALL & 0x00FF);
                           sendbuffer[0]=0xD1;
                           
                           //         usb_rawhid_send((void*)sendbuffer, 50);
                           startTimer2();
                           sei();
                           
                        }
                        else
                        {
                           
                        }
                        
                        //             if (buffer[9]& 0x02)// letzter Abschnitt
                        
                        if (buffer[17]& 0x02)// letzter Abschnitt
                        {
                           Serial.printf("default last Abschnitt: %d\n",buffer[17]);
                           ringbufferstatus |= (1<<LASTBIT);
                           if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
                           {
                              endposition=abschnittnummer; // erster ist letzter Abschnitt
                           }
                        }
                        
                        
                        
                        // Daten vom buffer in CNCDaten laden
                        {
                           uint8_t pos=(abschnittnummer);
                           pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
                           //if (abschnittnummer>8)
                           {
                              //lcd_putint1(pos);
                           }
                           uint8_t i=0;
                           for(i=0;i<USB_DATENBREITE;i++)
                           {
                              CNCDaten[pos][i]=buffer[i];  
                           }
                           
                        }
                        
                        
                        // Erster Abschnitt, naechsten Abschnitt laden
                        if ((abschnittnummer == 0)&&(endposition))
                        {
                           {
                              //lcd_putc('*');
                              // Version zurueckmelden
                              
                              
                              //versionl=VERSION & 0xFF;
                              //versionh=((VERSION >> 8) & 0xFF);
                              
                              
                              
                              
                              
                              sendbuffer[5]=abschnittnummer;
                              sendbuffer[6]=ladeposition;
                              sendbuffer[0]=0xAF;
                              usb_rawhid_send((void*)sendbuffer, 50);
                              sei();
                              //  sendbuffer[0]=0x00;
                              //  sendbuffer[5]=0x00;
                              //  sendbuffer[6]=0x00;
                              
                              
                           }  
                        }
                        
                        ringbufferstatus &= ~(1<<FIRSTBIT);
                        
                        // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
                        //if ((abschnittnummer >= 2)||(ringbufferstatus & (1<<LASTBIT)))                {
                        if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
                        {
                           {
                              ringbufferstatus &= ~(1<<LASTBIT);
                              ringbufferstatus |= (1<<STARTBIT);
                              
                           }
                        }
                        
                  //   } // default
                        
              
            
            
         }break; // default
            
            
            
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
      //noInterrupts();
      
      //     Serial.printf("\n\n                 Abschnitt 0 laden ringbufferstatus: %d\n",ringbufferstatus);
      ringbufferstatus &= ~(1<<STARTBIT);  // Startbit entfernen      
      ladeposition=0;  // laufender Zaehler fuer Ringbuffer, gefiltert mit Ringbuffertiefe
      AbschnittCounter=0;
      
      // Abschnitt 0 laden
      uint8_t l = sizeof(CNCDaten[ladeposition]);
#pragma mark default Ersten Abschnitt laden
//      Serial.printf("+++ Ersten Abschnitt laden AbschnittLaden_4M len: %d ringbufferstatus: %d\n",l,ringbufferstatus);
      uint8_t lage=AbschnittLaden_4M(CNCDaten[ladeposition]); // erster Wert im Ringbuffer
  //    Serial.printf("+++ Erster Abschnitt lage: %d\n",lage);
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
//      Serial.printf("motorstatus: %d\n",motorstatus);
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
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
   // Es hat noch Steps, CounterA ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if ((StepCounterA > 0)  && (CounterA == 0) &&(!(anschlagstatus & (1<< END_A0))))
   {
      noInterrupts();
      
      // Impuls starten
      digitalWriteFast(MA_STEP,LOW);
      CounterA = DelayA;                     // CounterA zuruecksetzen fuer neuen Impuls
       
      //     Serial.printf("Motor A StepCounterA: %d\n",StepCounterA);
      //     Serial.printf("Motor A CounterA: %d StepCounterA: %d \n",CounterA, StepCounterA);
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
            
//            motorstatus |=  (1<< COUNT_B); // Relevanz zuteilen Motor B soll noch abzaehlen
//            motorstatus &= ~(1<< COUNT_A); 
         }
         else
         {
 //           motorstatus &= ~(1<< COUNT_B); 
            sendstatus |= (1<<COUNT_B); // Motor B auch markieren
         }
 //        Serial.printf("\nMotor A StepCounterA abgelaufen abschnittnummer: %d korrekturcounterx: %d korrekturcountery: %d StepCounterB: %d\n",abschnittnummer,korrekturcounterx, korrekturcountery,StepCounterB);
         Serial.printf("\nMotor A StepCounterA abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterB: %d sendstatus: %d \n", abschnittnummer, endposition, ringbufferstatus, StepCounterB, sendstatus);
         //        Serial.printf("Rampbreite: %d rampendstep: %d",rampbreite, rampendstep);
         
         // Begin Ringbuffer-Stuff
         
         if ((abschnittnummer==endposition)) // Ablauf fertig
         {  
            Serial.printf("-----------------------> Motor A endpos \n");
            //noInterrupts();
            if (cncstatus & (1<<GO_HOME))
            {
               homestatus |= (1<<COUNT_A);
 //              motorstatus |= (1<< COUNT_C);
            }

            ringbufferstatus = 0;
            
            //           if (StepCounterB == 0) // erst beide abzaehlen
            {
               sendstatus |= (1<<COUNT_LAST);
            }
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
            motorstatus &= ~(1<< COUNT_A);
            motorstatus=0;
            
            sendbuffer[0]=0xBD;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
        //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
            sendbuffer[22] = cncstatus;
            sendbuffer[8]=ladeposition & 0x00FF;
            usb_rawhid_send((void*)sendbuffer, 50);
             
            ladeposition=0;
            interrupts();
         }
         else 
         {
            
            uint8_t aktuellelage=0;
            aktuelleladeposition=(ladeposition & 0x00FF);
            aktuelleladeposition &= 0x03;
            
            // aktuellen Abschnitt laden
            //aktuellelage = CNCDaten[aktuelleladeposition][25];
            //globalaktuelleladeposition = ladeposition;
            aktuelleladeposition = ladeposition;
            
            //              Serial.printf("\tMotor A globalaktuelleladeposition: %d\n",globalaktuelleladeposition);
            // > verschoben in 'sendstatus-Bearbeitung
            aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
            //sendstatus = 0;
            //          }
            
            if (aktuellelage==2) // war letzter Abschnitt
            {
               Serial.printf("\tMotor A last Abschnitt\n");
               //noInterrupts();
               endposition=abschnittnummer; // letzter Abschnitt
               
               
                // Neu: letzten Abschnitt melden
                sendbuffer[0]=0xD0;
                sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                sendbuffer[6]=abschnittnummer & 0x00FF;
                //sendbuffer[8]=ladeposition & 0x00FF;
                //   sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
               sendbuffer[22] = cncstatus;
                usb_rawhid_send((void*)sendbuffer, 50);
                ringbufferstatus |= (1<<ENDBIT);
                ringbufferstatus |= (1<<LASTBIT);
                interrupts();
                //endposition = 0xFFFF;
                
               
            }
            else
            {
               //                    Serial.printf("\tMotor A next abschnittnummer: %d\n",abschnittnummer);
               
               
                sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
                sendbuffer[6]=abschnittnummer & 0x00FF;
                
                sendbuffer[8]=ladeposition & 0x00FF;
                //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                sendbuffer[0]=0xA1;
               sendbuffer[22] = cncstatus;
                usb_rawhid_send((void*)sendbuffer, 50);
                //sendstatus = 0;
                
               
            }
            ladeposition++;
            AbschnittCounter++;
            //             AbschnittCounter++;
            //          Serial.printf("\tMotor A Abschnittcounter: %d\n",AbschnittCounter);
         }
      }
      
      interrupts();
   }
   else
   {
      //OSZI_A_LO();
      if (digitalReadFast(MA_STEP) == 0) //100 ns
      {
         digitalWriteFast(MA_STEP,HIGH);
         
         if (StepCounterA ==0)                     // Keine Steps mehr fuer Motor A
         {
            Serial.printf("Motor A OFF\n");// Motor A OFF
       
            digitalWriteFast(MA_EN,HIGH);
            
            if (StepCounterB ==0)
            {
               digitalWriteFast(MB_EN,HIGH);
            }
            
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
      //noInterrupts();
      
      //STEPPERPORT_1 &= ~(1<<MB_STEP);               // Impuls an Motor B LO ON
      digitalWriteFast(MB_STEP,LOW);
      CounterB= DelayB;
      if (StepCounterB > 0)
      {
         StepCounterB--;
      }
      
      if ((StepCounterB == 0)  && (motorstatus & (1<< COUNT_B))) // StepCounterB jetzt abgelaufen, Motor B ist relevant fuer Stepcount 
      {
         sendstatus |= (1<<COUNT_B);
         if (StepCounterA > 0)  // Motor A ist noch nicht fertig
         {
//            motorstatus |=  (1<< COUNT_A);
//            motorstatus &= ~(1<< COUNT_B);
         }
         else
         {
//            motorstatus &= ~(1<< COUNT_B); 
            sendstatus |= (1<<COUNT_A); // Motor A abgelaufen, auch markieren
         }
         
         // Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d",abschnittnummer);
   //      Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d endposition: %d  StepCounterA: %d\n",abschnittnummer,endposition,StepCounterA);

         Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterA: %d sendstatus: %d \n", abschnittnummer, endposition, ringbufferstatus, StepCounterA, sendstatus);
          
         //         Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterA: %d sendstatus: %d\n", abschnittnummer, endposition, ringbufferstatus,StepCounterA,sendstatus);
         
         // Begin Ringbuffer-Stuff
         
         if ((abschnittnummer==endposition) )
         {  
            if (cncstatus & (1<<GO_HOME))
            {
               homestatus |= (1<<COUNT_B);
         //      motorstatus |= (1<< COUNT_C);
            }

            //cli();
            Serial.printf("----------------------> Motor B endpos \n");
            //noInterrupts();
            ringbufferstatus = 0;
            
            //           if (StepCounterA == 0)
            {
               sendstatus |= (1<<COUNT_LAST); // beenden abzeigen
            }
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
            //     motorstatus=0;
            
            //           sendbuffer[0]=0xAD;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            sendbuffer[8]=ladeposition & 0x00FF;
            sendbuffer[22] = cncstatus;
            usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            interrupts();
         }
         else 
         { 
            uint8_t aktuellelage=0;
            aktuelleladeposition=(ladeposition & 0x00FF);
            aktuelleladeposition &= 0x03;
            
            // aktuellen Abschnitt laden
            aktuellelage = CNCDaten[aktuelleladeposition][25];
            //globalaktuelleladeposition = ladeposition;
            aktuelleladeposition = ladeposition;
            
            
            
            if (aktuellelage==2) // war letzter Abschnitt
            {
               //noInterrupts();
               endposition=abschnittnummer; // letzter Abschnitt
               Serial.printf("\tMotor B last Abschnitt\n");
               // sendstatus |= (1<<COUNT_LAST);
               
               
               // Neu: letzten Abschnitt melden
               
                sendbuffer[0]=0xD0;
                sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                sendbuffer[6]=abschnittnummer & 0x00FF;
                
                sendbuffer[8]=ladeposition & 0x00FF;
                //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
               sendbuffer[22] = cncstatus;
                usb_rawhid_send((void*)sendbuffer, 50);
                
               
            }  
            else 
            {
               
               // neuen Abschnitt abruffen
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
              // sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
               sendbuffer[22] = cncstatus;
               sendbuffer[0]=0xA1;
               usb_rawhid_send((void*)sendbuffer, 50);
               sei();
               
            }
            
         }
         
      }
      
      
      interrupts();
   }
   else// if (CounterB)
   {
      
      if (digitalReadFast(MB_STEP) == 0) //100 ns
      {
         digitalWriteFast(MB_STEP,HIGH);
         if (StepCounterB ==0)                     // Keine Steps mehr fuer Motor B
         {
            Serial.printf("Motor B OFF \n");
            //STEPPERPORT_1 |= (1<<MB_EN);               // Motor B OFF
            digitalWriteFast(MB_EN,HIGH);
            
            if (StepCounterA ==0)
            {
               digitalWriteFast(MA_EN,HIGH);
            }
         }
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
      //noInterrupts();
      // Impuls starten
      //STEPPERPORT_2 &= ~(1<<MC_STEP);   // Impuls an Motor C LO -> ON
      digitalWriteFast(MC_STEP,LOW);
      CounterC=DelayC;                     // CounterC zuruecksetzen fuer neuen Impuls
      
      
      StepCounterC--;
      
      // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
      if (StepCounterC ==0 && (motorstatus & (1<< COUNT_C)))    // Motor A ist relevant fuer Stepcount 
      {
         Serial.printf("\nMotor C StepCounterC abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d  sendstatus: \n", abschnittnummer, endposition, ringbufferstatus,  sendstatus);

         //            STEPPERPORT_2 |= (1<<MC_EN);                          // Motor C OFF
         //StepCounterD=0; 
         
         // Begin Ringbuffer-Stuff
         //if (ringbufferstatus & (1<<ENDBIT))
         
         sendstatus |= (1<<COUNT_C);
         
         if (abschnittnummer==endposition)
         {  
            //cli();
            Serial.printf("\nMotor C endpos\n");
            sendstatus |= (1<<COUNT_LAST);
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
               aktuelleladeposition=(ladeposition & 0x00FF);
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
         //Serial.printf("Motor C  StepCounterC 0");
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
               aktuelleladeposition=(ladeposition & 0x00FF);
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
      
      //Serial.printf("\n++++++++++++++++++++++++++++++\nsendstatus: %d abschnittnummer: %d endposition: %d globalaktuelleladeposition: %d: StepCounterA: %d StepCounterB: %d\n", sendstatus,abschnittnummer,endposition,globalaktuelleladeposition,StepCounterA, StepCounterB);
 //     Serial.printf("\n++++++++++++++++++++++++++++++\nsendstatus: %d abschnittnummer: %d endposition: %d aktuelleladeposition: %d: StepCounterA: %d StepCounterB: %d\n", sendstatus,abschnittnummer,endposition,aktuelleladeposition,StepCounterA, StepCounterB);
      
      //      if ((sendstatus == 3) ) 
      if ((sendstatus  <= 3) ) 
      {
         
         
               Serial.printf("\nsendstatus .task abschnittnummer: %d endposition: %d \n",abschnittnummer, endposition);
         if (abschnittnummer == endposition)
         {
                        Serial.printf("\n****************************************\n");
                        Serial.printf("\n sendstatus <=3  wert: %d  abschnittnummer = endposition\n",sendstatus);
                        Serial.printf("\n****************************************\n");
         }
         
         motorstatus=0;
         sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
         sendbuffer[6]=abschnittnummer & 0x00FF;
         
         //sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
         sendbuffer[8]=aktuelleladeposition & 0x00FF;
         //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
         
         {
            // ***************************************
            sendbuffer[0]=0xA1;
            // ***************************************
            
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
               //             Serial.printf("\tsendstatus next Abschnitt\n\n");
               //uint8_t aktuellelage = AbschnittLaden_4M(CNCDaten[(globalaktuelleladeposition & 0x03)]);
               uint8_t aktuellelage = AbschnittLaden_4M(CNCDaten[(aktuelleladeposition & 0x03)]);
               
               //           Serial.printf("\n****************************************\n");
               //           Serial.printf("Load Abschnitt %d\n",globalaktuelleladeposition);
               //           Serial.printf("****************************************\n");
               
               //            Serial.printf("\n\tsendstatus next Abschnitt geladen aktuellelage: \n",aktuellelage);
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
         
         //Serial.printf("\nsendstatus: %d abschnittnummer: %d globalaktuelleladeposition: %d\n", sendstatus,abschnittnummer,globalaktuelleladeposition);  
         //         Serial.printf("\n end <=3 sendstatus: %d abschnittnummer: %d aktuelleladeposition: %d\n", sendstatus,abschnittnummer,aktuelleladeposition);  
         sendstatus = 0;
      }
      
      
      if ((sendstatus & (1<<COUNT_LAST)) && ((StepCounterA == 0) && (StepCounterB == 0))) // 131
      {
         //        Serial.printf("\t+++   sendstatus last   +++\n");
         Serial.printf("\t COUNT LAST StepCounterA: %d StepCounterB: %d\n", StepCounterA, StepCounterB);
         if (abschnittnummer == endposition)
         {
            Serial.printf("\n**************************************** \n");
            Serial.printf(" sendstatus 131 COUNT_LAST  abschnittnummer = endposition");
            Serial.printf("\n****************************************\n");
         }
         
         //       sendbuffer[0]=0xD0;
         //      motorstatus=0;
         
         // ***************************************
         sendbuffer[0]=0xAD;
         // ***************************************
         
         sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
         sendbuffer[6]=abschnittnummer & 0x00FF;
         
         //sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
         sendbuffer[8]=aktuelleladeposition & 0x00FF;
         //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
         usb_rawhid_send((void*)sendbuffer, 50);
         sendstatus = 0;
      }
      // ladeposition++;
      AbschnittCounter++;
      sendstatus = 0;
      
   }
   
   if (sendstatus)
   {
      //Serial.printf("\nsendstatus: %d abschnittnummer: %d globalaktuelleladeposition: %d\n", sendstatus,abschnittnummer,globalaktuelleladeposition);
      //Serial.printf("\nsendstatus: %d abschnittnummer: %d aktuelleladeposition: %d\n", sendstatus,abschnittnummer,aktuelleladeposition);
      //     sendstatus = 0;
   }
   
   interrupts();
   // End Motor D
   
   
   
   
   
   
   
   /**   End CNC-routinen   ***********************/
}
