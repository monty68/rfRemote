/*
 *    File:         RFremote.cpp
 *
 *    Description:  RF Home Automation Library
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in RFremote.h)
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "RFremote.h"
#include "RFdebug.h"

/*******************************************************************************
 * Timer defintion(s)
 */
 
// Teensy 2 and 3.x
#if defined(__MK20DX128__) || defined(__MK20DX256__)
#include <IntervalTimer.h>
IntervalTimer rfRemoteTimer;

// Arduino Due
#elif defined ( __SAM3X8E__)
#error "rfRemote: Please add uSecond timer code for SAM3X8E\n"
#define RFREMOTE_SAM3_TIMER

// Arduino Uno
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
//#error "rfRemote: Please add uSecond Timer1 code for Uno\n"
#define RFREMOTE_AVR_TIMER1

// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#error "rfRemote: Please test uSecond Timer3 code for Mega2560\n"
#define RFREMOTE_AVR_TIMER3
#define AVR_TIMER3_RESOLUTION 65536      // Timer3 is 16 bit

#else
#error "rfRemote: Please add uSecond timer code/macros here\n"
#endif

/*******************************************************************************
 * Initialize static class members
 */
int rfRemote::rxPin = -1;
volatile bool rfRemote::rxSignal = LOW;
volatile word rfRemote::rxPulse = 0;
volatile unsigned long rfRemote::rxStamp = 0;

int rfRemote::txPin = -1;
const char *rfRemote::ptxData = NULL;
rfDevice *rfRemote::ptxDevice;
volatile RFREMOTE_TX_STATE rfRemote::txState = RFREMOTE_TX_IDLE;

/*
 * Class construction
 */
rfRemote::rfRemote(int rxpin, int txpin)
{
  rxEnable(rxpin);
  txEnable(txpin);
  rxSignal = LOW;
  rxPulse = 0;
  rxStamp = 0;
}

/*
 * Class deconstruction
 */
rfRemote::~rfRemote()
{
  rxDisable();
  txDisable();
}

/*******************************************************************************
 * Find an RF device
 */
rfDevice *rfRemote::findDevice(byte type)
{
  int d = 0;
  while (rfDevices[d].type != 0)
  {
    if (rfDevices[d].type == type)
      return rfDevices[d].device;
    d++;
  }
  return NULL;
}

rfDevice *rfRemote::findDevice(const char *name)
{
  int d = 0;
  while (rfDevices[d].type != 0)
  {
    if (strcmp(rfDevices[d].device->name(), name) == 0)
      return rfDevices[d].device;
    d++;
  }
  return NULL; 
}

/*******************************************************************************
 * Receive a Data Packet
 */
uint32_t rfRemote::rxPacket(rfDevice *device)
{
  uint32_t d2;

  return rxPacket(device, &d2);
}

uint32_t rfRemote::rxPacket(rfDevice* device, uint32_t *d2)
{
  *d2 = device->rxData[1];
  return device->rxData[0];
}

/*******************************************************************************
 * RX Data Decoder(s)
 */
rfDevice* rfRemote::rxDecoder()
{
  cli();
  bool rf_signal = rxSignal;
  word rf_pulse = rxPulse;
  rxPulse = 0;
  sei();

  if (rf_pulse)
  {
    int d = 0;
    while (rfDevices[d].type != 0)
    {
      if (rfDevices[d].device->rxDecoder(rf_pulse, rf_signal))
        return rfDevices[d].device;
      d++;
    }
  }
  return NULL;
}

#ifndef RFREMOTE_IGNORE_DEBUGGER
rfDevice* rfRemote::rxDebugger()
{
  static rfdDEBUG devDebug;

  cli();
  bool rf_signal = rxSignal;
  word rf_pulse = rxPulse;
  rxPulse = 0;
  sei();

  if (rf_pulse)
  {
    if (devDebug.rxDebug(rf_pulse, rf_signal))
      return &devDebug;
  }
  return NULL;
}
#endif

/*
 * RX Enable
 */
bool rfRemote::rxEnable(int pin)
{
  int rxisr = digitalPinToInterrupt(pin);

  if (rxisr != NOT_AN_INTERRUPT)
  {
    rxPin = pin;
    pinMode(pin, INPUT);
    attachInterrupt(rxisr, rxISR, CHANGE);
    return true;
  }
  rxPin = -1;
  return false;
}

/*
 * RX Disable
 */
void rfRemote::rxDisable()
{
  if (rxPin != -1)
  {
    int rxisr = digitalPinToInterrupt(rxPin);
    
    if (rxisr != NOT_AN_INTERRUPT)
      detachInterrupt(rxisr);
    rxPin = -1;     
  }
}

/*
 * Receiver ISR Routine - TODO: Ignore our transmissions?!
 */
void rfRemote::rxISR()
{
  unsigned long  now = micros();

  if (rfRemote::txState != RFREMOTE_TX_IDLE)
    return;

  rxSignal = digitalRead(rxPin);
  rxPulse = (word)now - rxStamp;
  rxStamp = now;
}

/*******************************************************************************
 * Transmit a Data Packet
 */
rfDevice* rfRemote::txPacket(const char *name, uint32_t d1, uint32_t d2, bool wait)
{
  rfDevice* device = findDevice(name);

  if (device)
  {
    if (txPacket(device, d1, d2, wait))
      return device;
  }
  return NULL;
}

rfDevice* rfRemote::txPacket(byte type, uint32_t d1, uint32_t d2, bool wait)
{
  rfDevice* device = findDevice(type);

  if (device)
  {
    if (txPacket(device, d1, d2, wait))
      return device;
  }
  return NULL;
}

bool rfRemote::txPacket(rfDevice* device, uint32_t d1, uint32_t d2, bool wait)
{
  const char * pd;
  
  // Make sure we are not already transmitting!
  while (rfRemote::txState != RFREMOTE_TX_IDLE)
    ;  

  rfRemote::txState = RFREMOTE_TX_SEND;

  DEBUG("[TX] ");
  DEBUG(device->name());
  DEBUG(" ");
  DEBUG(device->bits());
  DEBUG(" ");

  if ((pd = device->txEncode(d1, d2)) != NULL)
  {
    DEBUG(pd);
    DEBUG(" - ");
    DEBUG(d1);
    DEBUG(" / ");
    DEBUGLN(d2);

    if (wait)
      return (device->txPacket(pd));
    return (txStart(device, pd)); 
  }

  DEBUGLN("");
  return false;
}

/*
 * Transmitter Enable
 */
void rfRemote::txEnable(int pin)
{
  if (rfRemote::txPin != pin)
  {
    rfRemote::txDisable();
    rfRemote::txPin = pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    rfRemote::txState = RFREMOTE_TX_IDLE;
  }
}

/*
 * Transmitter Start
 */
bool rfRemote::txStart(rfDevice* ptxdevice, const char *ptxdata)
{
  // Only start if we have a valid transmitter pin
  if (rfRemote::txPin == -1)
    return false;

  // Make sure we are not already transmitting!
  while (rfRemote::txState != RFREMOTE_TX_IDLE)
    ;
    
  rfRemote::txState = RFREMOTE_TX_SEND;
  rfRemote::ptxDevice = ptxdevice;
  rfRemote::ptxData = ptxdata;
  uint32_t usecPeriod = ptxdevice->pulseShort;

  DEBUG("[TX] ");
  DEBUG(ptxdevice->name());
  DEBUG(" ");
  DEBUG(ptxdevice->bits());
  DEBUG(" ");
  DEBUG("Pulse Size: ");
  DEBUGLN(usecPeriod);
  DEBUGLN(micros());

  digitalWrite(txPin, HIGH);

  // Start the transmitter timer
  //
  // Teensy 2 & 3.x
  #if defined(__INTERVALTIMER_H__)
    return rfRemoteTimer.begin(txISR, usecPeriod);
  #elif defined(RFREMOTE_SAM3_TIMER)
  #elif defined(RFREMOTE_AVR_TIMER1)
  #elif defined(RFREMOTE_AVR_TIMER3)
    // clear control register A
    TCCR3A = 0;
    
    // set mode as phase and frequency correct pwm, stop the timer
    TCCR3B = _BV(WGM13);
    
    // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
    long cycles = (F_CPU * uSecPeriod) / 2000000;
    unsigned char clockSelectBits;
    unsigned int pwmPeriod; 

    if(cycles < AVR_TIMER3_RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
    else if((cycles >>= 3) < AVR_TIMER3_RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
    else if((cycles >>= 3) < AVR_TIMER3_RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
    else if((cycles >>= 2) < AVR_TIMER3_RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
    else if((cycles >>= 2) < AVR_TIMER3_RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
    else        cycles = AVR_TIMER3_RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
    ICR3 = pwmPeriod = cycles;                                                                // ICR1 is TOP in p & f correct pwm mode
    
    TCCR3A &= ~_BV(COM3A1);                         // clear the bit that enables pwm on PE3
    TCCR3A &= ~_BV(COM3B1);                         // clear the bit that enables pwm on PE4
    TCCR3A &= ~_BV(COM3C1);                         // clear the bit that enables pwm on PE5
  
    TCCR3B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    TCCR3B |= clockSelectBits;                      // reset clock select register
    
    TIMSK3 = _BV(TOIE1);                            // sets the timer overflow interrupt enable bit
    sei();                                          // ensures that interrupts are globally enabled
    TCNT3 = 0;
  #endif
  return true;
}

/*
 * Transmitter Stop
 */
void rfRemote::txStop()
{
  // Only stop if we have a valid transmitter pin
  if (rfRemote::txPin == -1)
    return;

  // Shut off the transmitter
  rfRemote::txState = RFREMOTE_TX_IDLE;
  digitalWrite(rfRemote::txPin, LOW);

  // Stop the transmitter timer
  //
  // Teensy 2 & 3.x
  #if defined(__INTERVALTIMER_H__)
    rfRemoteTimer.end();
  #elif defined(RFREMOTE_SAM3_TIMER)
  #elif defined(RFREMOTE_AVR_TIMER1)
  #elif defined(RFREMOTE_AVR_TIMER3)
    TCCR3B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
    TIMSK3 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit 
    TCCR3A |= _BV(COM3A1);
    TCCR3A |= _BV(COM3B1);
    TCCR3A |= _BV(COM3C1);
  #endif 
}

/*
 * Transmitter Disable
 */
void rfRemote::txDisable()
{
  rfRemote::txState = RFREMOTE_TX_IDLE;
  
  if (rfRemote::txPin != -1)
  {
    rfRemote::txStop();
    rfRemote::txPin = -1;  
  }
}

/*
 * ISR Wrapper
 */
#if !defined(__INTERVALTIMER_H__) && !defined(RFREMOTE_SAM3_TIMER)
  #if defined(RFREMOTE_AVR_TIMER1)
    ISR(TIMER1_OVF_vect)
  #elif defined(RFREMOTE_AVR_TIMER3)
    ISR(TIMER3_OVF_vect)
  #endif
#else
  void rfRemote::txISR()
#endif
{
  rfRemote::ptxDevice->txISR();
  if (rfRemote::txState == RFREMOTE_TX_IDLE)
    rfRemote::txStop();
}
/******************************************************************************/
