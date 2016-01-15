/*
 *    File:         RFremote.cpp
 *
 *    Description:  RF Home Automation Library
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in RFremote.h)
 */
#include <avr/interrupt.h>
#include "RFremote.h"
#include "RFdebug.h"

/*******************************************************************************
 * Timer defintion(s)
 */

// Teensy 2 and 3.x
#if defined(__MK20DX128__) || defined(__MK20DX256__)
  #include <avr/io.h>
  #include <IntervalTimer.h>
  IntervalTimer rfRemoteTimer;

// Arduino Due
#elif defined ( __SAM3X8E__)
  #include <DueTimer.h>
  #define RFREMOTE_SAM3_TIMER
  #define sei()
  #define cli()

// Arduino Uno
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #include <avr/io.h>
  #define RFREMOTE_AVR_TIMER1
  #define AVR_TIMER1_RESOLUTION 65536      // Timer1 is 16 bit
// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #include <avr/io.h>
  #define RFREMOTE_AVR_TIMER3
  #define AVR_TIMER3_RESOLUTION 65536      // Timer3 is 16 bit
#else
  #error "rfRemote: Please add uSecond timer code/macros\n"
#endif

/*******************************************************************************
 * Initialize static class members
 */
unsigned int rfRemote::rxPin = -1;
volatile bool rfRemote::rxSignal = LOW;
volatile word rfRemote::rxPulse = 0;
volatile unsigned long rfRemote::rxStamp = 0;

unsigned int rfRemote::txPin = -1;
const char *rfRemote::ptxData = NULL;
rfDevice *rfRemote::ptxDevice;
volatile RFREMOTE_TX_STATE rfRemote::txState = RFREMOTE_TX_IDLE;

/*
 * Class construction
 */
rfRemote::rfRemote(unsigned int rxpin, unsigned int txpin)
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
bool rfRemote::rxEnable(unsigned int pin)
{
  if (pin != RFREMOTE_BAD_PIN)
  {
    int rxisr = digitalPinToInterrupt(pin);

    DEBUG("RX Pin=");
    DEBUG(pin);
    DEBUG(" - ISR: ");
    DEBUGLN(rxisr);

    if (rxisr != NOT_AN_INTERRUPT)
    {
      rxPin = pin;
      pinMode(pin, INPUT);
      attachInterrupt(rxisr, rxISR, CHANGE);
      return true;
    }
  }

  rxPin = RFREMOTE_BAD_PIN;
  return false;
}

/*
 * RX Disable
 */
void rfRemote::rxDisable()
{
  if (rxPin != RFREMOTE_BAD_PIN)
  {
    int rxisr = digitalPinToInterrupt(rxPin);

    if (rxisr != NOT_AN_INTERRUPT)
      detachInterrupt(rxisr);
    rxPin = RFREMOTE_BAD_PIN;
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
void rfRemote::txEnable(unsigned int pin)
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
  if (rfRemote::txPin == RFREMOTE_BAD_PIN)
    return false;

  rfRemote::txState = RFREMOTE_TX_SEND;
  rfRemote::ptxDevice = ptxdevice;
  rfRemote::ptxData = ptxdata;
  uint32_t uSecPeriod = 10; //ptxdevice->pulseShort;

  // Start the transmitter timer
  //
  // Teensy 2 & 3.x
  #if defined(__INTERVALTIMER_H__)
    return rfRemoteTimer.begin(txISR, uSecPeriod);
  #elif defined(RFREMOTE_SAM3_TIMER)
    DueTimer::getAvailable().attachInterrupt(txISR).start(uSecPeriod);
  #elif defined(RFREMOTE_AVR_TIMER1)
    // clear control register A
    TCCR1A = 0;

    // set mode 8: phase and frequency correct pwm, stop the timer
    TCCR1B = _BV(WGM13);

    // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
    long cycles = (F_CPU * uSecPeriod) / 2000000;
    unsigned char clockSelectBits;
    unsigned int pwmPeriod;
	char oldSREG;

    if(cycles < AVR_TIMER1_RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
    else if((cycles >>= 3) < AVR_TIMER1_RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
    else if((cycles >>= 3) < AVR_TIMER1_RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
    else if((cycles >>= 2) < AVR_TIMER1_RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
    else if((cycles >>= 2) < AVR_TIMER1_RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
    else        cycles = AVR_TIMER1_RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
    oldSREG = SREG;
    cli();
    ICR1 = pwmPeriod = cycles;                                                                // ICR1 is TOP in p & f correct pwm mode
    SREG = oldSREG;

    TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    TCCR1B |= clockSelectBits;                      // reset clock select register, and starts the clock

    TIMSK1 &= ~_BV(TOIE1);
    GTCCR |= _BV(PSRSYNC);  // Reset prescaler (NB: shared with all 16 bit timers);
    sei();
    TCNT1 = 0;

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
  if (rfRemote::txPin == RFREMOTE_BAD_PIN)
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
    TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
    TIMSK1 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit
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
  
  if (rfRemote::txPin != RFREMOTE_BAD_PIN)
  {
    rfRemote::txStop();
    rfRemote::txPin = RFREMOTE_BAD_PIN;
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

/*******************************************************************************
 * Receiver Decoder
 */
bool rfDevice::rxDecoder(word rf_pulse, bool rf_signal)
{
  float tl = 1.0 - (tolerance * 0.01);
  float th = 1.0 + (tolerance * 0.01);

  if (rf_pulse >= (pulseShort * tl))
  {
    if (rf_pulse <= (pulseShort * th))
    {
      rf_pulse = PULSE_SHORT + !rf_signal;
    }
    else if (rf_pulse >= (pulseLong * tl) && rf_pulse <= (pulseLong * th))
    {
      rf_pulse = PULSE_LONG + !rf_signal;
    }
    else if (rf_pulse >= (pulseSync * tl) && rf_pulse <= (pulseSync * th))
    {
      rf_pulse = PULSE_SYNC + !rf_signal;
    }
    else if (rf_pulse >= (pulseLatch * tl) && rf_pulse <= (pulseLatch * th))
    {
      rf_pulse = PULSE_LATCH + !rf_signal;
    }

    if (rxPulse(rf_pulse, rf_signal))
    {
      bool success = false;

      rxLastPulse = rf_pulse;
      if(rxState == DONE)
      {
        rxState = IDLE;
        success = rxRepeats();
      }

      return success;
    }
  }

  rxReset();
  return false;
}

/*
 * Default Receiver Pulse Decoder
 */
bool rfDevice::rxPulse(word rf_pulse, bool rf_signal)
{
  if (rf_pulse >= PULSE_TYPE_MAXIMUM)
    return false;

  switch(rxState)
  {
    case IDLE:
      if (rf_pulse != PULSE_SYNC_L)
        break;
      rxState = pulseLatch != 0 ? LATCH : SYNC;
      return true;

    case LATCH:
      if (rf_pulse != PULSE_LATCH)
        break;
      /* NO break; */

    case SYNC:
      rxBits = 0;
      rxData[0] = rxData[1] = 0;
      rxState = BITS;
      /* NO break; */

    case BITS:
           if (rf_pulse == bit0.h)  { rxState = BIT0; }
      else if (rf_pulse == bit1.h)  { rxState = BIT1; }
      else if (rf_pulse == bitF.h)  { rxState = BITF; }
      else
      {
        break;
      }
      return true;

    case BIT0:
      if (rf_pulse != bit0.l)
        break;
      return rxBit('0');

    case BIT1:
      if (rf_pulse != bit1.l)
        break;
      return rxBit('1');

    case BITF:
      if (rf_pulse != bitF.l)
        break;
      return rxBit('F');
  }

  return false;
}

/*
 * Store a bit using Manchester encoding
 */
bool rfDevice::rxManchester(char value)
{
  // manchester code, long pulse flips the bit
  rxFlip ^= value;
  return rxBit(rxFlip);
}

/*
 * Store Received Bit
 */
bool rfDevice::rxBit(char type)
{
  // Store Bits<32
  if (rxBits < 32)
  {
    if (type == '1')
      rxData[0] += 1;

    if (type == 'F')
      rxData[1] += 1;

    if (rxBits + 1 < pktBits)
    {
      rxData[0] = rxData[0] << 1;
      rxData[1] = rxData[1] << 1;
    }
  }

  rxState = BITS;
  if (++rxBits >= pktBits)
    rxState = DONE;
  return true;
}

/*
 * Check for duplicate packets
 */
bool rfDevice::rxRepeats()
{
  unsigned char *pData = (unsigned char *)&rxData[0];
  word crc = ~0;

  // calculate the checksum over the current packet
  for (byte b = 0; b < 8; b++)
    crc = crc16(crc, pData[b]);

  // how long was it since the last decoded packet
  word now = millis() / 100;
  word since = now - rxLastPkt;

  // if different crc or too long ago, this cannot be a repeated packet
  if (crc != rxLastCRC || since > pktGaps)
  {
    //Serial.println("New Packet?");
    rxCount = 0;
  }

  // save last values and decide whether to report this as a new packet
  rxLastCRC = crc;
  rxLastPkt = now;
  return (++rxCount == rxRepeat);
}

uint16_t rfDevice::crc16(uint16_t crc, uint8_t a)
{
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
    else
        crc = (crc >> 1);
  }

  return crc;
}

/*
 * Reset Receiver
 */
void rfDevice::rxReset()
{
  rxLastPulse = PULSE_UNKNOWN;
  rxState = IDLE;
  rxFlip = rxBits = 0;
}

/*******************************************************************************
 * Encode transmission packet
 */
char rfDevice::bitBuffer[] = { 0 };

const char * rfDevice::txEncode(uint32_t d1, uint32_t d2)
{
  return dec2binWzerofill(d1, pktBits);

  unsigned long b;
  int i = 0;

  if (pktBits > 32 && bitF.l != PULSE_UNKNOWN)
  {
    // TODO!!!!
    return NULL;
  }
  else if(pktBits <= 64)
  {
    for (i = 0, b = 1; i < pktBits && i < 32; i++, b <<= 1)
      bitBuffer[i] = (d1 & b) ? '1' : '0';

    for (i = 0, b = 1; i + 32 < pktBits && i < 32; i++, b <<= 1)
      bitBuffer[i + 32] = (d2 & b) ? '1' : '0';
  }
  else
    return NULL;

  bitBuffer[pktBits] = '\0';
  return bitBuffer;
}

/*******************************************************************************
 * transmit a packet synchronously
 */
bool rfDevice::txPacket(const char *ptxData)
{
  byte txCount = 0;
  byte txBit = 0;

  if (ptxData)
  {
    rfRemote::txState = RFREMOTE_TX_SYNC;
    digitalWrite(rfRemote::txPin, HIGH);
    delayMicroseconds(pulseShort);
    digitalWrite(rfRemote::txPin, LOW);
    delayMicroseconds(pulseSync);

    while (++txCount <= txRepeat)
    {
      rfRemote::txState = RFREMOTE_TX_DATA;
      for (txBit = 0; ptxData[txBit] != '\0'; txBit++)
      {
        switch (ptxData[txBit])
        {
          case '0':
            digitalWrite(rfRemote::txPin, HIGH);
            delayMicroseconds((unsigned long)(bit0.h & ~1) == PULSE_LONG ? pulseLong : pulseShort);
            digitalWrite(rfRemote::txPin, LOW);
            delayMicroseconds((unsigned long)(bit0.l & ~1) == PULSE_LONG ? pulseLong : pulseShort);
            break;
          case '1':
            digitalWrite(rfRemote::txPin, HIGH);
            delayMicroseconds((unsigned long)(bit1.h & ~1) == PULSE_LONG ? pulseLong : pulseShort);
            digitalWrite(rfRemote::txPin, LOW);
            delayMicroseconds((unsigned long)(bit1.l & ~1) == PULSE_LONG ? pulseLong : pulseShort);
            break;
          case 'F':
            digitalWrite(rfRemote::txPin, bitF.h & 1);
            delayMicroseconds((unsigned long)(bitF.h & ~1) == PULSE_LONG ? pulseLong : pulseShort);
            digitalWrite(rfRemote::txPin, bitF.l & 1);
            delayMicroseconds((unsigned long)(bitF.l & ~1) == PULSE_LONG ? pulseLong : pulseShort);
            break;
        }
      }

      rfRemote::txState = RFREMOTE_TX_SYNC;
      digitalWrite(rfRemote::txPin, HIGH);
      delayMicroseconds(pulseShort);
      digitalWrite(rfRemote::txPin, LOW);
      delayMicroseconds(pulseSync);
    }

    rfRemote::txState = RFREMOTE_TX_IDLE;
    return true;
  }

  return false;
}

/*******************************************************************************
 * Transmitter ISR
 */
void rfDevice::txISR()
{
  static unsigned long txStamp = 0;
  static signed short txTrigger;
  static byte txBit, txCount = 0;
  word txPulse;

  bool txSignal = digitalRead(rfRemote::txPin);
  unsigned long  now = micros();

  txPulse = (word)now - txStamp;
  txStamp = now;

  if (rfRemote::txState == RFREMOTE_TX_SEND)
  {
    txCount = txBit = 0;
    txTrigger = rfRemote::ptxDevice->pulseSync;
    digitalWrite(rfRemote::txPin, LOW);
    rfRemote::txState = RFREMOTE_TX_DATA;
    return;
  }
  else if((txTrigger -= txPulse) > 0)
    return;

  switch(rfRemote::txState)
  {
    case RFREMOTE_TX_SYNC:
      txBit = 0;
      txTrigger = rfRemote::ptxDevice->pulseSync;
      rfRemote::txState = RFREMOTE_TX_DATA;
      digitalWrite(rfRemote::txPin, LOW);

      if (++txCount == rfRemote::ptxDevice->txRepeat)
        rfRemote::txState = RFREMOTE_TX_TAIL;
      return;

    case RFREMOTE_TX_DATA:
      switch(rfRemote::ptxData[txBit])
      {
        case '0': txPulse = !txSignal ? rfRemote::ptxDevice->bit0.h : rfRemote::ptxDevice->bit0.l; break;
        case '1': txPulse = !txSignal ? rfRemote::ptxDevice->bit1.h : rfRemote::ptxDevice->bit1.l; break;
        case 'F': txPulse = !txSignal ? rfRemote::ptxDevice->bitF.h : rfRemote::ptxDevice->bitF.l; break;
        case '\0':
          if (txBit == rfRemote::ptxDevice->pktBits)
          {
            txTrigger = rfRemote::ptxDevice->pulseShort;
            rfRemote::txState = RFREMOTE_TX_SYNC;
            digitalWrite(rfRemote::txPin, HIGH);
            return;
          }
          /* NO break; */

        default:
          rfRemote::txState = RFREMOTE_TX_IDLE;
          return;
      }

      if (txPulse == PULSE_SHORT_L || txPulse == PULSE_SHORT_H)
        txTrigger = rfRemote::ptxDevice->pulseShort;
      if (txPulse == PULSE_LONG_L || txPulse == PULSE_LONG_H)
        txTrigger = rfRemote::ptxDevice->pulseLong;

      txSignal = txPulse & 1;

      digitalWrite(rfRemote::txPin, txSignal);

      if (!txSignal)
        txBit++;

      return;

    default:
      break;
  }

  rfRemote::txState = RFREMOTE_TX_IDLE;
}

/*******************************************************************************
 * Turns a decimal value to its binary representation
 */
char* rfDevice::dec2binWzerofill(uint32_t Dec, unsigned int bitLength)
{
  return dec2binWcharfill(Dec, bitLength, '0');
}

char* rfDevice::dec2binWcharfill(uint32_t Dec, unsigned int bitLength, char fill)
{
  unsigned int i=0;

  while (Dec > 0)
  {
    bitBuffer[32+i++] = ((Dec & 1) > 0) ? '1' : fill;
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j < bitLength; j++)
  {
    if (j >= bitLength - i)
    {
      bitBuffer[j] = bitBuffer[ 31 + i - (j - (bitLength - i)) ];
    }
    else
    {
      bitBuffer[j] = fill;
    }
  }

  bitBuffer[bitLength] = '\0';

  return bitBuffer;
}
/******************************************************************************/
