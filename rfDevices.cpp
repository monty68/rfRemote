/*
 *    File:         rfDevices.cpp
 *
 *    Description:  RF Home Automation Library
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in rfRemote.h)
 */
#include "rfRemote.h"

/*******************************************************************************
 * RSL366T 4x4 On/Off Remote (e.g. Maplin's)
 */
class rfdRSL366T : public rfDevice {
  public:
    rfdRSL366T()
      : rfDevice("RSL3", 24, 25, 450, 1350, 13800) {}
};

/*******************************************************************************
 * Home Easy HE200.
 */
class rfdHomeE200 : public rfDevice {
  public:
    rfdHomeE200()
      : rfDevice("HE200", 24, 25, 358, 1074, 10080) {}
};
                 
/*******************************************************************************
 * Byron (http://www.chbyron.eu/) Bell Push (e.g. 98YJ)
 */
class rfdByronBell : public rfDevice {
  public:
    rfdByronBell()
      : rfDevice("BYRB", 32, 25, 500, 1580, 7090) {}
};

/*******************************************************************************
 * NECO () Universal Garage/Gate Door Control
 */
class rfdNeco : public rfDevice {
  public:
    rfdNeco()
      : rfDevice("NECO", 64, 25, 395, 770, 15900, 3900) {}

  protected:
    bool rxPulse(word rf_pulse, bool rf_signal)
    {
      if (rf_pulse >= PULSE_TYPE_MAXIMUM)
        return false;
     
      switch(rxState)
      {
        case IDLE:
          if (rf_pulse != PULSE_SYNC_L)
            break;
          rxState = SYNC;
          rxFlip = 0;
          return true;
        
        case SYNC:
          if (rxLastPulse == PULSE_SHORT_H && rf_pulse == PULSE_LATCH_L)
          {
            if (rxFlip < 10)
              return false;
            rxState = LATCH;            
          }
          else if (rxLastPulse == PULSE_SHORT_H && rf_pulse == PULSE_SHORT_L)
          {
            rxFlip++;
          }
          else if (rf_pulse != PULSE_SHORT_H)
            break;
          return true;
        
        case LATCH:
          rxState = BITS;
          /* NO break; */

        default:
          return rfDevice::rxPulse(rf_pulse, rf_signal);
      }
      
      return false;
    }
};

/*******************************************************************************
 * LightwaveRF (http://lightwaverf.com/)
 */
#define _LWRF_MSG_BUFFER 10

class rfdLightwave : public rfDevice {
  public:

    rfdLightwave()
      : rfDevice("LWRF", 64, 35, 320, 1280, 10240)
      {
        txRepeat = 10;
      }

  protected:
    static byte Nibbles[];

    bool rxPulse(word rf_pulse, bool rf_signal)
    {
      static byte rxPkt[_LWRF_MSG_BUFFER] = { 0 };
      static byte rxBidx = 0;
      static byte rxByte = 0;
    
      if (rf_pulse >= PULSE_TYPE_MAXIMUM)
        return false;
     
      switch(rxState)
      {
        case IDLE:
          if (rf_pulse != PULSE_SYNC_L)
            break;
          rxState = SYNC;
          return true;
        
        case SYNC:
          if (rf_pulse == PULSE_SHORT_L)
          {
            rxData[0] = rxData[1] = 0;
            rxBidx = 0;
            rxState = DATA;            
          }
          else if (rf_pulse != PULSE_SHORT_H)
            break;
          return true;
          
        case DATA:
          if (rf_pulse == PULSE_SHORT_L)
          {
            rxBits = 0;
          }
          else if (rf_pulse == PULSE_LONG_L)
          {
            rxBits = 1;  
          }
          else if (rf_pulse != PULSE_SHORT_H)
            break;
          rxState = BITS;
          rxByte = 0;
          return true;
          
        case BITS:
          if (rf_pulse == PULSE_SHORT_L)
          {
            rxByte = rxByte << 1 | 1;
            rxBits += 1;
          }
          else if(rf_pulse == PULSE_LONG_L)
          {
            rxByte = rxByte << 2 | 2;
            rxBits += 2;            
          }
          else if(rf_pulse != PULSE_SHORT_H)
            break;

          if (rxBits > 8)
          {
            rxPkt[rxBidx] = rxByte;
            rxBits = 0;

            if (++rxBidx >= _LWRF_MSG_BUFFER)
            {            
              // Store the transmitters ID
              int b = 4;
              
              rxData[1] = (uint32_t)findNibble(rxPkt[b]);
              while((++b) < _LWRF_MSG_BUFFER)
              {
                rxData[1] <<= 4;
                rxData[1] |= (uint32_t)findNibble(rxPkt[b]);
              };

              // Add the channel number
              rxData[0] = (uint32_t)findNibble(rxPkt[2]);
              rxData[0] <<= 8;

              // Add the command number
              rxData[0] |= (uint32_t)findNibble(rxPkt[3]);
              rxData[0] <<= 12;

              // Add the parameter value
              rxData[0] |= (uint32_t)findNibble(rxPkt[0]);
              rxData[0] <<= 4;
              rxData[0] |= (uint32_t)findNibble(rxPkt[1]);
              
              rxState = DONE;
            }
            else
              rxState = DATA;
          }
          
          return true;

        default:
          break;
      }
      
      return false;     
    }
    
    /*
     * Lookup nibble value
     */
    byte findNibble(byte b)
    {
      for(int i = 0; i < 16; i++)
      {
        if (b == Nibbles[i])
          return i;
      }
      return 0xFF;
    }

};

byte rfdLightwave::Nibbles[] =
{
  0xF6,0xEE,0xED,0xEB,0xDE,0xDD,0xDB,0xBE,
  0xBD,0xBB,0xB7,0x7E,0x7D,0x7B,0x77,0x6F
};

/*******************************************************************************
 * Known Device Drivers
 */
rfdRSL366T    devRSL366T;
rfdHomeE200   devHomeE200;
rfdByronBell  devByronBell;
rfdNeco       devNeco;
rfdLightwave  devLightwave;

/*******************************************************************************
 * Array of Known Device Drivers
 */
RFREMOTE_DEVICE rfRemote::rfDevices[] =
{
  { 1,   &devRSL366T   },
  { 2,   &devHomeE200  },
  { 3,   &devByronBell },
  { 100, &devLightwave },
//  { 101, &devNeco      },
  { 0,  NULL }            // Must be the last entry!  
};

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
    crc = _crc16_update(crc, pData[b]);
  
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
  
  txPulse = rfRemote::ptxDevice->pulseShort;
  
  if (rfRemote::txState == RFREMOTE_TX_SEND)
  {
    txCount = txBit = 0;
    txTrigger = rfRemote::ptxDevice->pulseSync;
    digitalWrite(rfRemote::txPin, LOW);
    rfRemote::txState = RFREMOTE_TX_DATA;
    DEBUG("0");
    return;
  }
  else if((txTrigger -= txPulse) > 0)
  {
    DEBUG(txSignal);
    return;
  }

  switch(rfRemote::txState)
  {
    case RFREMOTE_TX_SYNC:
      txBit = 0;
      txTrigger = rfRemote::ptxDevice->pulseSync;
      rfRemote::txState = RFREMOTE_TX_DATA;
      digitalWrite(rfRemote::txPin, LOW);
      DEBUG("0");
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
            DEBUGLN("");
            txTrigger = rfRemote::ptxDevice->pulseShort;
            rfRemote::txState = RFREMOTE_TX_SYNC;
            digitalWrite(rfRemote::txPin, HIGH);
            DEBUG("1");
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
      DEBUG(txSignal);
      
      if (!txSignal)
      {
        DEBUG("-");
        txBit++;
      }

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

