/*
 *    File:         rfDevLWRF.h
 *
 *    Description:  RF Home Automation Library
 *                  LightwaveRF (http://lightwaverf.com/)
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in RFremote.h)
 */
#if !defined(_RFDEVLWRF_H)
#define _RFDEVLWRF_H
#include "rfRemote.h"

#ifdef __cplusplus
extern "C" {
#endif

#define _LWRF_MSG_BUFFER 10

class rfdLightwave : public rfDevice
{
  public:
    rfdLightwave()
      : rfDevice("LWRF", 64, 35, 320, 1280, 10240)
      {
        txRepeat = 10;
      }

  protected:
    bool rxPulse(word rf_pulse, bool rf_signal)
    {
      static byte rxPkt[_LWRF_MSG_BUFFER] = { 0 };
      static byte rxBidx = 0;
      static byte rxByte = 0;

      if (rf_pulse >= PULSE_TYPE_MAXIMUM)
        return false;
      rf_signal = !rf_signal;

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
    byte findNibble(byte b, bool value = false)
    {
      static byte Nibbles[] =
      {
        0xF6,0xEE,0xED,0xEB,0xDE,0xDD,0xDB,0xBE,
        0xBD,0xBB,0xB7,0x7E,0x7D,0x7B,0x77,0x6F
      };

      if (value /*|| b < 16*/)
        return Nibbles[b];

      for(int i = 0; i < 16; i++)
      {
        if (b == Nibbles[i])
          return i;
      }
      return 0xFF;
    }
};

#ifdef __cplusplus
}
#endif
#endif
/******************************************************************************/