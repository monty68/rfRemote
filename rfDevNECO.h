/*
 *    File:         rfDevNECO.h
 *
 *    Description:  RF Home Automation Library
 *                  NECO () Universal Garage/Gate Door Control
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in RFremote.h)
 */
#if !defined(_RFDEVNECO_H)
#define _RFDEVNECO_H
#include "rfRemote.h"

#ifdef __cplusplus
extern "C" {
#endif

class rfdNeco : public rfDevice
{
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

#ifdef __cplusplus
}
#endif
#endif
/******************************************************************************/