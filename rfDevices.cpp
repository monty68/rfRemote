/*
 *    File:         rfDevices.cpp
 *
 *    Description:  RF Home Automation Library
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in rfRemote.h)
 */
#include "rfRemote.h"
#include "rfDevLWRF.h"
//#include "rfDevNECO.h"

/*******************************************************************************
 * RSL366T 4x4 On/Off Remote (e.g. Maplin's)
 */
class rfdRSL366T : public rfDevice {
  public:
    rfdRSL366T()
      : rfDevice("RSL3", 24, 25, 430, 1350, 13800) {}
};

/*******************************************************************************
 * Home Easy HE200.
 */
class rfdHomeE200 : public rfDevice {
  public:
    rfdHomeE200()
      : rfDevice("HE200", 24, 25, 350, 1050, 10050) {}
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
 * Known Device Drivers
 */
rfdRSL366T    devRSL366T;
rfdHomeE200   devHomeE200;
rfdByronBell  devByronBell;

#ifdef _RFDEVLWRF_H
  rfdLightwave  devLightwave;
#endif

#ifdef _RFDEVNECO_H
  rfdNeco       devNeco;
#endif

/*******************************************************************************
 * Array of Known Device Drivers
 */
RFREMOTE_DEVICE rfRemote::rfDevices[] =
{
  { 1,   &devRSL366T   },
  { 2,   &devHomeE200  },
  { 3,   &devByronBell },

  #ifdef _RFDEVLWRF_H
  { 100, &devLightwave },
  #endif
  #ifdef _RFDEVNECO_H
  { 101, &devNeco      },
  #endif

  { 0,  NULL }            // Must be the last entry!
};
/******************************************************************************/

