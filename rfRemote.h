/*
 *    File:         rfRemote.h
 *
 *    Description:  RF Home Automation Library
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith, based on the RCSwitch library
 *                  Copyright 2011 Suat �zg�r, the LightwaveRF library, Copyright 2012/13
 *                  Lawrie Griffiths and the library by Bob Tidey and library by <jc@wippler.nl>.
 */
#ifndef _RFREMOTE_H
#define _RFREMOTE_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
  #include "Energia.h"
#else
  #include "WProgram.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * General equates, defintions and contstants
 */
#define RFREMOTE_BAD_PIN    ((unsigned int)-1)

// Uncomment the below to exclude the debugger receiver code
//#define RFREMOTE_IGNORE_DEBUGGER 1

/*
 * Debug Macro's
 */
#define DEBUGLN(A)          Serial.println(A)
#define DEBUG(A)            Serial.print(A)

/*
 * Transmitter State Machine
 */
typedef enum rfremote_tx_state_t
{
  RFREMOTE_TX_IDLE = 0,
  RFREMOTE_TX_SEND,
  RFREMOTE_TX_SYNC,
  RFREMOTE_TX_LATCH,
  RFREMOTE_TX_DATA,
  RFREMOTE_TX_TAIL, 
}RFREMOTE_TX_STATE;

/******************************************************************************/

/*
 * RF OOK (On-Off Keying) Device Driver Abstract Base Class
 */
class rfDevice
{
  friend class rfRemote;
  public:
    enum {
      PULSE_UNKNOWN = 0,
      PULSE_SHORT = 2,
      PULSE_SHORT_L = 2,
      PULSE_SHORT_H,
      PULSE_LONG = 4,
      PULSE_LONG_L = 4,
      PULSE_LONG_H,
      PULSE_SYNC = 6,
      PULSE_SYNC_L = 6,
      PULSE_SYNC_H,
      PULSE_LATCH = 8,
      PULSE_LATCH_L = 8,
      PULSE_LATCH_H,
      PULSE_TYPE_MAXIMUM
    };
    
    enum { IDLE, SYNC, LATCH, DATA, BITS, BIT0, BIT1, BITF, DONE };
    typedef struct _pulsetimes_t { byte  h, l; }PULSE, *PPULSE;

    rfDevice(const char *name, byte bits, short tol, short shp, short lnp
      , short snp, short ltp = 0
      , byte b0h = PULSE_SHORT_H, byte b0l = PULSE_LONG_L
      , byte b1h = PULSE_LONG_H, byte b1l = PULSE_SHORT_L
      , byte bfh = 0, byte bfl = 0)
      : devName(name), pktBits(bits), pktGaps(5), rxRepeat(2), txRepeat(8)
      , tolerance(tol), pulseShort(shp), pulseLong(lnp), pulseSync(snp)
      , pulseLatch(ltp)
      {
        bit0.h = b0h;
        bit0.l = b0l;
        bit1.h = b1h;
        bit1.l = b1l;
        bitF.h = bfh;
        bitF.l = bfl;
        rxLastCRC = rxLastPkt = 0;
        rxCount = 0;
        rxReset();
      }
    ~rfDevice() {}

    const char *name()  { return devName; }
    byte bits() { return pktBits; }
    
    virtual void txISR(); // ??????????????????????

    /*    
     * Utility functions
     */
    static char* dec2binWzerofill(uint32_t dec, unsigned int length);
    static char* dec2binWcharfill(uint32_t dec, unsigned int length, char fill);

  private:
    rfDevice();
    static char bitBuffer[64 + 1];
    static uint16_t crc16(uint16_t crc, uint8_t a);

  protected:
    const char *devName;      // Device Name
    byte pktBits;             // Number of bits in a packet
    byte pktGaps;             // Minimum gap between packets
    byte rxRepeat;            // Minimum number of repeat receives before success
    byte txRepeat;            // Number of duplicate packets to transmit
    short tolerance;          // Tolerance % on received pulse durations
    short pulseShort;         // Length of standard short pulse
    short pulseLong;          // Length of standard long pulse   
    short pulseSync;          // Length of the sync pulse
    short pulseLatch;         // Length of any latch pulse
    PULSE bit0;               // '0' Bit timings
    PULSE bit1;               // '1' Bit timings
    PULSE bitF;               // Floating Bit timings   
    byte rxState;             // Receiver state
    byte rxCount;             // Count of duplicate packets
    byte rxBits;              // Bit(s) received
    byte rxFlip;              // Manchester bit state   
    word rxLastPulse;         // Last successful pulse
    word rxLastCRC;           // Last packet received CRC
    word rxLastPkt;           // Last packet received time
    uint32_t rxData[2];       // Received data

    bool rxDecoder(word rf_pulse, bool rf_signal);
    bool rxManchester(char value);
    bool rxBit(char type);
    bool rxRepeats();
    void rxReset();

    virtual bool rxPulse(word rf_pulse, bool rf_signal);
    virtual const char * txEncode(uint32_t d1, uint32_t d2);
    virtual bool txPacket(const char *ptxData);
};

/*
 * RF Device Driver Information Structure
 */
typedef struct rfremote_device_t
{
    char      type;
    rfDevice* device;
}RFREMOTE_DEVICE, *PRFREMOTE_DEVICE;

#include "rfDebug.h"

/*
 * RF Remote main class
 */
class rfRemote
{
  friend class rfDevice;
  private:
    rfRemote() {};
  public:
    rfRemote(unsigned int rxpin, unsigned int txpin);
    ~rfRemote();
    static rfDevice *ptxDevice;

#ifndef RFREMOTE_IGNORE_DEBUGGER
    static rfDevice* rxDebugger();
#endif
    static bool rxEnable(unsigned int pin);
    static void rxDisable();
    rfDevice* rxDecoder();
    uint32_t rxPacket(rfDevice *device);
    uint32_t rxPacket(rfDevice *device, uint32_t *d2);

    static void txEnable(unsigned int pin);
    static void txDisable();
    rfDevice * txPacket(const char *, uint32_t d1, uint32_t d2 = 0, bool wait = true);
    rfDevice * txPacket(byte type, uint32_t d1, uint32_t d2 = 0, bool wait = true);
    bool txPacket(rfDevice* device, uint32_t d1, uint32_t d2 = 0, bool wait = true);
    static rfDevice * findDevice(byte);
    static rfDevice * findDevice(const char *);

    static void txStop();

    static unsigned int txPin;
    static const char *ptxData;
    static volatile RFREMOTE_TX_STATE txState;

  protected:
    static void rxISR();
    static unsigned int rxPin;
    static volatile bool rxSignal;
    static volatile word rxPulse;
    static volatile unsigned long rxStamp;

    static bool txStart(rfDevice* ptxdevice, const char *ptxdata);
    static void txISR();

    /*
     * Device Drivers
     */
    static RFREMOTE_DEVICE rfDevices[];
};

#ifdef __cplusplus
}
#endif
#endif
/******************************************************************************/
