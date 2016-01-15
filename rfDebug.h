/*
 *    File:         rfDebug.h
 *
 *    Description:  RF Home Automation Library
 *
 *    Copyright:    Copyright (c) 2015,16 P.Monteith (See details in RFremote.h)
 */
#if !defined(_RFDEBUG_H) && !defined(RFREMOTE_IGNORE_DEBUGGER)
#define _RFDEBUG_H
#include "rfRemote.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Debug Receiver Driver, Used for analysing signals of new devices
 */
class rfdDEBUG : public rfDevice {
  friend class rfRemote;
  public:
    rfdDEBUG()
      : rfDevice("DEBUG", 0, 25, 0, 0, 0, 0, 0, 0, 0, 0) {}

  protected:
    bool rxDebug(word rf_pulse, bool rf_signal)
    {
      static unsigned int t1 = 0, t2 = 0, t3 = 0, t4 = 0, tx = 0;
      static unsigned int a1 = 0, a2 = 0, a3 = 0, a4 = 0, ax = 0;
      static unsigned int syn = 0, clkT = 0, clk2T = 0, sample = 0;  
      static unsigned int tmp = 0, bc = 0, sn = 0, sp = 0, lp = 0;
      static unsigned int thresh = 6000;
      static bool done = false;
      static int phase = 0;
      char s[100];

      if (clkT != 0 && phase < 100)
      {
        tmp = rf_pulse;

        if (tmp < thresh)
        {
          if (tmp < (clkT*0.5))
          {
            clkT = tmp;
          }
          else if ((tmp >= (clkT*0.5)) && (tmp <= (clkT*1.5)))
          {
            t1++;
            a1 += tmp;
            sample++;
            clkT = (a1/t1);
          }
          else if ((tmp >= (clkT*1.8)) && (tmp <= (clkT*2.7)))
          {
            t2++;
            a2 += tmp;
            sample++;
            clkT = (a2/2) / t2;
          }
          else if ((tmp >= (clkT*2.8)) && (tmp <= (clkT*3.7)))
          {
            t3++;
            a3 += tmp;
            sample++;
            clkT = (a3/3) / t3;
          }
          else if ((tmp >= (clkT*3.8)) && (tmp <= (clkT*4.3)))
          {
            t4++;
            a4 += tmp;
            sample++;
            clkT = (a4/4) / t4;
          }
          else
          {
            //clkT = -1;
            tx++;
            ax += tmp;
          }
        }
        else
        {
          done = true;
          syn += rf_pulse;
        }
      }
      else if(rf_pulse > thresh && phase < 100)
      {
        t1 = t2 = t3 = t4 = tx = 0;
        a1 = a2 = a3 = a4 = ax = 0;
        sample = syn = 0;
        clkT = rf_pulse;
        syn += rf_pulse;
      }
      
      if (done && phase < 24)
      {
        if (clkT > 200)
        {
          a2 /= ((t2 > 0) ? t2 : 1);
          a3 /= ((t3 > 0) ? t3 : 1);
          a4 /= ((t4 > 0) ? t4 : 1);
          ax /= ((tx > 0) ? tx : 1);

          if (sample != 0)
          {
            phase++;
            
            sn += (syn / 2);
            bc += (t1+t2+t3+t4+tx);      
            sp += clkT;
            
            tmp = 0;
            
            if (t2) tmp++;
            if (t3) tmp++;
            if (t4) tmp++;
            if (tx) tmp++;
            
            clk2T = ((a2 + a3 + a4 + ax) / tmp);
            
            lp += clk2T;
          }
          
          sprintf(s, "[%2.2d] S=%d(%d), SYN: %d - T1=%d (%d), T2=%d (%d), T3=%d (%d), T4=%d (%d), T?=%d (%d) {%d|%d} - %d", 
            phase, t1+t2+t3+t4+tx, sample, syn / 2, t1, clkT, t2, a2, t3, a3, t4, a4, tx, ax, clk2T, tmp, sn / phase
          );
          DEBUGLN(s);
          
          if (phase >= 24 && sample != 0)
          {
            pktBits = (bc / phase + 1) / 2;
            lp /= phase;
            sp /= phase;
            sn /= phase;
            double r1 = 1 / ((double)sp / (double)lp);
            double r2 = ceil(r1);
            double r3 = 1 / ((double)sp / (double)sn);
            double r4 = floor(r3);
            
            sprintf(s, "[*] Bits: %d - SP: %d, LP: %d (%f, 1:%.0f) - SYN: %d (%f, 1:%.0f)", 
              pktBits, sp, lp, r1, r2, sn, r3, r4);
            DEBUGLN(s);
            
            sprintf(s, "[*] A2: %d, A3: %d, A4: %d, AX: %d", a2, a3, a4, ax);
            DEBUGLN(s);

            if (pktBits == 24 || pktBits == 32)
            {
              DEBUGLN("[!] Attempting to decode packet(s), release button and now try pressing all buttons on the remote");
              tolerance = 40;
              pulseShort = sp;
              pulseLong = lp;
              pulseSync = syn / 2;
              bit0 = { PULSE_SHORT_H, PULSE_LONG_L };
              bit1 = { PULSE_LONG_H, PULSE_SHORT_L };
              phase = 80;
              return false;
            }
            else
            {
              sn = syn / 2;
              phase = 100;
              return false;
            }
          }
        }

        t1 = t2 = t3 = t4 = tx = 0;
        a1 = a2 = a3 = a4 = ax = 0;
        sample = syn = 0;
        clkT = 0;
        done = false;
      }
      else if (phase == 80)
      {
        // Should be decoding simple 24/32 bit packets now
        if (rxDecoder(rf_pulse, rf_signal))
        {
          DEBUG("[$] ");
          //DEBUG(pktBits);
          //DEBUG(" ");
          
          // Lets see what we got    
          DEBUG(dec2binWzerofill(rxData[0], pktBits > 32 ? 32 : pktBits));
          DEBUG(" - ");
          DEBUG(rxData[0]);
          
          if (pktBits > 32)
          {
            DEBUG(" : ");  
            DEBUG(dec2binWzerofill(rxData[1], pktBits - 32));
            DEBUG(" - ");
            DEBUG(rxData[0]);
          }
          
          DEBUGLN("");

          return true;
        }
      }
      else if(phase == 100)
      {
        // More complex protcol, so we just dump out the packet as best we can
        //
        if ((rf_pulse >= (sn*0.7)) && (rf_pulse <= (sn*1.3)))
        {
          DEBUGLN("");
          sprintf(s, "~");
          DEBUG(s);
          phase++;
        }
      }
      else if(phase == 101)
      {
        char p;

        if ((rf_pulse >= (sp*0.5)) && (rf_pulse <= (sp*1.5)))
        {
          p = !rf_signal ? 'S' : 's';
        }
        else if ((rf_pulse >= (lp*0.5)) && (rf_pulse <= (lp*1.5)))
        {
          p = !rf_signal ? 'L' : 'l';
        }
        else if ((rf_pulse >= (a2*0.5)) && (rf_pulse <= (a2*1.5)))
        {
          p = '2';
        }
        else if ((rf_pulse >= (a3*0.5)) && (rf_pulse <= (a3*1.5)))
        {
          p = '3';
        }
        else if ((rf_pulse >= (a4*0.5)) && (rf_pulse <= (a4*1.5)))
        {
          p = '4';
        }
        else if ((rf_pulse >= (ax*0.5)) && (rf_pulse <= (ax*1.5)))
        {
          p = !rf_signal ? 'X' : 'x';
        }
        else if ((rf_pulse >= (sn*0.5)) && (rf_pulse <= (sn*1.5)))
        {
          p = !rf_signal ? '#' : '~';
          phase = 100;
        }
        else
        {
          p = '?';
        }
         
        sprintf(s, "%c", p);
        DEBUG(s);
      }
      
      return false;
    }
};

#ifdef __cplusplus
}
#endif
#endif
/******************************************************************************/

