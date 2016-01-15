#include "rfRemote.h"

// Instantiate the RF library
// RF Receiver must be on Pin #2
//
rfRemote rf(2, -1);

void setup()
{
  Serial.begin(115200);
  delay(1500);
  DEBUGLN("[.] Ready");
  Serial.flush();
}

void loop()
{
  rfDevice *rfd;

  if ((rfd = rf.rxDecoder()) != NULL)
  {
    uint32_t rfCode, rfCode2;
    byte bits;

    // Report device type which received a packet
    DEBUG("[");
    DEBUG(rfd->name());
    DEBUG("] ");

    bits = rfd->bits();
    DEBUG(bits);
    DEBUG(" ");

    // Lets see what we got
    rfCode = rf.rxPacket(rfd, &rfCode2);

    DEBUG(rfd->dec2binWzerofill(rfCode, bits > 32 ? 32 : bits));
    DEBUG(" - ");
    DEBUG(rfCode);

    if (bits > 32)
    {
      DEBUG(" : ");
      DEBUG(rfd->dec2binWzerofill(rfCode2, bits - 32));
      DEBUG(" - ");
      DEBUG(rfCode2);
    }

    DEBUGLN("");
  }
}
