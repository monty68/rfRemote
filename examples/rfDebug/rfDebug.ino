#include "rfRemote.h"
#ifdef RFREMOTE_IGNORE_DEBUGGER
#error "Please undefine RFREMOTE_IGNORE_DEBUGGER to use the debugging receiver"
#endif

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

  if ((rfd = rf.rxDebugger()) != NULL)
  {
  }
}