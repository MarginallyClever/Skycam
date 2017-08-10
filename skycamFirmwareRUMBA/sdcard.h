#ifndef SDCARD_H
#define SDCARD_H
//------------------------------------------------------------------------------
// Skycam - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2017-08-09
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Skycam for more information.
//------------------------------------------------------------------------------

#include <SPI.h>
#include <SD.h>

#ifdef HAS_SD

extern File root;
extern char sd_inserted;
extern char sd_printing_now;
extern char sd_printing_paused;
extern float sd_percent_complete;
#endif


#endif // SDCARD_H
