//------------------------------------------------------------------------------
// Skycam - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2017-08-09
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Skycam for more information.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions


extern float limit_ax;
extern float limit_ay;
extern float limit_az;

extern float limit_bx;
extern float limit_by;
extern float limit_bz;

extern float limit_cx;
extern float limit_cy;
extern float limit_cz;

extern float limit_dx;
extern float limit_dy;
extern float limit_dz;


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
void EEPROM_writeLong(int ee, long value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
float EEPROM_readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


//------------------------------------------------------------------------------
char loadVersion() {
  return EEPROM.read(ADDR_VERSION);
}


//------------------------------------------------------------------------------
void loadConfig() {
  char versionNumber = loadVersion();
  if( versionNumber != EEPROM_VERSION ) {
    // If not the current EEPROM_VERSION or the EEPROM_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(ADDR_VERSION,EEPROM_VERSION);
  }
  
  // Retrieve stored configuration
  robot_uid=EEPROM_readLong(ADDR_UUID);
  loadDimensions();
}


//------------------------------------------------------------------------------
void saveUID() {
  Serial.println(F("Saving UID."));
  EEPROM_writeLong(ADDR_UUID,(long)robot_uid);
}


//------------------------------------------------------------------------------
void saveDimensions() {
  Serial.println(F("Saving dimensions."));
  EEPROM_writeLong(ADDR_AX,limit_ax*100);
  EEPROM_writeLong(ADDR_AY,limit_ay*100);
  EEPROM_writeLong(ADDR_AZ,limit_az*100);
  
  EEPROM_writeLong(ADDR_BX,limit_bx*100);
  EEPROM_writeLong(ADDR_BY,limit_by*100);
  EEPROM_writeLong(ADDR_BZ,limit_bz*100);
  
  EEPROM_writeLong(ADDR_CX,limit_cx*100);
  EEPROM_writeLong(ADDR_CY,limit_cy*100);
  EEPROM_writeLong(ADDR_CZ,limit_cz*100);

  EEPROM_writeLong(ADDR_DX,limit_dx*100);
  EEPROM_writeLong(ADDR_DY,limit_dy*100);
  EEPROM_writeLong(ADDR_DZ,limit_dz*100);
}


//------------------------------------------------------------------------------
void loadDimensions() {
  limit_ax = (float)EEPROM_readLong(ADDR_AX)/100.0f;
  limit_ay = (float)EEPROM_readLong(ADDR_AY)/100.0f;
  limit_az = (float)EEPROM_readLong(ADDR_AZ)/100.0f;

  limit_bx = (float)EEPROM_readLong(ADDR_BX)/100.0f;
  limit_by = (float)EEPROM_readLong(ADDR_BY)/100.0f;
  limit_bz = (float)EEPROM_readLong(ADDR_BZ)/100.0f;

  limit_cx = (float)EEPROM_readLong(ADDR_CX)/100.0f;
  limit_cy = (float)EEPROM_readLong(ADDR_CY)/100.0f;
  limit_cz = (float)EEPROM_readLong(ADDR_CZ)/100.0f;

  limit_dx = (float)EEPROM_readLong(ADDR_DX)/100.0f;
  limit_dy = (float)EEPROM_readLong(ADDR_DY)/100.0f;
  limit_dz = (float)EEPROM_readLong(ADDR_DZ)/100.0f;
}


//------------------------------------------------------------------------------
void adjustDimensions(
  float ax,float ay,float az,
  float bx,float by,float bz,
  float cx,float cy,float cz,
  float dx,float dy,float dz)
{

  char changed=0;
  float v;
  
  v = floor(ax*100)/100.0f;  if(limit_ax != ax) { limit_ax = ax; changed=1; }  
  v = floor(ay*100)/100.0f;  if(limit_ay != ay) { limit_ay = ay; changed=1; }  
  v = floor(az*100)/100.0f;  if(limit_az != az) { limit_az = az; changed=1; }
  v = floor(bx*100)/100.0f;  if(limit_bx != bx) { limit_ax = bx; changed=1; }  
  v = floor(by*100)/100.0f;  if(limit_by != by) { limit_ay = by; changed=1; }  
  v = floor(bz*100)/100.0f;  if(limit_bz != bz) { limit_az = bz; changed=1; }  
  v = floor(cx*100)/100.0f;  if(limit_cx != cx) { limit_ax = cx; changed=1; }  
  v = floor(cy*100)/100.0f;  if(limit_cy != cy) { limit_ay = cy; changed=1; }  
  v = floor(cz*100)/100.0f;  if(limit_cz != cz) { limit_az = cz; changed=1; }  
  v = floor(dx*100)/100.0f;  if(limit_dx != dx) { limit_ax = dx; changed=1; }  
  v = floor(dy*100)/100.0f;  if(limit_dy != dy) { limit_ay = dy; changed=1; }  
  v = floor(dz*100)/100.0f;  if(limit_dz != dz) { limit_az = dz; changed=1; }  

  if(changed) saveDimensions();
}


/**
 * This file is part of Skycam.
 *
 * Skycam is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Skycam is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DrawbotGUI.  If not, see <http://www.gnu.org/licenses/>.
 */
