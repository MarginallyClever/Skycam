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

#include <SPI.h>  // pkm fix for Arduino 1.5

#include "Vector3.h"
#include "sdcard.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// robot UID
int robot_uid=0;

// position of each motor
float limit_ax = 0;
float limit_ay = 0;
float limit_az = 0;

float limit_bx = 0;
float limit_by = 0;
float limit_bz = 0;

float limit_cx = 0;
float limit_cy = 0;
float limit_cz = 0;

float limit_dx = 0;
float limit_dy = 0;
float limit_dz = 0;

float homeX=0;
float homeY=0;
float homeZ=0;

// plotter position.
float posx, posy, posz;  // pen state
float feed_rate=DEFAULT_FEEDRATE;
float acceleration=DEFAULT_ACCELERATION;
float step_delay;

char absolute_mode=1;  // absolute or incremental programming mode?

// Serial comm reception
char serialBuffer[MAX_BUF+1];  // Serial buffer
int sofar;                     // Serial buffer progress
static long last_cmd_time;     // prevent timeouts


Vector3 tool_offset[NUM_TOOLS];
int current_tool=0;


long line_number=0;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


//------------------------------------------------------------------------------
char readSwitches() {
#ifdef USE_LIMIT_SWITCH
  // get the current switch state
  return ( (digitalRead(LIMIT_SWITCH_PIN_LEFT)==LOW) | (digitalRead(LIMIT_SWITCH_PIN_RIGHT)==LOW) );
#else
  return 0;
#endif  // USE_LIMIT_SWITCH
}


//------------------------------------------------------------------------------
// feed rate is given in units/min and converted to cm/s
void setFeedRate(float v1) {
  if( feed_rate != v1 ) {
    feed_rate = v1;
    if(feed_rate > MAX_FEEDRATE) feed_rate = MAX_FEEDRATE;
    if(feed_rate < MIN_FEEDRATE) feed_rate = MIN_FEEDRATE;
#ifdef VERBOSE
    Serial.print(F("F="));
    Serial.println(feed_rate);
#endif
  }
}

void findStepDelay() {
  step_delay = 1000000.0f/DEFAULT_FEEDRATE;
}


/** 
 * delay in microseconds 
 */
void pause(long us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}


/**
 * print the current feed rate
 */
void printFeedRate() {
  Serial.print(F("F"));
  Serial.print(feed_rate);
  Serial.print(F("steps/s"));
}


/**
 * Inverse Kinematics turns cartesian coordinates into step count for each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param z cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float x, float y, float z, long *motorStepArray) {
  float dx,dy,dz;

  dx = x - limit_ax;  dy = y - limit_ay;  dz = z - limit_az;  motorStepArray[0] = lround( sqrt(dx*dx+dy*dy+dz*dz) / THREAD_PER_STEP );
  dx = x - limit_bx;  dy = y - limit_by;  dz = z - limit_bz;  motorStepArray[1] = lround( sqrt(dx*dx+dy*dy+dz*dz) / THREAD_PER_STEP );
  dx = x - limit_cx;  dy = y - limit_cy;  dz = z - limit_cz;  motorStepArray[2] = lround( sqrt(dx*dx+dy*dy+dz*dz) / THREAD_PER_STEP );
  dx = x - limit_dx;  dy = y - limit_dy;  dz = z - limit_dz;  motorStepArray[3] = lround( sqrt(dx*dx+dy*dy+dz*dz) / THREAD_PER_STEP );
}


/** 
 * Forward Kinematics - turns belt lengths into cartesian coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param x the resulting cartesian coordinate
 * @param y the resulting cartesian coordinate
 * @param z the resulting cartesian coordinate
 */
void FK(long *motorStepArray,float &x,float &y,float &z) {
  float a = (float)motorStepArray[0] * THREAD_PER_STEP;
  float b = (float)motorStepArray[1] * THREAD_PER_STEP;
  float c = (float)motorStepArray[2] * THREAD_PER_STEP;
  float d = (float)motorStepArray[3] * THREAD_PER_STEP;
}

/**
 * M101 Qq Xx Yy Zz
 * adjust position of one motor
 */
void M101() {
  // motor number 1=a,2=b,3=c,4=d
  int q = parseNumber('Q',-1);
  float x,y,z;
  switch(q) {
    case 0:  x = parseNumber('X',limit_ax);  y = parseNumber('Y',limit_ay);  z = parseNumber('Z',limit_az);  break;
    case 1:  x = parseNumber('X',limit_bx);  y = parseNumber('Y',limit_by);  z = parseNumber('Z',limit_bz);  break;
    case 2:  x = parseNumber('X',limit_cx);  y = parseNumber('Y',limit_cy);  z = parseNumber('Z',limit_cz);  break;
    case 3:  x = parseNumber('X',limit_dx);  y = parseNumber('Y',limit_dy);  z = parseNumber('Z',limit_dz);  break;
    default:
      Serial.println(F("Invalid motor number. 1=a,2=b,3=c,4=d\n"));  
      return;
  }
  
  M102();
}

// M102 report motor positions
void M102() {
  Serial.print(F("M101 Q0 X"));  Serial.print(limit_ax);  Serial.print(F(" Y");  Serial.print(limit_ay);  Serial.print(F(" Z");  Serial.print(limit_az);
  Serial.print(F("M101 Q1 X"));  Serial.print(limit_bx);  Serial.print(F(" Y");  Serial.print(limit_by);  Serial.print(F(" Z");  Serial.print(limit_bz);
  Serial.print(F("M101 Q2 X"));  Serial.print(limit_cx);  Serial.print(F(" Y");  Serial.print(limit_cy);  Serial.print(F(" Z");  Serial.print(limit_cz);
  Serial.print(F("M101 Q3 X"));  Serial.print(limit_dx);  Serial.print(F(" Y");  Serial.print(limit_dy);  Serial.print(F(" Z");  Serial.print(limit_dz);
}


// M103 save motor positions
void M103() {
  saveDimensions();
  Serial.println(F("Motor positions saved.");
}


/**
 * Test that IK(FK(A))=A
 */
void testKinematics() {/*
  long A[NUM_AXIES],i,j;
  float C,D,x=0,y=0;

  for(i=0;i<3000;++i) {
    x = random(limit_xmax,limit_xmax)*0.1;
    y = random(limit_ymin,limit_ymax)*0.1;

    IK(x,y,A);
    FK(A,C,D);
    Serial.print(F("\tx="));  Serial.print(x);
    Serial.print(F("\ty="));  Serial.print(y);
    for(int j=0;j<NUM_AXIES;++j) {
      Serial.print('\t');
      Serial.print(AxisLetters[j]);
      Serial.print(A[j]);
    }
    Serial.print(F("\tx'="));  Serial.print(C);
    Serial.print(F("\ty'="));  Serial.print(D);
    Serial.print(F("\tdx="));  Serial.print(C-x);
    Serial.print(F("\tdy="));  Serial.println(D-y);
  }*/
}

/**
 * Translate the XYZ through the IK to get the number of motor steps and move the motors.
 * @input x destination x value
 * @input y destination y value
 * @input z destination z value
 * @input new_feed_rate speed to travel along arc
 */
void polargraph_line(float x,float y,float z,float new_feed_rate) {
  long steps[NUM_AXIES];
  IK(x,y,z,steps);
  posx=x;
  posy=y;
  posz=z;
  feed_rate = new_feed_rate;
  motor_line(steps,new_feed_rate);
}


/**
 * Move the pen holder in a straight line using bresenham's algorithm
 * @input x destination x value
 * @input y destination y value
 * @input z destination z value
 * @input new_feed_rate speed to travel along arc
 */
void line_safe(float x,float y,float z,float new_feed_rate) {
  x-=tool_offset[current_tool].x;
  y-=tool_offset[current_tool].y;
  z-=tool_offset[current_tool].z;

  // split up long lines to make them straighter?
  Vector3 destination(x,y,z);
  Vector3 startPoint(posx,posy,posz);
  Vector3 dp = destination - startPoint;
  Vector3 temp;

  float len=dp.Length();
  int pieces = ceil(dp.Length() * (float)SEGMENT_PER_CM_LINE );

  float a;
  long j;

  // draw everything up to (but not including) the destination.
  for(j=1;j<pieces;++j) {
    a=(float)j/(float)pieces;
    temp = dp * a + startPoint;
    polargraph_line(temp.x,temp.y,temp.z,new_feed_rate);
  }
  // guarantee we stop exactly at the destination (no rounding errors).
  polargraph_line(x,y,z,new_feed_rate);
}


/**
 * This method assumes the limits have already been checked.
 * This method assumes the start and end radius match.
 * This method assumes arcs are not >180 degrees (PI radians)
 * @input cx center of circle x value
 * @input cy center of circle y value
 * @input x destination x value
 * @input y destination y value
 * @input z destination z value
 * @input dir - ARC_CW or ARC_CCW to control direction of arc
 * @input new_feed_rate speed to travel along arc
 */
void arc(float cx,float cy,float x,float y,float z,char clockwise,float new_feed_rate) {
  // get radius
  float dx = posx - cx;
  float dy = posy - cy;
  float sr=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float sa=atan3(dy,dx);
  float ea=atan3(y-cy,x-cx);
  float er=sqrt(dx*dx+dy*dy);
  
  float da=ea-sa;
  if(clockwise!=0 && da<0) ea+=2*PI;
  else if(clockwise==0 && da>0) sa+=2*PI;
  da=ea-sa;
  float dr = er-sr;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len1 = abs(da) * sr;
  float len = sqrt( len1 * len1 + dr * dr );

  int i, segments = ceil( len * SEGMENT_PER_CM_ARC );

  float nx, ny, nz, angle3, scale;
  float a,r;
  for(i=0;i<=segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);

    a = ( da * scale ) + sa;
    r = ( dr * scale ) + sr;
    
    nx = cx + cos(a) * r;
    ny = cy + sin(a) * r;
    nz = ( z - posz ) * scale + posz;
    // send it to the planner
    line_safe(nx,ny,nz,new_feed_rate);
  }
}


/**
 * Instantly move the virtual plotter position.  Does not check if the move is valid.
 */
void teleport(float x,float y,float z) {
  wait_for_empty_segment_buffer();
  
  posx=x;
  posy=y;
  posz=z;

  // @TODO: posz?
  long steps[NUM_AXIES];
  IK(posx,posy,posz,steps);
  motor_set_step_count(steps);
}


/**
 * Print a helpful message to serial.  The first line must never be changed to play nice with the JAVA software.
 */
void help() {
  Serial.print(F("\n\nHELLO WORLD! I AM SKYCAM #"));
  Serial.println(robot_uid);
  sayVersionNumber();
  Serial.println(F("== http://www.makelangelo.com/ =="));
  Serial.println(F("M100 - display this message"));
  Serial.println(F("As well as the following G-codes (http://en.wikipedia.org/wiki/G-code):"));
  Serial.println(F("G00,G01,G02,G03,G04,G28,G90,G91,G92,M18,M101,M102,M103,M114"));
}


void sayVersionNumber() {
  char versionNumber = loadVersion();
  
  Serial.print(F("Firmware v"));
  Serial.println(versionNumber,DEC);
}


/**
 * Print the X,Y,Z, feedrate, and acceleration to serial.
 * Equivalent to gcode M114
 */
void where() {
  wait_for_empty_segment_buffer();

  Serial.print(F("X"   ));  Serial.print(posx);
  Serial.print(F(" Y"  ));  Serial.print(posy);
  Serial.print(F(" Z"  ));  Serial.print(posz);
  Serial.print(' '      );  printFeedRate();
  Serial.print(F(" A"  ));  Serial.println(acceleration);
  Serial.print(F(" HX="));  Serial.print(homeX);
  Serial.print(F(" HY="));  Serial.print(homeY);
  Serial.print(F(" HY="));  Serial.println(homeZ);
  
}


/**
 * Set the relative tool offset
 * @input axis the active tool id
 * @input x the x offset
 * @input y the y offset
 * @input z the z offset
 */
void set_tool_offset(int axis,float x,float y,float z) {
  tool_offset[axis].x=x;
  tool_offset[axis].y=y;
  tool_offset[axis].z=z;
}


/**
 * @return the position + active tool offset
 */
Vector3 get_end_plus_offset() {
  return Vector3(tool_offset[current_tool].x + posx,
                 tool_offset[current_tool].y + posy,
                 tool_offset[current_tool].z + posz);
}


/**
 * Change the currently active tool
 */
void tool_change(int tool_id) {
  if(tool_id < 0) tool_id=0;
  if(tool_id >= NUM_TOOLS) tool_id=NUM_TOOLS-1;
  current_tool=tool_id;
#ifdef HAS_SD
  if(sd_printing_now) {
    sd_printing_paused=true;
  }
#endif
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=serialBuffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)serialBuffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * process commands in the serial receive buffer
 */
void processCommand() {
  // blank lines
  if(serialBuffer[0]==';') return;

  long cmd;

  // is there a line number?
  cmd=parseNumber('N',-1);
  if(cmd!=-1 && serialBuffer[0]=='N') {  // line number must appear first on the line
    if( cmd != line_number ) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(line_number);
      return;
    }

    // is there a checksum?
    if(strchr(serialBuffer,'*')!=0) {
      // yes.  is it valid?
      char checksum=0;
      int c=0;
      while(serialBuffer[c]!='*' && c<MAX_BUF) checksum ^= serialBuffer[c++];
      c++; // skip *
      int against = strtod(serialBuffer+c,NULL);
      if( checksum != against ) {
        Serial.print(F("BADCHECKSUM "));
        Serial.println(line_number);
        return;
      }
    } else {
      Serial.print(F("NOCHECKSUM "));
      Serial.println(line_number);
      return;
    }

    line_number++;
  }

  if(!strncmp(serialBuffer,"UID",3)) {
    robot_uid=atoi(strchr(serialBuffer,' ')+1);
    saveUID();
  }


  cmd=parseNumber('M',-1);
  switch(cmd) {
  case 6:  tool_change(parseNumber('T',current_tool));  break;
  case 17:  motor_engage();  break;
  case 18:  motor_disengage();  break;
  case 100:  help();  break;
  case 101:  M101();  break;  // adjust one motor position per call
  case 102:  M102();  break;  // report all motor positions
  case 103:  M103();  break;  // save all motor positions
  case 110:  line_number = parseNumber('N',line_number);  break;
  case 114:  where();  break;
  default:  break;
  }

  cmd=parseNumber('G',-1);
  switch(cmd) {
  case 0:
  case 1: {  // line
      Vector3 offset=get_end_plus_offset();
      acceleration = min(max(parseNumber('A',acceleration),1),2000);
      line_safe( parseNumber('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
                 parseNumber('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
                 parseNumber('Z',(absolute_mode?offset.z:0)   )     + (absolute_mode?0:offset.z),
                 parseNumber('F',feed_rate) );
      break;
    }
  case 2:
  case 3: {  // arc
      Vector3 offset=get_end_plus_offset();
      acceleration = min(max(parseNumber('A',acceleration),1),2000);
      setFeedRate(parseNumber('F',feed_rate));
      arc(parseNumber('I',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
          parseNumber('J',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
          parseNumber('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
          parseNumber('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
          parseNumber('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z),
          (cmd==2) ? 1 : 0,
          parseNumber('F',feed_rate) );
      break;
    }
  case 4:  {  // dwell
      wait_for_empty_segment_buffer();
      float delayTime = parseNumber('S',0) + parseNumber('P',0)*1000.0f;
      pause(delayTime);
      break;
    }
  case 28:  findHome();  break;
  case 29:  calibrateBelts();  break;
  case 54:
  case 55:
  case 56:
  case 57:
  case 58:
  case 59: {  // 54-59 tool offsets
    int tool_id=cmd-54;
    set_tool_offset(tool_id,parseNumber('X',tool_offset[tool_id].x),
                            parseNumber('Y',tool_offset[tool_id].y),
                            parseNumber('Z',tool_offset[tool_id].z));
    break;
    }
  case 90:  absolute_mode=1;  break;  // absolute mode
  case 91:  absolute_mode=0;  break;  // relative mode
  case 92: {  // set position (teleport)
      Vector3 offset = get_end_plus_offset();
      teleport( parseNumber('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
                 parseNumber('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y)
               //parseNumber('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z)
                 );
      break;
    }
  default:  break;
  }

  cmd=parseNumber('D',-1);
  switch(cmd) {
  case 0: {  // jog one motor
    jogMotorTest();
    break;
  case 4:  SD_StartPrintingFile(strchr(serialBuffer,' ')+1);  break;  // read file
  case 5:
    sayVersionNumber();
  case 6:  // set home
    setHome(parseNumber('X',(absolute_mode?homeX:0)*10)*0.1 + (absolute_mode?0:homeX),
            parseNumber('Y',(absolute_mode?homeY:0)*10)*0.1 + (absolute_mode?0:homeY),
            parseNumber('Z',(absolute_mode?homeZ:0)*10)*0.1 + (absolute_mode?0:homeZ));
  case 10:  // get hardware version
    Serial.print(F("D10 V"));
    Serial.println(SKYCAM_HARDWARE_VERSION);
    break;
  case 11:
    // if you accidentally upload m3 firmware to an m5 then upload it ONCE with this line uncommented.
    adjustDimensions( 50, 50,50,
                     -50, 50,50,
                     -50,-50,50,
                      50,-50,50);
    break;
  default:  break;
  }
}


void jogMotorTest() {
  int q=parseNumber('Q',-1);
  int stepPin,dirPin;
  
  switch(q) {
    case 0:  stepPin = MOTOR_0_STEP_PIN;  dirPin=MOTOR_0_DIR_PIN;  break;
    case 1:  stepPin = MOTOR_1_STEP_PIN;  dirPin=MOTOR_1_DIR_PIN;  break;
    case 2:  stepPin = MOTOR_2_STEP_PIN;  dirPin=MOTOR_2_DIR_PIN;  break;
    case 3:  stepPin = MOTOR_3_STEP_PIN;  dirPin=MOTOR_3_DIR_PIN;  break;
    default:
      Serial.println(F("Invalid motor number. 0=A,1=B,2=C,3=D."));
      return;
  }
  
  digitalWrite(dirPin,amount < 0 ? motors[0].reel_in : motors[0].reel_out);
  findStepDelay();
  int i;
  for(i=0;i<STEPS_PER_TURN;++i) {
    digitalWrite(stepPin,HIGH);
    digitalWrite(stepPin,LOW);
    pause(step_delay);
  }
}


// equal to three decimal places?
boolean equalEpsilon(float a,float b) {
  int aa = a*10;
  int bb = b*10;
  //Serial.print("aa=");        Serial.print(aa);
  //Serial.print("\tbb=");      Serial.print(bb);
  //Serial.print("\taa==bb ");  Serial.println(aa==bb?"yes":"no");
  
  return aa==bb;
}


void setHome(float x,float y,float z) {
  boolean dx = equalEpsilon(x,homeX);
  boolean dy = equalEpsilon(y,homeY);
  boolean dz = equalEpsilon(z,homeZ);
  if( dx==false || dy==false || dz==false ) {
    //Serial.print(F("Was    "));    Serial.print(homeX);    Serial.print(',');    Serial.println(homeY);
    //Serial.print(F("Is now "));    Serial.print(    x);    Serial.print(',');    Serial.println(    y);
    //Serial.print(F("DX="));    Serial.println(dx?"true":"false");
    //Serial.print(F("DY="));    Serial.println(dy?"true":"false");
    homeX = x;
    homeY = y;
    homeZ = z;
    saveHome();
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void parser_ready() {
  sofar=0;  // clear input buffer
  Serial.print(F("\n> "));  // signal ready to receive input
  last_cmd_time = millis();
}


/**
 * reset all tool offsets
 */
void tools_setup() {
  for(int i=0;i<NUM_TOOLS;++i) {
    set_tool_offset(i,0,0,0);
  }
}


/**
 * runs once on machine start
 */
void setup() {
  // start communications
  Serial.begin(BAUD);

  loadConfig();

  motor_setup();
  motor_engage();
  tools_setup();

  //easyPWM_init();
  SD_init();
  LCD_init();

  // initialize the plotter position.
  teleport(0,0,0);
  setFeedRate(DEFAULT_FEEDRATE);
  
  // display the help at startup.
  help();
  
  parser_ready();
}


/**
 * See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
 */
void Serial_listen() {
  // listen for serial commands
  while(Serial.available() > 0) {
    char c = Serial.read();
    if(c=='\r') continue;
    if(sofar<MAX_BUF) serialBuffer[sofar++]=c;
    if(c=='\n') {
      serialBuffer[sofar]=0;

      // echo confirmation
//      Serial.println(F(serialBuffer));

      // do something with the command
      processCommand();
      parser_ready();
    }
  }
}


/**
 * main loop
 */
void loop() {
  Serial_listen();
  SD_check();
#ifdef HAS_LCD
  LCD_update();
#endif

  // The PC will wait forever for the ready signal.
  // if Arduino hasn't received a new instruction in a while, send ready() again
  // just in case USB garbled ready and each half is waiting on the other.
  if( !segment_buffer_full() && (millis() - last_cmd_time) > TIMEOUT_OK ) {
    parser_ready();
  }
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
