#ifndef CONFIGURE_H
#define CONFIGURE_H
//------------------------------------------------------------------------------
// Skycam - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2017-08-09
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Skycam for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Sanity check
//------------------------------------------------------------------------------

// wrong board type set
#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define VERBOSE           (1)  // add to get a lot more serial output.

#define SKYCAM_HARDWARE_VERSION 1

// for serial comms
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?


#define MICROSTEPS           (16.0)  // microstepping on this microcontroller
#define STEPS_PER_TURN       (400.0 * MICROSTEPS)  // default number of steps per turn * microsteps

#define MAX_FEEDRATE         (9000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (7000.0)
#define DEFAULT_ACCELERATION (2500)

#define PULLEY_PITCH         (4.0)  // 20 gt2 teeth = 40mm, or 4cm.
#define THREAD_PER_STEP      (PULLEY_PITCH / STEPS_PER_TURN)

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define NUM_MOTORS           (4)
#define NUM_TOOLS            (6)
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.
#define SEGMOD(x)            ((x)&(MAX_SEGMENTS-1))

// for arc directions
#define ARC_CW               (1)
#define ARC_CCW              (-1)
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?


// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49

#define LCD_HEIGHT         4
#define LCD_WIDTH          20

#define BLEN_C             2
#define BLEN_B             1
#define BLEN_A             0
#define encrot0            0
#define encrot1            2
#define encrot2            3
#define encrot3            1

// Board types.  Don't change this!
#define BOARD_RUMBA 1
#define BOARD_RAMPS 2

// Your choice of board
#define MOTHERBOARD BOARD_RUMBA

#define HAS_SD                   // comment this out if there is no SD card
#define HAS_LCD                  // comment this out if there is no SMART LCD controller


#define MAX_MOTORS                 (6)

#define MOTOR_0_DIR_PIN           (16)
#define MOTOR_0_STEP_PIN          (17)
#define MOTOR_0_ENABLE_PIN        (48)
#define MOTOR_0_LIMIT_SWITCH_PIN  (37)

#define MOTOR_1_DIR_PIN           (47)
#define MOTOR_1_STEP_PIN          (54)
#define MOTOR_1_ENABLE_PIN        (55)
#define MOTOR_1_LIMIT_SWITCH_PIN  (36)

// alternate pins in case you want to do something interesting
#define MOTOR_2_DIR_PIN           (56)
#define MOTOR_2_STEP_PIN          (57)
#define MOTOR_2_ENABLE_PIN        (62)
#define MOTOR_2_LIMIT_SWITCH_PIN  (35)

#define MOTOR_3_DIR_PIN           (22)
#define MOTOR_3_STEP_PIN          (23)
#define MOTOR_3_ENABLE_PIN        (24)
#define MOTOR_3_LIMIT_SWITCH_PIN  (34)

#define MOTOR_4_DIR_PIN           (25)
#define MOTOR_4_STEP_PIN          (26)
#define MOTOR_4_ENABLE_PIN        (27)
#define MOTOR_4_LIMIT_SWITCH_PIN  (33)

#define MOTOR_5_DIR_PIN           (28)
#define MOTOR_5_STEP_PIN          (29)
#define MOTOR_5_ENABLE_PIN        (39)
#define MOTOR_5_LIMIT_SWITCH_PIN  (32)

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

// Smart controller settings
#define BEEPER             44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40

// Encoder rotation values
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43


#if NUM_MOTORS > MAX_MOTORS
#error "The number of motors needed is more than this board supports."
#endif


//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define EEPROM_VERSION          1  // Increment EEPROM_VERSION when adding new variables
#define ADDR_VERSION            0                          // 0..255 (1 byte)
#define ADDR_UUID               (ADDR_VERSION+1)           // long - 4 bytes
#define ADDR_AX                 (ADDR_UUID+4)              // float - 4 bytes
#define ADDR_AY                 (ADDR_AX+4)                // float - 4 bytes
#define ADDR_AZ                 (ADDR_AY+4)                // float - 4 bytes
#define ADDR_BX                 (ADDR_AZ+4)                // float - 4 bytes
#define ADDR_BY                 (ADDR_BX+4)                // float - 4 bytes
#define ADDR_BZ                 (ADDR_BY+4)                // float - 4 bytes
#define ADDR_CX                 (ADDR_BZ+4)                // float - 4 bytes
#define ADDR_CY                 (ADDR_CX+4)                // float - 4 bytes
#define ADDR_CZ                 (ADDR_CY+4)                // float - 4 bytes
#define ADDR_DX                 (ADDR_CZ+4)                // float - 4 bytes
#define ADDR_DY                 (ADDR_DX+4)                // float - 4 bytes
#define ADDR_DZ                 (ADDR_DY+4)                // float - 4 bytes


//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------
// for timer interrupt control
#define CLOCK_FREQ            (16000000L)
#define MAX_COUNTER           (65536L)
// time passed with no instruction?  Make sure PC knows we are waiting.
#define TIMEOUT_OK            (1000)

// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START


//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long step_count;
  long delta;  // number of steps to move
  long absdelta;
  int dir;
  float delta_normalized;
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
  int limit_switch_state;
  int reel_in;
  int reel_out;
} Motor;


typedef struct {
  Axis a[NUM_MOTORS];
  int steps_total;
  int steps_taken;
  int accel_until;
  int decel_after;
  unsigned short feed_rate_max;
  unsigned short feed_rate_start;
  unsigned short feed_rate_start_max;
  unsigned short feed_rate_end;
  char nominal_length_flag;
  char recalculate_flag;
  char busy;
} Segment;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern Segment line_segments[MAX_SEGMENTS];
extern Segment *working_seg;
extern volatile int current_segment;
extern volatile int last_segment;
extern float acceleration;
extern Motor motors[NUM_MOTORS];
extern const char *AxisLetters;
extern const char *motorNames;

#endif // CONFIGURE_H
