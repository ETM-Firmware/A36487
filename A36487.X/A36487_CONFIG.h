#ifndef __A36487_CONFIG_h
#define __A36487_CONFIG_h



#define MIN_REQUESTED_PRF_DECIHERTZ   10
#define MAX_REQUESTED_PRF_DECIHERTZ   5000



// DPARKER import the compile options here so these are only valid on external trigger mode
#define MAX_TRIGGER_HIGH_TIME_TMR1_UNITS        100  // Each TMR1 unit is 6.4us so this is 640uS
#define TRIGGER_STAYED_HIGH_FAULT_LEVEL         5
#define TRIGGER_NOT_VALID_FAULT_LEVEL           5
#define TRIGGER_PERIOD_TOO_SHORT_FAULT_LEVEL    5
#define TRIGGER_LENGTH_TOO_SHORT_FAULT_LEVEL    5
#define TRIGGER_COUNTER_DECREMENT_INTERVAL      1000

#define TRIGGER_MAX_BAD_MESSAGE_COUNT           8


#endif
