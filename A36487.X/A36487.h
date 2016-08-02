// PULSE SYNCHRONIZATION BOARD FIRMWARE
#ifndef __A36487_h
#define __A36487_h

#include <XC.h>
#include <libpic30.h>
#include <timer.h>
#include <uart.h>
#include <adc12.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"


/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus
  PIN_G14 - Used/Configured by ETM CAN 
  PIN_G12 - Used/Configured by ETM CAN 
  PIN_C1  - Used/Configured by ETM CAN 

  SPI2   - Used/Configured by Serial Shift Delay Line Software

  Timer1 - Used to measure PRF and 
  Timer2 - Used for 10ms Timing
  Timer3 - Not used

  INT3 - Used for Pulse Trigger ISR

  UART2 - Use for dose command from customer

  ADC Module - Not Used

*/


//Oscillator Setup
#define FCY             10000000    //40MHz clock / 4 = 10MHz


//These values are calculated or measured by the pulse sync board
typedef struct{
  unsigned int led_flash_counter;        // This is used to time the on board LED flashing
  unsigned int counter_config_received;  // This is used to determine what configuration data has been received
  unsigned int control_state;            // This is the state of the state machine
  unsigned int last_period;              // The was the last PFR period
  unsigned int period_filtered;          // This is an filtered version of the PRF
  unsigned int pulses_on;                // This counts the number of pusles, sent out to each board so that data between pulses can be synced
  unsigned int trigger_complete;         // This bit is set when the trigger ISR occurs
  unsigned int led_state;                // DPARKER - HOW IS THIS USED???? As far as I can tell it isn't        
  unsigned char personality;             // DPARKER - This is not working at the moment //0=UL, 1=L, 2=M, 3=H
  unsigned int  next_pulse_level_energy_command;  // This stores the next pulse level command, it is loaded into this_pulse_level_energy_command after a trigger
  unsigned int  this_pulse_level_energy_command;
  
  TYPE_DIGITAL_INPUT pfn_fan_fault;
  
} TYPE_GLOBAL_DATA_A36487;


// STATES
#define STATE_INIT              10
#define STATE_WAIT_FOR_CONFIG   20
#define STATE_HV_OFF            30
#define STATE_HV_ENABLE         40
#define STATE_X_RAY_ENABLE      50
#define STATE_FAULT             60
#define STATE_UNKNOWN           70


#define _FAULT_TIMING_MISMATCH                     _FAULT_0
#define _FAULT_CAN_COMMUNICATION_LATCHED           _FAULT_1
#define _FAULT_RF_STATUS                           _FAULT_2
#define _FAULT_PFN_STATUS                          _FAULT_3
#define _FAULT_TRIGGER_STAYED_ON                   _FAULT_4
#define _FAULT_PANEL_OPEN                          _FAULT_5
#define _FAULT_KEYLOCK_OPEN                        _FAULT_6


#define _STATUS_CUSTOMER_HV_DISABLE                _WARNING_0
#define _STATUS_CUSTOMER_X_RAY_DISABLE             _WARNING_1
#define _STATUS_LOW_MODE_OVERRIDE                  _WARNING_2
#define _STATUS_HIGH_MODE_OVERRIDE                 _WARNING_3
#define _STATUS_PERSONALITY_READ_COMPLETE          _WARNING_4
//#define _STATUS_PANEL_OPEN                       _WARNING_5
//#define _STATUS_KEYLOCK_OPEN                     _WARNING_6
#define _STATUS_TRIGGER_STAYED_ON                  _WARNING_7


#define _PERSONALITY_BIT_0                         _NOT_LOGGED_0
#define _PERSONALITY_BIT_1                         _NOT_LOGGED_1
#define _PERSONALITY_BIT_2                         _NOT_LOGGED_2
#define _PERSONALITY_BIT_3                         _NOT_LOGGED_3

#define LED_STARTUP_FLASH_TIME                     300 // 3 Seconds


// #defines that set up the log data variables
#define grid_start_high3          (*(unsigned char*)&slave_board_data.log_data[0])
#define grid_start_high2          (*((unsigned char*)&slave_board_data.log_data[0] + 1))
#define grid_start_high1          (*(unsigned char*)&slave_board_data.log_data[1])
#define grid_start_high0          (*((unsigned char*)&slave_board_data.log_data[1] + 1))
#define dose_sample_delay_high    (*(unsigned char*)&slave_board_data.log_data[2])
#define pfn_delay_high            (*((unsigned char*)&slave_board_data.log_data[2] + 1))

#define grid_stop_high3           (*(unsigned char*)&slave_board_data.log_data[4])
#define grid_stop_high2           (*((unsigned char*)&slave_board_data.log_data[4] + 1))
#define grid_stop_high1           (*(unsigned char*)&slave_board_data.log_data[5])
#define grid_stop_high0           (*((unsigned char*)&slave_board_data.log_data[5] + 1))
#define magnetron_current_sample_delay_high    (*(unsigned char*)&slave_board_data.log_data[6])
#define afc_delay_high            (*((unsigned char*)&slave_board_data.log_data[6] + 1))

#define grid_start_low3           (*(unsigned char*)&slave_board_data.log_data[8])
#define grid_start_low2           (*((unsigned char*)&slave_board_data.log_data[8] + 1))
#define grid_start_low1           (*(unsigned char*)&slave_board_data.log_data[9])
#define grid_start_low0           (*((unsigned char*)&slave_board_data.log_data[9] + 1))
#define dose_sample_delay_low     (*(unsigned char*)&slave_board_data.log_data[10])
#define pfn_delay_low             (*((unsigned char*)&slave_board_data.log_data[10] + 1))

#define grid_stop_low3            (*(unsigned char*)&slave_board_data.log_data[12])
#define grid_stop_low2            (*((unsigned char*)&slave_board_data.log_data[12] + 1))
#define grid_stop_low1            (*(unsigned char*)&slave_board_data.log_data[13])
#define grid_stop_low0            (*((unsigned char*)&slave_board_data.log_data[13] + 1))
#define magnetron_current_sample_delay_low    *(unsigned char*)&slave_board_data.log_data[14]
#define afc_delay_low             (*((unsigned char*)&slave_board_data.log_data[14] + 1))

#define log_data_rep_rate_deci_hertz        slave_board_data.log_data[3]

#define trigger_width             (*((unsigned char*)&slave_board_data.log_data[7] + 0))
#define trigger_width_filtered    (*((unsigned char*)&slave_board_data.log_data[7] + 1))

#define data_grid_start           (*(((unsigned char*)&slave_board_data.log_data[11]) + 0))
#define data_grid_stop            (*(((unsigned char*)&slave_board_data.log_data[11]) + 1))



// DIGITAL INPUT PINS
#define PIN_CUSTOMER_BEAM_ENABLE_IN         _RG2
#define ILL_CUSTOMER_BEAM_ENABLE            1

#define PIN_CUSTOMER_XRAY_ON_IN             _RG3
#define ILL_CUSTOMER_XRAY_ON                1



// DIGITAL OUTPUT PINS
#define PIN_CPU_HV_ENABLE_OUT               _RD8
#define OLL_CPU_HV_ENABLE                   1

#define PIN_CPU_XRAY_ENABLE_OUT             _RC13
#define OLL_CPU_XRAY_ENABLE                 1

#define PIN_CPU_WARNING_LAMP_OUT            _RD9
#define OLL_CPU_WARNING_LAMP                1






/*
  ----------  ADC CONFIGURATION --------------
  ADC is not used
  Disable the ADC and set all pins to digital Inputs
*/
  
#define ADCON1_SETTING  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADPCFG_SETTING  0xFFFF // All pins are digital Inputs





/*
  ------------ UART 2 Configuration -----------------------
  Use UART1, 625KBaud, Disable TX
*/
#define A36487_U2_MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36487_U2_STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_DISABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A36487_U2_BRG_VALUE         0 // 625KBaud at 10 MHz CLK
  

/* 
   ---------- TMR Configuration -----------
   Timer1 - Used to measure the PRF
   With 10Mhz Clock, x64 multiplier will yield max period of 419mS, 6.4 uS per Tick

   Timer2 - Used to measure 10ms
*/
    
#define T1CON_VALUE                    (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT)
#define PR1_VALUE_400mS                62500

#define T2CON_VALUE                    (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_SOURCE_INT & T2_32BIT_MODE_OFF)
#define PR2_VALUE_10mS                 12500                        



// ***Digital Pin Definitions***

// DPARKER - CLEAN UP PIN DEFFENITIONS and INITIALIZAITONS


// Customer Interface


// ----------- Output Pins ------------ 
/* 

   B4 - Ready Out
   B5 - Standby Out

   C2 - Shift Data Out
   C3 - Shift Clock

   D0 - Sumflt Out
   D5 - Reset Pulse Width Clock
   D10 - Warmup Out
   D11 - Shift Register
   D12 - Shift Register
   D13 - Shift Register

   G1  - RS 485 Driver Enable
   G13 - LED Ready
   G15 - LED Xray On


*/
 
#define A36487_TRISA_VALUE 0b1111111111111111
#define A36487_TRISB_VALUE 0b1111111111001111
#define A36487_TRISC_VALUE 0b1111111111110011
#define A36487_TRISD_VALUE 0b1100001111011110
#define A36487_TRISF_VALUE 0b1111111111111111
#define A36487_TRISG_VALUE 0b0101111111111101



// -----------  LED Outputs ------------------------
// PIN C1  - Managed by CAN Module
// PIN G12 - Managed by CAN Module
// PIN G14 - Managed by CAN Module

#define PIN_LED_XRAY_ON                     _LATG15
#define PIN_LED_READY                       _LATG13
#define OLL_LED_ON                          0



// ------------ Personality Module Interface --------------- //
#define PIN_ID_SHIFT_OUT                    _LATC2
#define OLL_ID_SHIFT                        1

#define PIN_ID_CLK_OUT                      _LATC3
#define OLL_ID_CLK                          1

#define PIN_ID_DATA_IN                      _RC4
#define ILL_ID_DATA                         1



// ----------------- CUSTOMER IO OUTPUTS ---------------  //
#define PIN_CPU_STANDBY_OUT                 _LATB5
#define OLL_CPU_STANDBY                     0

#define PIN_CPU_READY_OUT                   _LATB4
#define OLL_CPU_READY                       0

#define PIN_CPU_WARMUP_OUT                  _LATD10
#define OLL_CPU_WARMUP                      0

#define PIN_CPU_SUMFLT_OUT                  _LATD0
#define OLL_CPU_SUMFLT                      0


// ----------------- CUSTOMER IO INPUTS ---------------  //
#define PIN_LOW_MODE_IN                     _RB3
#define PIN_HIGH_MODE_IN                    _RB2
#define ILL_MODE_BIT_SELECTED               0 




// ------------ SYSTEM IO INPUTS ------------------ //
#define PIN_KEY_LOCK_IN                     _RF7
#define ILL_KEY_LOCK_FAULT                  1

#define PIN_PANEL_IN                        _RF8
#define ILL_PANEL_OPEN                      1

#define PIN_RF_OK                           _RA7
#define ILL_PIN_RF_FAULT                    0

#define PIN_PFN_OK                          _RG0
#define ILL_PIN_PFN_FAULT                   0

#define PIN_XRAY_CMD_MISMATCH_IN            _RD14
#define ILL_XRAY_CMD_MISMATCH               1


// ------------ LOAD SHIFT REGISTER OUTPUTS ------------------ //
#define PIN_LD_DELAY_GUN_OUT		    _LATD11
#define PIN_LD_DELAY_AFC_OUT		    _LATD13
#define PIN_LD_DELAY_PFN_OUT		    _LATD12


// ------------------ TRIGGER WIDTH INTERFACE --------------- //
#define PIN_TRIG_INPUT                      _RF6
#define ILL_TRIG_ON                         1

#define PIN_PW_CLR_CNT_OUT                  _LATD5
#define OLL_PW_CLR_CNT                      1

// UART
#define PIN_RS_485_DRIVER_ENABLE            _LATG1
#define OLL_RS_485_RECEIVE_MODE             0




// These defines are what is actually read from the shift register
#define HIGH_DOSE               0x77
#define MEDIUM_DOSE             0xCC
#define LOW_DOSE                0xAA
#define ULTRA_LOW_DOSE          0x99



#endif
