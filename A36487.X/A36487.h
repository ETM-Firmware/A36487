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
  OPTIONS
  6/4 Mev
  PLC Internface 
  Serial Dose
  (Disable Internal Trigger)
  (Disable Can Interface - Or not for debugging purposes)
  
  2.5 Mev
  Can Interface
  Serial Dose
  (Disable Internal Trigger)
  (Disable PLC Interface)

  NDT
  Can Interface
  Internal Trigger
  (Disable Serial Dose)
  (DIsable PLC Interface)
  
 */

#define __PLC_INTERFACE  // IF NOT, Implies Can interface 
#define __TRIGGER_AFC_HIGH_ONLY

//#define __INTERNAL_TRIGGER // - Does not Have Serial Dose
//#define __COMPILE_MODE_2_5


#define DELAY_PLC   2500000  // 2.5 million cycles
#define MIN_PERIOD  1


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
  Timer3 - Used to Limit the PRF

  Timer4/5 - Reserved for the CAN module

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
  unsigned char personality;             // DPARKER - This is not working at the moment //0=UL, 1=L, 2=M, 3=H

  
  TYPE_DIGITAL_INPUT pfn_fan_fault;
  
  AnalogInput  analog_1;
  AnalogInput  analog_2;
  AnalogInput  analog_3;
  AnalogInput  analog_4;
  AnalogInput  analog_5;
  AnalogInput  analog_6;
  AnalogInput  analog_7;
  AnalogInput  analog_8;


  unsigned int analog_interface_timer;
  unsigned int state_analog_data_read;
  unsigned int analog_register_select;
  unsigned int analog_interface_attempts;

  unsigned int prf_ok;
  unsigned int accumulator_counter;

  unsigned int this_pulse_level;
  unsigned int this_pulse_width;

  unsigned int next_pulse_level;
  unsigned int next_pulse_width;

  unsigned int message_received;

  unsigned int bad_message_count;
  unsigned int total_missed_messages;
  unsigned int message_received_count;


  unsigned int trigger_width_update_ready;


} TYPE_GLOBAL_DATA_A36487;


// STATES
#define STATE_INIT              10
#define STATE_WAIT_FOR_CONFIG   20
#define STATE_HV_OFF            30
#define STATE_HV_ENABLE         40
#define STATE_X_RAY_ENABLE      50
#define STATE_FAULT             60
#define STATE_UNKNOWN           70

// State for Analog Interface with PLC
#define ANALOG_STATE_WAIT_FOR_DATA_READ  10
#define ANALOG_STATE_WAIT_FOR_PLC_SET    20
#define ANALOG_STATE_WAIT_READ_COMPLETE  30
#define ANALOG_STATE_DATA_READ_FAULT     40


#define _FAULT_TIMING_MISMATCH                     _FAULT_0
#define _FAULT_CAN_COMMUNICATION_LATCHED           _FAULT_1
#define _FAULT_RF_STATUS                           _FAULT_2
#define _FAULT_PFN_STATUS                          _FAULT_3
//#define _FAULT_TRIGGER_STAYED_ON                   _FAULT_4
#define _FAULT_PANEL_OPEN                          _FAULT_5
#define _FAULT_KEYLOCK_OPEN                        _FAULT_6


#define _STATUS_CUSTOMER_HV_DISABLE                _WARNING_0
#define _STATUS_CUSTOMER_X_RAY_DISABLE             _WARNING_1
#define _STATUS_LOW_MODE_OVERRIDE                  _WARNING_2
#define _STATUS_HIGH_MODE_OVERRIDE                 _WARNING_3
#define _STATUS_PERSONALITY_READ_COMPLETE          _WARNING_4
//#define _STATUS_PANEL_OPEN                       _WARNING_5
//#define _STATUS_KEYLOCK_OPEN                     _WARNING_6
//#define _STATUS_TRIGGER_STAYED_ON                _WARNING_7


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

#define trigger_set_decihertz     slave_board_data.log_data[16] // DPARKER Confirm this location is ok with documentation


//  ----------  ADC CONFIGURATION --------------

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 400ns (4 clocks per ADC clock), Sample Time is 4 ADC Clock
  Total Conversion time is 18 TAD = 7.2uS
  8 Samples per Interrupt, use alternating buffers
  Scan Through Selected Inputs
  
  We can convert all 8 samples in 57.6uS

  128 samples of all 8 takes 7.37milliseconds
*/

#define ADCON1_SETTING_EN  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING_EN  (ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCHS_SETTING_EN   (ADC_CH0_POS_SAMPLEA_AN7 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN7 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING_EN  (ENABLE_AN8_ANA & ENABLE_AN9_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA & ENABLE_AN12_ANA & ENABLE_AN13_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)
#define ADCSSL_SETTING_EN  (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7)
#define ADCON3_SETTING_EN  (ADC_SAMPLE_TIME_11 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_4Tcy)

/*
  ADC is not used
  Disable the ADC and set all pins to digital Inputs
*/

#define ADCON1_SETTING_DIS  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING_DIS  0x0000
#define ADCON3_SETTING_DIS  0x0000
#define ADCHS_SETTING_DIS   0x0000
#define ADPCFG_SETTING_DIS  0xFFFF // All pins are digital Inputs
#define ADCSSL_SETTING_DIS  0x0000


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

#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT)
#define TMR3_DELAY_2400US              24000                        
#define TMR3_DELAY_200US                2000


#define T3CON_VALUE_PS_1               (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1   & T3_SOURCE_INT)
#define T3CON_VALUE_PS_8               (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8   & T3_SOURCE_INT)
#define T3CON_VALUE_PS_64              (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_64  & T3_SOURCE_INT)
#define T3CON_VALUE_PS_256             (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_256 & T3_SOURCE_INT)


/* 
   // DPARKER change ID -> PERSONALITY to make more clear

   
   ---------- Output Pins --------------
   C2 -  PIN_ID_SHIFT_OUT
   C3 -  PIN_ID_CLK_OUT
   C14 - PIN_ENERGY_CPU_OUT
   D1  - PIN_RF_POLARITY_OUT
   D2  - PIN_HVPS_POLARITY_OUT
   D3  - PIN_GUN_POLARITY_OUT
   D4  - PIN_CPU_START_OUT
   D7  - PIN_AFC_TRIGGER_ENABLE_OUT
   D9  - PIN_CPU_WARNING_LAMP_OUT
   D11 - PIN_LD_DELAY_GUN_OUT
   D12 - PIN_LD_DELAY_PFN_OUT
   D13 - PIN_LD_DELAY_AFC_OUT
   D15 - PIN_ANALOG_READ_COMPLETE_OUT 
   F2  - PIN_PORTAL_GANTRY_MODE_OUT



   -------- INPUT PINS ------------
   A6  - PIN_CPU_GUNDRIVER_OK_IN
   A7  - PIN_CPU_RF_OK_IN
   A12 - PIN_TRIGGER_IN
   B2  - PIN_HIGH_MODE_IN
   B3  - PIN_LOW_MODE_IN
   C4  - PIN_ID_DATA_IN
   C13 - PIN_CPU_XRAY_ENABLE_IN
   D8  - PIN_CPU_HV_ENABLE_IN
   D14 - PIN_XRAY_CMD_MISMATCH_IN
   F6  - PIN_PACKAGE_VALID_IN
   F7  - PIN_KEY_LOCK_IN
   F8  - PIN_PANEL_IN
   G0  - PIN_CPU_PFN_OK_IN
   G2  - PIN_CUSTOMER_BEAM_ENABLE_IN
   G3  - PIN_CUSTOMER_XRAY_ON_IN

   // DPARKER - MORE POSSIBLE FAULT INPUTS - POSSIBLY NOT USED




*/
#define PIN_ID_SHIFT_OUT                  _LATC2
#define PIN_ID_CLK_OUT                    _LATC3
#define PIN_ENERGY_CPU_OUT                _LATC14
#define PIN_RF_POLARITY_OUT               _LATD1
#define PIN_HVPS_POLARITY_OUT             _LATD2
#define PIN_GUN_POLARITY_OUT              _LATD3
#define PIN_CPU_START_OUT                 _LATD4
#define PIN_AFC_TRIGGER_ENABLE_OUT        _LATD7
#define PIN_CPU_WARNING_LAMP_OUT          _LATD9
#define PIN_LD_DELAY_GUN_OUT              _LATD11
#define PIN_LD_DELAY_PFN_OUT              _LATD12
#define PIN_LD_DELAY_AFC_OUT              _LATD13
#define PIN_ANALOG_READ_COMPLETE_OUT      _LATD15
#define PIN_PORTAL_GANTRY_MODE_OUT        _LATF2

#define PIN_40US_TEST_POINT               _LATA15


#define OLL_CPU_WARNING_LAMP                1
#define OLL_ANALOG_READ_COMPLETE            1
#define OLL_CPU_START                       1
#define OLL_AFC_TRIGGER_ENABLE              1
#define OLL_ENERGY_LEVEL_HIGH               1
#define OLL_POLARITY_NORMAL                 0

#define PIN_CPU_GUNDRIVER_OK_IN           _RA6
#define PIN_CPU_RF_OK_IN                  _RA7
#define PIN_TRIGGER_IN                    _RA12
#define PIN_HIGH_MODE_IN                  _RB2
#define PIN_LOW_MODE_IN                   _RB3
#define PIN_ID_DATA_IN                    _RC4
#define PIN_CPU_XRAY_ENABLE_IN            _RC13
#define PIN_CPU_HV_ENABLE_IN              _RD8
#define PIN_XRAY_CMD_MISMATCH_IN          _RD14
#define PIN_PACKAGE_VALID_IN              _RF6
#define PIN_KEY_LOCK_IN                   _RF7   // Only looked at by CAN code
#define PIN_PANEL_IN                      _RF8   // Only looked at by CAN code
#define PIN_CPU_PFN_OK_IN                 _RG0
#define PIN_CUSTOMER_BEAM_ENABLE_IN       _RG2
#define PIN_CUSTOMER_XRAY_ON_IN           _RG3



#define ILL_CUSTOMER_BEAM_ENABLE          1
#define ILL_CUSTOMER_XRAY_ON              1
#define ILL_MODE_BIT_SELECTED             1
#define ILL_KEY_LOCK_FAULT                1
#define ILL_PANEL_OPEN                    1
#define ILL_PIN_RF_FAULT                  0
#define ILL_PIN_PFN_FAULT                 0
#define ILL_PIN_GUN_FAULT                 0
#define ILL_XRAY_CMD_MISMATCH             1



#define ILL_PLC_ENABLE_HV                 1
#define ILL_PLC_READY                     1
#define ILL_PACKAGE_VALID                 1
#define ILL_TRIGGER_ACTIVE                1



//-------- MUST BE OUTPUTS IN CAN MODE AND INPUTS IN PLC MODE ----------------- //
#define PIN_CPU_READY_OUT                   _LATB4
#define PIN_CPU_STANDBY_OUT                 _LATB5
#define PIN_CPU_XRAY_ENABLE_OUT             _LATC13
#define PIN_CPU_SUMFLT_OUT                  _LATD0
//#define PIN_CPU_HV_ENABLE_OUT               _LATD8 DPARKER testing only
#define PIN_CPU_HV_ENABLE_OUT               _LATG15
#define PIN_CPU_WARMUP_OUT                  _LATD10

#define OLL_CPU_READY                       1
#define OLL_CPU_STANDBY                     1
#define OLL_CPU_XRAY_ENABLE                 1
#define OLL_CPU_SUMFLT                      1
#define OLL_CPU_HV_ENABLE                   1
#define OLL_CPU_WARMUP                      1







#ifdef __PLC_INTERFACE

#define A36487_TRISA_VALUE 0b0111111111111111
#define A36487_TRISB_VALUE 0b1111111111111111
#define A36487_TRISC_VALUE 0b1011111111110011
#define A36487_TRISD_VALUE 0b0100010101100001
#define A36487_TRISF_VALUE 0b1111111111111011
#define A36487_TRISG_VALUE 0b0101111111111111

#else

#define A36487_TRISA_VALUE 0b1111111111111111
#define A36487_TRISB_VALUE 0b1111111111001111
#define A36487_TRISC_VALUE 0b1001111111110011
#define A36487_TRISD_VALUE 0b0100000001100000
#define A36487_TRISF_VALUE 0b1111111111111011
#define A36487_TRISG_VALUE 0b0101111111111111

#endif





#ifdef __PLC_INTERFACE

#define _MACRO_HV_ENABLE         (PIN_CPU_HV_ENABLE_IN == ILL_PLC_ENABLE_HV)
#define _MACRO_NOT_HV_ENABLE     (PIN_CPU_HV_ENABLE_IN == !ILL_PLC_ENABLE_HV)

#define _MACRO_XRAY_ENABLE       ((PIN_CPU_XRAY_ENABLE_IN == ILL_PLC_READY) && ((PIN_LOW_MODE_IN == ILL_MODE_BIT_SELECTED) || (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED)))
#define _MACRO_NOT_XRAY_ENABLE   ((PIN_CPU_XRAY_ENABLE_IN == !ILL_PLC_READY) || ((PIN_LOW_MODE_IN == !ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == !ILL_MODE_BIT_SELECTED)))

#else

#define _MACRO_HV_ENABLE         (ETMCanSlaveGetSyncMsgPulseSyncDisableHV() == 0)
#define _MACRO_NOT_HV_ENABLE     (ETMCanSlaveGetSyncMsgPulseSyncDisableHV())


#define _MACRO_XRAY_ENABLE       (ETMCanSlaveGetSyncMsgPulseSyncDisableXray() == 0)
#define _MACRO_NOT_XRAY_ENABLE   (ETMCanSlaveGetSyncMsgPulseSyncDisableXray())

#endif







// Customer Interface


// ----------- Output Pins ------------ 
/* 

   B4 - Ready Out
   B5 - Standby Out

   C2 - Shift Data Out
   C3 - Shift Clock
   C13 - Xray Enable

   D0 - Sumflt Out
   D5 - Reset Pulse Width Clock
   D6 - Pic PRF OK
   D8 - HV Enable Out
   D10 - Warmup Out
   D11 - Shift Register
   D12 - Shift Register
   D13 - Shift Register

   G1  - RS 485 Driver Enable
   G13 - LED Ready
   G15 - LED Xray On


*/
 




// -----------  LED Outputs ------------------------
// PIN C1  - Managed by CAN Module
// PIN G12 - Managed by CAN Module
// PIN G14 - Managed by CAN Module

#define PIN_LED_XRAY_ON                     _LATG15
#define PIN_LED_READY                       _LATG13
#define OLL_LED_ON                          0



// These defines are what is actually read from the shift register
#define HIGH_DOSE               0x77
#define MEDIUM_DOSE             0xCC
#define LOW_DOSE                0xAA
#define ULTRA_LOW_DOSE          0x99




#ifdef __PLC_INTERFACE
  // FORCE RESET TO BE ACTIVE
#define ETMCanSlaveGetSyncMsgResetEnable() (1)
  // DPARKER need to confirm that this works

#define ETMCanSlaveGetComFaultStatus() (0)

#endif  



#endif
