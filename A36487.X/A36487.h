// PULSE SYNCHRONIZATION BOARD FIRMWARE
//
// DESCRIPTION:
//      1. All the action happens here
//      2. Calling the initialize routine
//      3. Sampling the trigger width
//      4. Interpreting the customer energy command
//      5. Setting the appropriate energy levels
//      6. Programing the delay lines for the trigger delays/widths
//      7. Reading the personality module
//

#ifndef __A36487_h
#define __A36487_h

#include <XC.h>
#include <libpic30.h>
#include <timer.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"


//Oscillator Setup
#define FCY             10000000    //40MHz clock / 4 = 10MHz


//These values are calculated or measured by the pulse sync board
typedef struct{
    unsigned int led_flash_counter;
    unsigned int counter_config_received;
    unsigned int state_machine;
    unsigned int led_state;        
    unsigned int last_period;
    unsigned int period_filtered;
    unsigned int pulses_on;
    unsigned int trigger_complete;
    unsigned char dose_sample_delay;         //calculated RF PCB Delay (target current)
    unsigned char pfn_delay;
    unsigned char afc_delay;
    unsigned char magnetron_current_sample_delay;

    TYPE_DIGITAL_INPUT pfn_fan_fault;
    
    
    unsigned char personality;      //0=UL, 1=L, 2=M, 3=H
    unsigned char last_trigger_filtered;
    unsigned char energy;
} PSB_DATA;

//Definitions
#define DOSE_LEVELS     11   //sets the amount of bits to converge into single dose level
#define HI              1
#define LOW             0

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
#define _FAULT_PANEL_OPEN                         _FAULT_5
#define _FAULT_KEYLOCK_OPEN                       _FAULT_6

#define _STATUS_CUSTOMER_HV_DISABLE                _WARNING_0
#define _STATUS_CUSTOMER_X_RAY_DISABLE             _WARNING_1
#define _STATUS_LOW_MODE_OVERRIDE                  _WARNING_2
#define _STATUS_HIGH_MODE_OVERRIDE                 _WARNING_3
#define _STATUS_PERSONALITY_READ_COMPLETE          _WARNING_4
//#define _STATUS_PANEL_OPEN                         _WARNING_5
//#define _STATUS_KEYLOCK_OPEN                       _WARNING_6
#define _STATUS_TRIGGER_STAYED_ON                  _WARNING_7


#define _PERSONALITY_BIT_0                         _NOT_LOGGED_0
#define _PERSONALITY_BIT_1                         _NOT_LOGGED_1
#define _PERSONALITY_BIT_2                         _NOT_LOGGED_2
#define _PERSONALITY_BIT_3                         _NOT_LOGGED_3



#define LED_WARMUP_STATUS                         (psb_data.led_state & 0x0001)
#define LED_STANDBY_STATUS                        (psb_data.led_state & 0x0002)
#define LED_READY_STATUS                          (psb_data.led_state & 0x0004)
#define LED_SUM_FAULT_STATUS                      (psb_data.led_state & 0x0008)

#define LED_STARTUP_FLASH_TIME                    300 // 3 Seconds



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

/*

// BufferByte Setup
#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02
*/
// Various definitions
#define TRIS_OUTPUT_MODE 0
#define TRIS_INPUT_MODE  1

// ***Digital Pin Definitions***

// Personality module
#define PIN_ID_SHIFT_OUT                    _LATC2
#define TRIS_PIN_ID_SHIFT_OUT               _TRISC2
#define OLL_ID_SHIFT                        1
#define PIN_ID_CLK_OUT                      _LATC3
#define TRIS_PIN_ID_CLK_OUT                 _TRISC3
#define OLL_ID_CLK                          1
#define PIN_ID_DATA_IN                      _RC4
#define TRIS_PIN_ID_DATA_IN                 _TRISC4
#define ILL_ID_DATA                         1

// Spare (not used in current application)
#define TRIS_PIN_PACKAGE_ID1_IN             _TRISF3
#define PIN_PACKAGE_ID1_IN                  _RF3
#define ILL_PACKAGE_ID1_OK                  0
#define TRIS_PIN_READY_FOR_ANALOG_OUT       _TRISD15	//READY / !ADJUSTING
#define PIN_READY_FOR_ANALOG_OUT            _LATD15
#define OLL_READY_FOR_ANALOG                1

//Control to PFN control board for Gantry/Portal Selection
#define TRIS_PIN_MODE_OUT                   _TRISF2
#define PIN_MODE_OUT                        _RF2
#define OLL_MODE_GANTRY                     1
#define OLL_MODE_PORTAL                     0

// STATUS from board A35487
#define PIN_XRAY_CMD_MISMATCH_IN            _RD14
#define TRIS_PIN_XRAY_CMD_MISMATCH_IN       _TRISD14
#define ILL_XRAY_CMD_MISMATCH               1
#define PIN_LOW_MODE_IN                     _RB3
#define TRIS_PIN_LOW_MODE_IN                _TRISB3
#define ILL_LOW_MODE                        1
#define PIN_HIGH_MODE_IN                    _RB2
#define TRIS_PIN_HIGH_MODE_IN               _TRISB2
#define ILL_HIGH_MODE                       1

#define TRIS_PIN_KEY_LOCK_IN                _TRISF7	 
#define PIN_KEY_LOCK_IN                     _RF7
#define ILL_KEY_LOCK_FAULT                  1

#define TRIS_PIN_PANEL_IN                   _TRISF8	
#define PIN_PANEL_IN                        _RF8
#define ILL_PANEL_OPEN                      1

// CONTROL to board A35487
#define TRIS_PIN_CUSTOMER_BEAM_ENABLE_IN    _TRISG2
#define PIN_CUSTOMER_BEAM_ENABLE_IN         _RG2
#define ILL_CUSTOMER_BEAM_ENABLE            1
#define TRIS_PIN_CUSTOMER_XRAY_ON_IN        _TRISG3	
#define PIN_CUSTOMER_XRAY_ON_IN             _RG3
#define ILL_CUSTOMER_XRAY_ON                1
#define TRIS_PIN_CPU_XRAY_ENABLE_OUT        _TRISC13
#define TRIS_PIN_CPU_HV_ENABLE_OUT          _TRISD8
#define PIN_CPU_HV_ENABLE_OUT               _RD8
#define OLL_CPU_HV_ENABLE                   1
#define PIN_CPU_XRAY_ENABLE_OUT             _RC13
#define OLL_CPU_XRAY_ENABLE                 1
#define TRIS_PIN_CPU_WARNING_LAMP_OUT       _TRISD9
#define PIN_CPU_WARNING_LAMP_OUT            _RD9
#define OLL_CPU_WARNING_LAMP                1

// Customer Status pins
#define PIN_CPU_STANDBY_OUT                 _RB5
#define TRIS_PIN_CPU_STANDBY_OUT            _TRISB5
#define OLL_CPU_STANDBY                     0
#define PIN_CPU_READY_OUT                   _RB4
#define TRIS_PIN_CPU_READY_OUT              _TRISB4
#define OLL_CPU_READY                       0
#define TRIS_PIN_CPU_SUMFLT_OUT             _TRISD0
#define PIN_CPU_SUMFLT_OUT                  _RD0
#define OLL_CPU_SUMFLT                      0
#define TRIS_PIN_CPU_WARMUP_OUT             _TRISD10
#define PIN_CPU_WARMUP_OUT                  _RD10
#define OLL_CPU_WARMUP                      0

//     LEDS
#define PIN_LED_READY                       _LATG13
#define TRIS_PIN_LED_READY                  _TRISG13
#define OLL_LED_ON                          0
//#define PIN_LED_STANDBY                     _LATG12  // This is used by CAN module to indicate board alive
//#define TRIS_PIN_LED_STANDBY                _TRISG12
//#define PIN_LED_WARMUP                      _LATG14  //This is used for CAN status
//#define TRIS_PIN_LED_WARMUP                 _TRISG14
#define PIN_LED_XRAY_ON                     _LATG15
#define TRIS_PIN_LED_XRAY_ON                _TRISG15
//#define PIN_LED_SUMFLT                      _LATC1  // This is used by Can module to indicate not ready
//#define TRIS_PIN_LED_SUMFLT                 _TRISC1

//Energy Pins
#define TRIS_PIN_ENERGY_CPU_OUT             _TRISC14
#define PIN_ENERGY_CPU_OUT                  _LATC14
#define OLL_ENERGY_CPU                      0
#define TRIS_PIN_AFC_TRIGGER_OK_OUT         _TRISD7
#define PIN_AFC_TRIGGER_OK_OUT              _LATD7
#define OLL_AFC_TRIGGER_OK                  1
#define TRIS_PIN_RF_POLARITY_OUT            _TRISD1
#define PIN_RF_POLARITY_OUT                 _LATD1
#define OLL_RF_POLARITY                     0
#define TRIS_PIN_HVPS_POLARITY_OUT          _TRISD2
#define PIN_HVPS_POLARITY_OUT		    _LATD2
#define OLL_HVPS_POLARITY                   0
#define TRIS_PIN_GUN_POLARITY_OUT           _TRISD3
#define PIN_GUN_POLARITY_OUT		    _LATD3
#define OLL_GUN_POLARITY                    0

// Pins for the delay line shift registers
#define TRIS_PIN_SPI_CLK_OUT                _TRISG6
#define PIN_SPI_CLK_OUT                     _LATG6
#define TRIS_PIN_SPI_DATA_OUT               _TRISG8
#define PIN_SPI_DATA_OUT                    _LATG8
#define TRIS_PIN_SPI_DATA_IN                _TRISG7
#define PIN_SPI_DATA_IN                     _RG7
#define TRIS_PIN_LD_DELAY_PFN_OUT           _TRISD12
#define PIN_LD_DELAY_PFN_OUT		    _LATD12
#define TRIS_PIN_LD_DELAY_AFC_OUT           _TRISD13
#define PIN_LD_DELAY_AFC_OUT		    _LATD13
#define TRIS_PIN_LD_DELAY_GUN_OUT           _TRISD11
#define PIN_LD_DELAY_GUN_OUT		    _LATD11

// Trigger Pulse width measure
#define TRIS_PIN_PW_SHIFT_OUT               _TRISD4
#define PIN_PW_SHIFT_OUT                    _LATD4
#define OLL_PW_SHIFT                        1
#define TRIS_PIN_PW_CLR_CNT_OUT             _TRISD5
#define PIN_PW_CLR_CNT_OUT                  _LATD5
#define OLL_PW_CLR_CNT                      1
#define TRIS_PIN_PW_HOLD_LOWRESET_OUT       _TRISD6
#define PIN_PW_HOLD_LOWRESET_OUT            _LATD6
#define OLL_PW_HOLD_LOWRESET                1		 /* 1, hold VALID_PULSE, 0, reset the START and VALID_PULSE	*/
#define TRIS_PIN_TRIG_INPUT                 _TRISF6
#define PIN_TRIG_INPUT                      _RF6
#define ILL_TRIG_ON                         1

//Interrupt pins
#define TRIS_PIN_ENERGY_CMD_IN1             _TRISA12	//INT1
#define PIN_ENERGY_CMD_IN1		    _RA12
#define TRIS_PIN_ENERGY_CMD_IN2             _TRISA13	//INT2 tied to INT1
#define PIN_ENERGY_CMD_IN2		    _RA13
#define TRIS_PIN_40US_IN2                   _TRISA14	//INT3
#define PIN_40US_IN2                        _RA14
#define TRIS_PIN_40US_IN1                   _TRISA15	//INT4 tied to INT3
#define PIN_40US_IN1                        _RA15
#define COMM_DRIVER_ENABLE_TRIS             _TRISG1		//Enable the communications driver
#define COMM_DRIVER_ENABLE_PIN              _RG1

//Bypass these to allow xray on
#define TRIS_PIN_RF_OK                      _TRISA7
#define PIN_RF_OK                           _RA7
#define ILL_PIN_RF_FAULT                    0

#define TRIS_PIN_GUN_OK                     _TRISA6
#define PIN_GUN_OK                          _RA6

#define TRIS_PIN_PFN_OK                     _TRISG0
#define PIN_PFN_OK                          _RG0
#define ILL_PIN_PFN_FAULT                   0

//Communications
#define COMM_RX_TRIS                        _TRISF4		//U2RX
#define COMM_RX_PIN                         _RF4
#define COMM_TX_TRIS                        _TRISF5		//U2TX



// These defines are what is actually read from the shift register
#define HIGH_DOSE       0x77
#define MEDIUM_DOSE     0xCC
#define LOW_DOSE        0xAA
#define ULTRA_LOW_DOSE  0x99

//Pin requirements
#define PIN_ID_SHIFT_OUT	_LATC2
#define TRIS_PIN_ID_SHIFT_OUT   _TRISC2
#define OLL_ID_SHIFT            1

#define PIN_ID_CLK_OUT 		_LATC3
#define TRIS_PIN_ID_CLK_OUT	_TRISC3
#define OLL_ID_CLK   		1

#define PIN_ID_DATA_IN          _RC4
#define TRIS_PIN_ID_DATA_IN     _TRISC4
#define ILL_ID_DATA  		1




#endif
