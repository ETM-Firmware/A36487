#include "A36487.h"
#include "A36487_CONFIG.h"


#define PIN_GUN_HV_ON       PIN_HVPS_POLARITY_OUT
#define PIN_GUN_BEAM_ENABLE PIN_RF_POLARITY_OUT

#define OLL_ENABLE_GUN_HV   0
#define OLL_ENABLE_GUN_BEAM 0


/*
  DPARKER need to figure out and extend the ETM Can functions for high, low, cab scan mode so that all modules are more generic
*/

#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_0      0x3210
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_1      0x3211
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_ALL    0x3212


#define DOSE_COMMAND_LOW_ENERGY   0  
#define DOSE_COMMAND_HIGH_ENERGY  1


#define PULSE_LEVEL_CARGO_HIGH    1
#define PULSE_LEVEL_CARGO_LOW     0




// DPARKER - Move these to the CAN Module - All boards need to know if the energy is commanded high or low
// They need to be incorporated into the can_slave modules


// Global Variables
TYPE_GLOBAL_DATA_A36487 global_data_A36487;
unsigned char uart2_input_buffer[16];
unsigned int  uart2_next_byte;






void DoStateMachine(void);
void InitializeA36487(void);
unsigned char ReadDosePersonality(void);
void DoA36487(void);
void DoStartupLEDs(void);
void DoPostTriggerProcess(void);
void ProgramShiftRegistersDelays(void);
void ProgramShiftRegistersGrid(unsigned char dose_command);
unsigned int GetThisPulseLevel(void);
unsigned char InterpolateValue(unsigned int val_0, unsigned int val_1, unsigned int val_2, unsigned int val_3, unsigned char index);


void SetupT3PRFDeciHertz(unsigned int decihertz);
void UpdateAnalogDataFromPLC(void);
void ReadAnalogRegister(void);
int SendPersonalityToPLC(unsigned char id);



//Processor Setup
_FOSC(EC & CSW_FSCM_OFF); // Primary Oscillator without PLL and Startup with User Selected Oscillator Source, CLKOUT 10MHz is used to measure trigger width.
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_27 & PBOR_ON & MCLR_EN); // Brown out and Power on Timer settings
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



int main(void) {
  global_data_A36487.control_state = STATE_INIT;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  // LOOP Through the state machine is around 30uS Nominally, 50uS when it processes a sync command
  // Need to determine how much time processing a trigger takes.

  switch (global_data_A36487.control_state) {


  case STATE_INIT:
    InitializeA36487();
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    global_data_A36487.control_state = STATE_WAIT_FOR_CONFIG;
    break;

  case STATE_WAIT_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    global_data_A36487.counter_config_received = 0;
    while (global_data_A36487.control_state == STATE_WAIT_FOR_CONFIG) {
      DoA36487();
      DoStartupLEDs();
      PIN_CPU_WARMUP_OUT = !OLL_CPU_WARMUP;
      PIN_CPU_STANDBY_OUT = !OLL_CPU_STANDBY;
      PIN_CPU_READY_OUT = !OLL_CPU_READY;
      PIN_CPU_SUMFLT_OUT = OLL_CPU_SUMFLT;
      if ((global_data_A36487.led_flash_counter >= LED_STARTUP_FLASH_TIME) && (global_data_A36487.counter_config_received == 0b1111)) {
	global_data_A36487.control_state = STATE_HV_OFF;
      }

    }
    break;

  case STATE_HV_OFF:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    while (global_data_A36487.control_state == STATE_HV_OFF) {
      DoA36487();
      
      if (_MACRO_HV_ENABLE) {
	global_data_A36487.control_state = STATE_HV_ENABLE;
      }

      if (_FAULT_REGISTER) {
	global_data_A36487.control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_HV_ENABLE:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    while (global_data_A36487.control_state == STATE_HV_ENABLE) {
      DoA36487();

      if (PIN_CUSTOMER_BEAM_ENABLE_IN == ILL_CUSTOMER_BEAM_ENABLE) {
	PIN_GUN_HV_ON = OLL_ENABLE_GUN_HV;
      } else {
	PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
      }
      
      if ((_MACRO_XRAY_ENABLE) && (PIN_CUSTOMER_BEAM_ENABLE_IN == ILL_CUSTOMER_BEAM_ENABLE)) {
	global_data_A36487.control_state = STATE_X_RAY_ENABLE;
      }
      
      if (_MACRO_NOT_HV_ENABLE) {
	global_data_A36487.control_state = STATE_HV_OFF;
      }

      if (_FAULT_REGISTER) {
	global_data_A36487.control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_X_RAY_ENABLE:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = OLL_CPU_HV_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    while (global_data_A36487.control_state == STATE_X_RAY_ENABLE) {
      DoA36487();

      if (PIN_HIGH_MODE_IN == PIN_LOW_MODE_IN) {
	PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
      } else {
	PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
	
      }
      
      if (PIN_CUSTOMER_XRAY_ON_IN == ILL_CUSTOMER_XRAY_ON) {
	PIN_CPU_WARNING_LAMP_OUT = OLL_CPU_WARNING_LAMP;
	PIN_GUN_BEAM_ENABLE = OLL_ENABLE_GUN_BEAM;
      } else {
	PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
	PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
      }
      
      if ((_MACRO_NOT_XRAY_ENABLE) || (_MACRO_NOT_HV_ENABLE) || (PIN_CUSTOMER_BEAM_ENABLE_IN == !ILL_CUSTOMER_BEAM_ENABLE)) {
	global_data_A36487.control_state = STATE_HV_ENABLE;
      }

      if (_FAULT_REGISTER) {
	global_data_A36487.control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    while (global_data_A36487.control_state == STATE_FAULT) {
      DoA36487();
      
      if (_FAULT_REGISTER == 0) {
	global_data_A36487.control_state = STATE_HV_OFF;
      }
    }
    break;

      
  default:
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    PIN_GUN_HV_ON = !OLL_ENABLE_GUN_HV;
    PIN_GUN_BEAM_ENABLE = !OLL_ENABLE_GUN_BEAM;
    global_data_A36487.control_state = STATE_UNKNOWN;
    while (1) {
      DoA36487();
    }
    break;
    
  } // switch (global_data_A36487.control_state) {
}



void InitializeA36487(void) {
  // Initialize Pins
  /* 
     DPARKER FIX
  PIN_CPU_READY_OUT           = !OLL_CPU_READY;
  PIN_CPU_STANDBY_OUT         = !OLL_CPU_STANDBY;  
  PIN_ID_SHIFT_OUT            = 0;
  PIN_ID_CLK_OUT              = 0;
  PIN_CPU_SUMFLT_OUT          = !OLL_CPU_SUMFLT;
  PIN_PW_CLR_CNT_OUT          = !OLL_PW_CLR_CNT;                // clear active
  PIN_CPU_WARMUP_OUT          = !OLL_CPU_WARMUP;
  PIN_LD_DELAY_PFN_OUT        = 0;
  PIN_LD_DELAY_AFC_OUT        = 0;
  PIN_LD_DELAY_GUN_OUT        = 0;
  PIN_LED_READY               = !OLL_LED_ON;
  PIN_LED_XRAY_ON             = !OLL_LED_ON;
  */


  PIN_GUN_POLARITY_OUT  = OLL_POLARITY_NORMAL;
  PIN_RF_POLARITY_OUT   = OLL_POLARITY_NORMAL;
  PIN_HVPS_POLARITY_OUT = OLL_POLARITY_NORMAL;
  PIN_ENERGY_CPU_OUT = OLL_ENERGY_LEVEL_HIGH;
  
  //PIN_GUN_CAB_SCAN_FIBER_OUT = OLL_GUN_CAB_SCAN_SELECTED;
  PIN_AFC_TRIGGER_ENABLE_OUT = OLL_AFC_TRIGGER_ENABLE;
  //PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;

  // Initialize all I/O Registers
  TRISA = A36487_TRISA_VALUE;
  TRISB = A36487_TRISB_VALUE;
  TRISC = A36487_TRISC_VALUE;
  TRISD = A36487_TRISD_VALUE;
  TRISF = A36487_TRISF_VALUE;
  TRISG = A36487_TRISG_VALUE;

  // Setupt Timer 2 to generate 10mS Period
  T2CON = T2CON_VALUE;
  PR2 = PR2_VALUE_10mS;
  TMR2 = 0;
  _T2IF = 0;

  // Setup Timer 1 to measure interpulse period.
  T1CON = T1CON_VALUE;
  PR1 = PR1_VALUE_400mS;

  // Setupt Timer 3 to measure the Pulse Holdoff time to prevent over PRF  
  T3CON = T3CON_VALUE;
  PR3 = TMR3_DELAY_2400US;
  TMR3 = 0;
  _T3IF = 0;
  
  // Disable ADC Module
  ADCON1 = ADCON1_SETTING_DIS;  
  ADCON2 = ADCON2_SETTING_DIS;
  ADCON3 = ADCON3_SETTING_DIS;
  ADPCFG = ADPCFG_SETTING_DIS;
  ADCSSL = ADCSSL_SETTING_DIS;
  ADCHS  = ADCHS_SETTING_DIS;
  


  // ------------ Configure Interrupts for Different Operational Modes ------------ //

  // Set up external INT1 */
  // This is the trigger interrupt
  _INT1IF = 0;		// Clear Interrupt flag
  _INT1IE = 1;		// Enable INT1 Interrupt
  _INT1EP = 0; 	        // Interrupt on rising edge
  _INT1IP = 7;		// Set interrupt to high priority

  // Configure UART2 for communicating with Serial Dose Command
  U2BRG = A36487_U2_BRG_VALUE;
  U2STA = A36487_U2_STA_VALUE;
  U2MODE = A36487_U2_MODE_VALUE;
  uart2_next_byte = 0;  
  _U2RXIF = 0;
  _U2RXIP = 6;
  _U2RXIE = 0;  // DISABLE UART INT for P1500 - MAGNETX  


  // Read the personality module
  global_data_A36487.personality = 0;

  _STATUS_PERSONALITY_READ_COMPLETE = 0;
  if (global_data_A36487.personality != 0xFF) {
    _STATUS_PERSONALITY_READ_COMPLETE = 1;
  }
  
  _PERSONALITY_BIT_0 = 0;
  _PERSONALITY_BIT_1 = 0;
  _PERSONALITY_BIT_2 = 0;
  _PERSONALITY_BIT_3 = 0;
  
  if (global_data_A36487.personality & 0x0001) {
    _PERSONALITY_BIT_0 = 1;
  }

  if (global_data_A36487.personality & 0x0002) {
    _PERSONALITY_BIT_1 = 1;
  }

  if (global_data_A36487.personality & 0x0004) {
    _PERSONALITY_BIT_2 = 1;
  }

  if (global_data_A36487.personality & 0x0008) {
    _PERSONALITY_BIT_3 = 1;
  }

  // Initialize the Slave Can Module
  _STATUS_CUSTOMER_X_RAY_DISABLE = 1;
  ETMEEPromUseInternal();
  ETMCanSlaveInitialize(CAN_PORT_1, FCY, ETM_CAN_ADDR_PULSE_SYNC_BOARD, _PIN_RG14, 4, _PIN_RG12, _PIN_RC1);
  ETMCanSlaveLoadConfiguration(36487, 002, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);
  
  // Configure SPI Module to write to the delay line sift Registers
  ConfigureSPI(ETM_SPI_PORT_2, ETM_DEFAULT_SPI_CON_VALUE, ETM_DEFAULT_SPI_CON2_VALUE, ETM_DEFAULT_SPI_STAT_VALUE, SPI_CLK_5_MBIT, FCY);
  
  // Initialize the digital faults
  ETMDigitalInitializeInput(&global_data_A36487.pfn_fan_fault, ILL_PIN_PFN_FAULT, 50);
  ETMDigitalInitializeInput(&global_data_A36487.gun_fault_fiber, ILL_PIN_GUN_FAULT, 10);
  ETMDigitalInitializeInput(&global_data_A36487.rf_fault_fiber, ILL_PIN_GUN_FAULT, 10);
  ETMDigitalInitializeInput(&global_data_A36487.digital_input_key_lock, ILL_KEY_LOCK_FAULT, 50);
  ETMDigitalInitializeInput(&global_data_A36487.digital_input_panel_open, ILL_PANEL_OPEN, 50);

  
  global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_FOR_DATA_READ;


}


void DoA36487(void) {
  unsigned long temp32;

  ETMCanSlaveDoCan();  // DPARKER disable this after we have finished testing code

  if (TMR1 > MAX_TRIGGER_HIGH_TIME_TMR1_UNITS) {
    if (PIN_TRIGGER_IN == ILL_TRIGGER_ACTIVE) {
      __delay32(20);
      if (global_data_A36487.trigger_complete != 1) {
	if (PIN_TRIGGER_IN == ILL_TRIGGER_ACTIVE) {
	  __delay32(20);
	  if (PIN_TRIGGER_IN == ILL_TRIGGER_ACTIVE) {
	    // DPARKER - the trigger has stayed high.  Need to start incrementing some counter and fault if it gets too high
	    global_data_A36487.trigger_stayed_high_count++;
	  }
	}
      }
    }
  }
  
  if (global_data_A36487.trigger_complete) {
    DoPostTriggerProcess();
    global_data_A36487.trigger_complete = 0;
  }

  // ---------- UPDATE LOCAL FAULTS ------------------- //


  if ((global_data_A36487.control_state == STATE_FAULT) && ETMCanSlaveGetSyncMsgResetEnable()) {
    _FAULT_REGISTER = 0;
  }
  
  ETMDigitalUpdateInput(&global_data_A36487.pfn_fan_fault, PIN_CPU_PFN_OK_IN);
  if (ETMDigitalFilteredOutput(&global_data_A36487.pfn_fan_fault) == ILL_PIN_PFN_FAULT) {
    _FAULT_PFN_STATUS = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_PFN_STATUS = 0;
    }
  }

  ETMDigitalUpdateInput(&global_data_A36487.gun_fault_fiber, PIN_CPU_GUNDRIVER_OK_IN);
  if (ETMDigitalFilteredOutput(&global_data_A36487.gun_fault_fiber) == ILL_PIN_GUN_FAULT) {
    _FAULT_GUN_STATUS_FIBER = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_GUN_STATUS_FIBER = 0;
    }
  } 

  ETMDigitalUpdateInput(&global_data_A36487.rf_fault_fiber, PIN_CPU_RF_OK_IN);
  if (ETMDigitalFilteredOutput(&global_data_A36487.rf_fault_fiber) == ILL_PIN_RF_FAULT) {
    _FAULT_RF_STATUS = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_RF_STATUS = 0;
    }
  } 

  ETMDigitalUpdateInput(&global_data_A36487.digital_input_panel_open, PIN_PANEL_IN);
  if (ETMDigitalFilteredOutput(&global_data_A36487.digital_input_panel_open) == ILL_PANEL_OPEN) {
    _FAULT_PANEL_OPEN = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_PANEL_OPEN = 0;
    }
  } 

  ETMDigitalUpdateInput(&global_data_A36487.digital_input_key_lock, PIN_KEY_LOCK_IN);
  if (ETMDigitalFilteredOutput(&global_data_A36487.digital_input_key_lock) == ILL_KEY_LOCK_FAULT) {
    _FAULT_KEYLOCK_OPEN = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_KEYLOCK_OPEN = 0;
    }
  }   
  
  if (PIN_XRAY_CMD_MISMATCH_IN == !ILL_XRAY_CMD_MISMATCH) {
    _FAULT_TIMING_MISMATCH = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_TIMING_MISMATCH = 0;
    }
  }

  if (ETMCanSlaveGetComFaultStatus()) {
    _FAULT_CAN_COMMUNICATION_LATCHED = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 0;
    }
  }

  // ------------- UPDATE STATUS -------------------- //
  _STATUS_CUSTOMER_HV_DISABLE = !PIN_CUSTOMER_BEAM_ENABLE_IN;
  _STATUS_CUSTOMER_X_RAY_DISABLE = !PIN_CUSTOMER_XRAY_ON_IN;
  _STATUS_LOW_MODE_OVERRIDE = PIN_LOW_MODE_IN;
  _STATUS_HIGH_MODE_OVERRIDE = PIN_HIGH_MODE_IN;
  
  if (_T2IF) {
    // Run these once every 10ms
    _T2IF = 0;

    global_data_A36487.led_flash_counter++;



    // Reset all the trigger errors if HV is off
    
    if (global_data_A36487.trigger_stayed_high_count >= TRIGGER_STAYED_HIGH_FAULT_LEVEL) {
      _FAULT_TRIGGER = 1;
    }
    
    if (global_data_A36487.trigger_not_valid_count >= TRIGGER_NOT_VALID_FAULT_LEVEL) {
      //_FAULT_TRIGGER = 1;
    }

    if (global_data_A36487.trigger_period_too_short_count >= TRIGGER_PERIOD_TOO_SHORT_FAULT_LEVEL) {
       _FAULT_TRIGGER = 1;
    }

    if (global_data_A36487.trigger_length_too_short_count >= TRIGGER_LENGTH_TOO_SHORT_FAULT_LEVEL) {
      _FAULT_TRIGGER = 1;
    }

    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      global_data_A36487.trigger_stayed_high_count = 0;
      global_data_A36487.trigger_not_valid_count = 0;
      global_data_A36487.trigger_period_too_short_count = 0;
      global_data_A36487.trigger_length_too_short_count = 0;
      _FAULT_TRIGGER = 0;
    }

    
    // -------------- UPDATE LED AND STATUS LINE OUTPUTS ---------------- //
    if (ETMCanSlaveGetSyncMsgPulseSyncWarmupLED()) {
      PIN_CPU_WARMUP_OUT = OLL_CPU_WARMUP;
    } else {
      PIN_CPU_WARMUP_OUT = !OLL_CPU_WARMUP;
    }
    
    if (ETMCanSlaveGetSyncMsgPulseSyncStandbyLED()) {
      PIN_CPU_STANDBY_OUT = OLL_CPU_STANDBY;
    } else {
      PIN_CPU_STANDBY_OUT = !OLL_CPU_STANDBY;
    }
    
    if (ETMCanSlaveGetSyncMsgPulseSyncReadyLED()) {
      PIN_LED_READY = OLL_LED_ON;
      PIN_CPU_READY_OUT = OLL_CPU_READY;
    } else {
      PIN_LED_READY = !OLL_LED_ON;
      PIN_CPU_READY_OUT = !OLL_CPU_READY;
    }
  
    if (ETMCanSlaveGetSyncMsgPulseSyncFaultLED()) {
      PIN_CPU_SUMFLT_OUT = OLL_CPU_SUMFLT;
    } else {
      PIN_CPU_SUMFLT_OUT = !OLL_CPU_SUMFLT;
    }
    
    // Calculate the rep rate
    temp32 = 1562500;
    temp32 /= global_data_A36487.period_filtered;
    log_data_rep_rate_deci_hertz = temp32;
    if (_T1IF || (log_data_rep_rate_deci_hertz < 25)) {
      // We are pulseing at less than 2.5Hz
      // Set the rep rate to zero
      log_data_rep_rate_deci_hertz = 0;
    }
    
    // Update the debugging Data
    ETMCanSlaveSetDebugRegister(0x0, global_data_A36487.control_state);
    ETMCanSlaveSetDebugRegister(0x1, trigger_set_point_active_decihertz);

    ETMCanSlaveSetDebugRegister(0x2, global_data_A36487.trigger_stayed_high_count);
    ETMCanSlaveSetDebugRegister(0x3, global_data_A36487.trigger_not_valid_count);
    ETMCanSlaveSetDebugRegister(0x4, global_data_A36487.trigger_period_too_short_count);
    ETMCanSlaveSetDebugRegister(0x5, global_data_A36487.trigger_length_too_short_count);
    ETMCanSlaveSetDebugRegister(0x6, global_data_A36487.trigger_decrement_counter);
    
    ETMCanSlaveSetDebugRegister(0x7, log_data_rep_rate_deci_hertz);
    
    ETMCanSlaveSetDebugRegister(0x8, global_data_A36487.total_missed_messages);
    ETMCanSlaveSetDebugRegister(0x9, global_data_A36487.previous_message_ok);
    ETMCanSlaveSetDebugRegister(0xA, global_data_A36487.message_received_count);
  }
}



void DoStartupLEDs(void) {
  switch ((global_data_A36487.led_flash_counter >> 4) & 0b11)
    {
    case 0:
      PIN_LED_READY   = !OLL_LED_ON;
      PIN_LED_XRAY_ON = !OLL_LED_ON;
      //PIN_LED_SUMFLT  = !OLL_LED_ON;
      break;

    case 1:
      PIN_LED_READY   = OLL_LED_ON;
      PIN_LED_XRAY_ON = !OLL_LED_ON;
      //PIN_LED_SUMFLT  = !OLL_LED_ON;
      break;

    case 2:
      PIN_LED_READY   = !OLL_LED_ON;
      PIN_LED_XRAY_ON = OLL_LED_ON;
      //PIN_LED_SUMFLT  = !OLL_LED_ON;
      break;

    case 3:
      PIN_LED_READY   = !OLL_LED_ON;
      PIN_LED_XRAY_ON = !OLL_LED_ON;
      //PIN_LED_SUMFLT  = OLL_LED_ON;
      break;
    }
}


void DoPostTriggerProcess(void) {
  unsigned long temp32;
  unsigned int temp16;
  
  ETMCanSlavePulseSyncSendNextPulseLevel(GetThisPulseLevel(), global_data_A36487.pulses_on, log_data_rep_rate_deci_hertz);
  
  ProgramShiftRegistersDelays();  // Load the shift registers

  global_data_A36487.pulses_on++; // This counts the pulses

  ProgramShiftRegistersGrid(0xFF);

  // Update the PRF
  global_data_A36487.period_filtered = RCFilterNTau(global_data_A36487.period_filtered, global_data_A36487.last_period, RC_FILTER_4_TAU);
  
  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    // Log Pulse by Pulse data

    temp32 = 1562500;
    temp32 /= global_data_A36487.last_period;
    temp16 = temp32;
    ETMCanSlaveSetDebugRegister(0xB, global_data_A36487.last_period);
    ETMCanSlaveSetDebugRegister(0xC, temp16);

    ETMCanSlaveLogPulseData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_LOG_0,
			    (global_data_A36487.pulses_on - 1),
			    *(unsigned int*)&trigger_width,
			    *(unsigned int*)&data_grid_start,
			    temp16);
  }

  global_data_A36487.trigger_decrement_counter++;
  if (global_data_A36487.trigger_decrement_counter >= TRIGGER_COUNTER_DECREMENT_INTERVAL) {
    global_data_A36487.trigger_decrement_counter = 0;
    
    if (global_data_A36487.trigger_stayed_high_count) {
      global_data_A36487.trigger_stayed_high_count--;
    }

    if (global_data_A36487.trigger_not_valid_count) {
      global_data_A36487.trigger_not_valid_count--;
    }
    
    if (global_data_A36487.trigger_period_too_short_count) {
      global_data_A36487.trigger_period_too_short_count--;
    }
    
    if (global_data_A36487.trigger_length_too_short_count) {
      global_data_A36487.trigger_length_too_short_count--;
    }
  }
}


void ProgramShiftRegistersDelays(void) {
  unsigned int data;
  unsigned char hv_trigger_delay;
  unsigned char pfn_trigger_delay;
  unsigned char magnetron_sample_delay;
  unsigned char afc_sample_delay;
  

  switch (GetThisPulseLevel())
    {
    case PULSE_LEVEL_CARGO_HIGH:
      hv_trigger_delay = dose_sample_delay_high;
      pfn_trigger_delay = pfn_delay_high;
      magnetron_sample_delay = magnetron_current_sample_delay_high;
      afc_sample_delay = afc_delay_high;
      break;
      
    case PULSE_LEVEL_CARGO_LOW:
      hv_trigger_delay = dose_sample_delay_low;
      pfn_trigger_delay = pfn_delay_low;
      magnetron_sample_delay = magnetron_current_sample_delay_low;
      afc_sample_delay = afc_delay_low;
      break;


    default:
      break;
    }
  
  // Send out the PFN & HV trigger delays
  data   = hv_trigger_delay;
  data <<= 8;
  data  += pfn_trigger_delay;
  SendAndReceiveSPI(data, ETM_SPI_PORT_2);
  PIN_LD_DELAY_PFN_OUT = 0;
  Nop();
  PIN_LD_DELAY_PFN_OUT = 1;
  Nop();

  // Send out the Magnetron and AFC Sample delays
  data   = magnetron_sample_delay;
  data <<= 8;
  data  += afc_sample_delay;
  SendAndReceiveSPI(data, ETM_SPI_PORT_2);
  PIN_LD_DELAY_AFC_OUT = 0;
  Nop();
  PIN_LD_DELAY_AFC_OUT = 1;
  Nop();
}

#define CAB_SCAN_GRID_START_HIGH   130
#define CAB_SCAN_GRID_STOP_HIGH    135

#define CAB_SCAN_GRID_START_LOW    130
#define CAB_SCAN_GRID_STOP_LOW     135

void ProgramShiftRegistersGrid(unsigned char dose_command) {
  unsigned int data;


  switch (GetThisPulseLevel())
    {
    case PULSE_LEVEL_CARGO_HIGH:
      data_grid_stop  = InterpolateValue(grid_stop_high0, grid_stop_high1, grid_stop_high2, grid_stop_high3, dose_command);
      data_grid_start = InterpolateValue(grid_start_high0, grid_start_high1, grid_start_high2, grid_start_high3, dose_command);
      break;
      
    case PULSE_LEVEL_CARGO_LOW:
      data_grid_stop  = InterpolateValue(grid_stop_low0, grid_stop_low1, grid_stop_low2, grid_stop_low3, dose_command);
      data_grid_start = InterpolateValue(grid_start_low0, grid_start_low1, grid_start_low2, grid_start_low3, dose_command);
      break;

    default:
      break;
    }

  // Send out Grid start and stop delays
  data   = data_grid_stop;
  data <<= 8;
  data  += data_grid_start;
  SendAndReceiveSPI(data, ETM_SPI_PORT_2);
  PIN_LD_DELAY_GUN_OUT = 0;
  Nop();
  PIN_LD_DELAY_GUN_OUT = 1;
  Nop();
}


unsigned int GetThisPulseLevel(void) {
  // DPARKER THIS NEEDS MORE WORK I THINK FOR ALL OF THE NON-PLC Based systems
  
  if (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED) {
    return PULSE_LEVEL_CARGO_HIGH;
  }
  
  return PULSE_LEVEL_CARGO_LOW;
}


unsigned char InterpolateValue(unsigned int val_0, unsigned int val_1, unsigned int val_2, unsigned int val_3, unsigned char index) {
  unsigned int result;
  unsigned char carry;
  
  val_0 <<= 1;
  val_1 <<= 1;
  val_2 <<= 1;
  val_3 <<= 1;
  
  if (index < 85) {
    result   = val_0 * (85 - index);
    result  += val_1 * (index);
    result  /= 85;
  } else if (index < 170) {
    result   = val_1 * (170 - index);
    result  += val_2 * (index - 85);
    result  /= 85;
  } else {
    result   = val_2 * (255 - index);
    result  += val_3 * (index - 170);
    result  /= 85;
  }
  
  carry = result & 0x0001;
  result >>= 1;
  result  += carry;
  
  return result;
}



void SetupT3PRFDeciHertz(unsigned int decihertz) {
#ifdef __INTERNAL_TRIGGER
  unsigned long period_long;
  unsigned int period_int;
  
  // decihertz is the requested PRF in decihertz
  if (decihertz <= MIN_REQUESTED_PRF_DECIHERTZ) {
    decihertz = MIN_REQUESTED_PRF_DECIHERTZ;
  }
  
  if (decihertz >= MAX_REQUESTED_PRF_DECIHERTZ) {
    decihertz = MAX_REQUESTED_PRF_DECIHERTZ;
  }
  
  period_long = 100000000;
  period_long /= decihertz;

  if (decihertz <= 20) {
    T3CON = T3CON_VALUE_PS_256;
    period_long = period_long >> 8;
  } else if (decihertz <= 190) {
    T3CON = T3CON_VALUE_PS_64;
    period_long = period_long >> 6;
  } else if (decihertz <= 1520) {
    T3CON = T3CON_VALUE_PS_8;
    period_long = period_long >> 3;
  } else {
    T3CON = T3CON_VALUE_PS_1;
  }
  
  if (period_long >= 0x0000FFFF) {
    period_long = 0x0000FFFF;
  }
  
  period_int = period_long;
  PR3 = period_int;

#endif
}






// Executes the CAN Commands
void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {

  unsigned int index_word;
  //unsigned int temp;
  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
      /*            
    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_0:
      *(unsigned int*)&grid_start_high3 = message_ptr->word2;
      *(unsigned int*)&grid_start_high1 = message_ptr->word1;
      *(unsigned int*)&dose_sample_delay_high = message_ptr->word0;
      global_data_A36487.counter_config_received |= 0b0001;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_1:
      *(unsigned int*)&grid_stop_high3 = message_ptr->word2;
      *(unsigned int*)&grid_stop_high1 = message_ptr->word1;
      *(unsigned int*)&magnetron_current_sample_delay_high = message_ptr->word0;
      global_data_A36487.counter_config_received |=0b0010;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_0:
      *(unsigned int*)&grid_start_low3 = message_ptr->word2;
      *(unsigned int*)&grid_start_low1 = message_ptr->word1;
      *(unsigned int*)&dose_sample_delay_low = message_ptr->word0;
      global_data_A36487.counter_config_received |= 0b0100;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_1:
      *(unsigned int*)&grid_stop_low3 = message_ptr->word2;
      *(unsigned int*)&grid_stop_low1 = message_ptr->word1;
      *(unsigned int*)&magnetron_current_sample_delay_low = message_ptr->word0;
      global_data_A36487.counter_config_received |= 0b1000;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_CUSTOMER_LED_OUTPUT:
      //global_data_A36487.led_state = message_ptr->word0;
      trigger_set_high_energy_decihertz = message_ptr->word2;
      trigger_set_low_energy_decihertz = message_ptr->word1;
      break;
      
      */

      
    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_0:
      grid_stop_high3  = (message_ptr->word1 & 0x00FF);
      grid_stop_high2  = (message_ptr->word1 & 0x00FF);
      grid_stop_high1  = (message_ptr->word1 & 0x00FF);
      grid_stop_high0  = (message_ptr->word1 & 0x00FF);

      grid_start_high3 = (message_ptr->word0 & 0x00FF);
      grid_start_high2 = (message_ptr->word0 & 0x00FF);
      grid_start_high1 = (message_ptr->word0 & 0x00FF);
      grid_start_high0 = (message_ptr->word0 & 0x00FF);

      afc_delay_high   = (message_ptr->word2 & 0x00FF);

      global_data_A36487.counter_config_received |= 0b0001;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_1:
      grid_stop_low3  = (message_ptr->word1 & 0x00FF);
      grid_stop_low2  = (message_ptr->word1 & 0x00FF);
      grid_stop_low1  = (message_ptr->word1 & 0x00FF);
      grid_stop_low0  = (message_ptr->word1 & 0x00FF);

      grid_start_low3 = (message_ptr->word0 & 0x00FF);
      grid_start_low2 = (message_ptr->word0 & 0x00FF);
      grid_start_low1 = (message_ptr->word0 & 0x00FF);
      grid_start_low0 = (message_ptr->word0 & 0x00FF);

      afc_delay_low   = (message_ptr->word2 & 0x00FF);

      global_data_A36487.counter_config_received |= 0b0010;
      break;
      
    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_MAGNETX_DOSE_ALL:
      dose_sample_delay_high              = (message_ptr->word2 & 0x00FF);
      pfn_delay_high                      = (message_ptr->word1 & 0x00FF);
      magnetron_current_sample_delay_high = (message_ptr->word0 & 0x00FF);

      dose_sample_delay_low               = (message_ptr->word2 & 0x00FF);
      pfn_delay_low                       = (message_ptr->word1 & 0x00FF);
      magnetron_current_sample_delay_low  = (message_ptr->word0 & 0x00FF);
      
      global_data_A36487.counter_config_received |= 0b1100;
      break;

    default:
      ETMCanSlaveIncrementInvalidIndex();
      break;
    }
}



// -------------------------- INTERRUPTS ------------------------ //


// Trigger is generated internally
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
#ifdef __INTERNAL_TRIGGER
  if ((global_data_A36487.control_state == STATE_X_RAY_ENABLE)) {
    PIN_CPU_START_OUT = OLL_CPU_START;
    __delay32(300);  // Delay 30 uS
  }
  
  PIN_CPU_START_OUT = !OLL_CPU_START;
  
  global_data_A36487.last_period = TMR1;
  TMR1 = 0;
  if (_T1IF) {
    // The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
    global_data_A36487.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
  }
  _T1IF = 0;
  
  global_data_A36487.trigger_complete = 1;
  trigger_width = 0xFF;
  trigger_width_filtered = 0xFF;
  global_data_A36487.this_pulse_width = 0xFF;

  UpdateEnergyAndPolarityOutputs();

#endif
  _T3IF = 0;
}



// External Trigger
void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {

  unsigned int period_min;
  unsigned int period_max;
  
  // A trigger was recieved
  
  // First check that the pulse is valid
  // It takes at least 4 clock cycles to get here
  // IF the trigger is valid and we are in the x-ray state, send out the start pulse
  if (PIN_TRIGGER_IN == ILL_TRIGGER_ACTIVE) {
    // The Trigger Pulse is Valid
    if (_T3IF) {
      // The minimum period between pulses has passed
      if ((global_data_A36487.control_state == STATE_X_RAY_ENABLE)) {
	PIN_CPU_START_OUT = OLL_CPU_START;
      }
      // Start The Holdoff Timer for the next pulse
      TMR3 = 0;
      _T3IF = 0;

      // Calculate the Trigger PRF
      // TMR1 is used to time the time between INT1 interrupts
      global_data_A36487.last_period = TMR1;

      TMR1 = 0;
      if (_T1IF) {
	// The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
	global_data_A36487.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
      }
      _T1IF = 0;

      period_max = global_data_A36487.prf_from_concentrator;
      if (period_max > 2) {
	period_max -= 2;
      } else {
	period_max = 0;
      }
      //period_max = prf_to_period_converstion_table[period_max];
      // DPARKER add this table
      
      
      period_min = global_data_A36487.prf_from_concentrator;
      period_min += 2;
      if (period_min > 0xFF) {
	period_min = 0xFF;
      }
      //period_min = prf_to_period_converstion_table[period_min];
      
      
      if ((global_data_A36487.last_period > period_max) || (global_data_A36487.last_period < period_min)) {
	
      }

      
      global_data_A36487.trigger_complete = 1;

      // DPARKER - REDUCE THIS DELAY TO ACCOUNT FOR THE TIME IT TAKES TO GET HERE
      __delay32(300);  // Delay 30 uS
      
      //UpdateEnergyAndPolarityOutputs();

      if (PIN_TRIGGER_IN == !ILL_TRIGGER_ACTIVE) {
	// The trigger did not stay high for long enough)
	global_data_A36487.trigger_length_too_short_count++;
      }
    } else {
      // Triggers were too close together
      global_data_A36487.trigger_period_too_short_count++;
    }
  } else {
    // Trigger pulse was too short to trigger a pulse.
    global_data_A36487.trigger_not_valid_count++;
  }
  PIN_CPU_START_OUT = !OLL_CPU_START;
  _INT1IF = 0;
}

  

void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _U2RXInterrupt(void) {
  _U2RXIF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    __asm__ ("Reset");
}



