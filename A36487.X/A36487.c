#include "A36487.h"
#include "A36487_CONFIG.h"

/*
  DPARKER need to figure out and extend the ETM Can functions for high, low, cab scan mode so that all modules are more generic
*/

#define DOSE_COMMAND_LOW_ENERGY   0  
#define DOSE_COMMAND_HIGH_ENERGY  1


#define PULSE_LEVEL_CARGO_HIGH    2
#define PULSE_LEVEL_CARGO_LOW     3
#define PULSE_LEVEL_CAB_HIGH      4
#define PULSE_LEVEL_CAB_LOW       5
#define PULSE_LEVEL_OFF           6




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
    global_data_A36487.control_state = STATE_WAIT_FOR_CONFIG;
    break;

  case STATE_WAIT_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
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
    while (global_data_A36487.control_state == STATE_HV_ENABLE) {
      DoA36487();

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
    PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    while (global_data_A36487.control_state == STATE_X_RAY_ENABLE) {
      DoA36487();

      if (PIN_CUSTOMER_XRAY_ON_IN == ILL_CUSTOMER_XRAY_ON) {
	PIN_CPU_WARNING_LAMP_OUT = OLL_CPU_WARNING_LAMP;
      } else {
	PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
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


  PIN_GUN_POLARITY_OUT  = !OLL_POLARITY_NORMAL;
  PIN_HVPS_POLARITY_OUT = OLL_POLARITY_NORMAL;
  PIN_RF_POLARITY_OUT   = OLL_POLARITY_NORMAL;


  PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;

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
  
#ifdef __PLC_INTERFACE
  // Enable ADC Convert
  ADCON1 = ADCON1_SETTING_EN;  
  ADCON2 = ADCON2_SETTING_EN;
  ADCON3 = ADCON3_SETTING_EN;
  ADPCFG = ADPCFG_SETTING_EN;
  ADCSSL = ADCSSL_SETTING_EN;
  ADCHS  = ADCHS_SETTING_EN;
  _ADIF = 0;
  _ADIE = 1;
  _ADIP = 3;
  _ADON = 1;
#endif


#ifdef __INTERNAL_TRIGGER
  // Set up T3 Interrupt
  _T3IE = 1;
  _T3IP = 7;
  T3CON = T3CON_VALUE;
  
  trigger_set_point_active_decihertz = 10;
  trigger_set_high_energy_decihertz = 10;
  trigger_set_low_energy_decihertz = 10;
  SetupT3PRFDeciHertz(trigger_set_point_active_decihertz);
#endif


#ifndef __INTERNAL_TRIGGER
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
  _U2RXIE = 1;  
#endif


  // Read the personality module
  global_data_A36487.personality = ReadDosePersonality();
  
#ifndef __PLC_INTERFACE
  global_data_A36487.personality = 0;
#endif

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

  


#ifdef __PLC_INTERFACE
  unsigned int personality_sent_results;
  unsigned int personality_send_attempts;
  
  personality_sent_results = 1;
  personality_send_attempts = 0;
  while ((personality_sent_results) && (personality_send_attempts < 30)) {
    personality_sent_results = SendPersonalityToPLC(global_data_A36487.personality);
    personality_send_attempts++;
    ClrWdt();
  }
#endif 
  
  global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_FOR_DATA_READ;


}



unsigned char ReadDosePersonality() {
  unsigned int data;
  unsigned int i;
  

  PIN_ID_CLK_OUT   = 0;  
  PIN_ID_SHIFT_OUT = 0;  
  __delay32(10);
  PIN_ID_SHIFT_OUT = 1;  
  __delay32(10);

  data = 0;

  if (PIN_ID_DATA_IN) {
    data |= 0x01;
  }
  
  for (i = 0; i < 8; i++) {
    PIN_ID_CLK_OUT = 0;
    data <<= 1;
    PIN_ID_CLK_OUT = 1;
    __delay32(5);
    if (PIN_ID_DATA_IN) {
      data |= 0x01;
    } 
  }

  if (data == ULTRA_LOW_DOSE) {
    return 3;
  }

  if (data == LOW_DOSE) {
    return 2;
  }
  
  if (data == MEDIUM_DOSE) {
    return 1;
  }
  
  if (data == HIGH_DOSE) {
    return 0;
  }
 
  return 0xFF;
}


void DoA36487(void) {
  unsigned long temp32;

  ETMCanSlaveDoCan();  // DPARKER disable this after we have finished testing code

#ifndef __INTERNAL_TRIGGER
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
#endif
  
  
#ifdef __PLC_INTERFACE
  ClrWdt();  // The watchdog is normally cleared by the Can Module, but without that we need to manually clear it
#endif

  if (global_data_A36487.trigger_complete) {
    DoPostTriggerProcess();
    global_data_A36487.trigger_complete = 0;
  }

  if ((global_data_A36487.trigger_width_update_ready) && (global_data_A36487.trigger_complete == 0)) {
    ProgramShiftRegistersGrid(global_data_A36487.this_pulse_width);
    global_data_A36487.trigger_width_update_ready = 0;
    PIN_40US_TEST_POINT = 0;
  }
  
  // ---------- UPDATE LOCAL FAULTS ------------------- //


#ifdef __PLC_INTERFACE
  if ((global_data_A36487.control_state == STATE_FAULT) && (_MACRO_NOT_HV_ENABLE)) {
    _FAULT_REGISTER = 0;
  }
#else
  if ((global_data_A36487.control_state == STATE_FAULT) && ETMCanSlaveGetSyncMsgResetEnable()) {
    _FAULT_REGISTER = 0;
  }
#endif
  
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



    // Look for trigger errors.
#ifndef  __INTERNAL_TRIGGER

    // Reset all the trigger errors if HV is off
    if (_MACRO_NOT_HV_ENABLE) {
      global_data_A36487.trigger_stayed_high_count = 0;
      global_data_A36487.trigger_not_valid_count = 0;
      global_data_A36487.trigger_period_too_short_count = 0;
      global_data_A36487.trigger_length_too_short_count = 0;

      _FAULT_TRIGGER = 0;
    }
    
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

#endif

    if (PIN_PACKAGE_ID_1 == ILL_RETURN_TRIGGER_FAULT) {
      ETMCanSlaveSetDebugRegister(0xB, 1);
      if (_FAULT_TRIGGER == 1) {
	PIN_PORTAL_GANTRY_MODE_OUT = OLL_TRIGGER_FAULT;  // Trigger Fault Active
	ETMCanSlaveSetDebugRegister(0xC, 0);
      } else {
	PIN_PORTAL_GANTRY_MODE_OUT = !OLL_TRIGGER_FAULT; // Trigger fault not active
	ETMCanSlaveSetDebugRegister(0xC, 1);
      }
    } else {
      ETMCanSlaveSetDebugRegister(0xB, 0);
      if (global_data_A36487.this_pulse_width >= 191) {
	PIN_PORTAL_GANTRY_MODE_OUT = OLL_GANTRY_MODE;
	ETMCanSlaveSetDebugRegister(0xC, 100);
      } else {
	PIN_PORTAL_GANTRY_MODE_OUT = !OLL_GANTRY_MODE;
	ETMCanSlaveSetDebugRegister(0xC, 101);
      }
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
    
#ifdef __PLC_INTERFACE
    UpdateAnalogDataFromPLC();
#endif
    
    // Update the debugging Data
    ETMCanSlaveSetDebugRegister(0x0, global_data_A36487.control_state);
    ETMCanSlaveSetDebugRegister(0x1, trigger_set_point_active_decihertz);

    ETMCanSlaveSetDebugRegister(0x2, global_data_A36487.trigger_stayed_high_count);
    ETMCanSlaveSetDebugRegister(0x3, global_data_A36487.trigger_not_valid_count);
    ETMCanSlaveSetDebugRegister(0x4, global_data_A36487.trigger_period_too_short_count);
    ETMCanSlaveSetDebugRegister(0x5, global_data_A36487.trigger_length_too_short_count);
    ETMCanSlaveSetDebugRegister(0x6, global_data_A36487.trigger_decrement_counter);
    

    
    ETMCanSlaveSetDebugRegister(0x7, grid_stop_high3);
    ETMCanSlaveSetDebugRegister(0x8, grid_start_high3);
    ETMCanSlaveSetDebugRegister(0x9, grid_stop_low3);
    ETMCanSlaveSetDebugRegister(0xA, grid_start_low3);

  }
}


void UpdateAnalogDataFromPLC(void) {
  global_data_A36487.analog_interface_timer++;
  
  if ((global_data_A36487.state_analog_data_read != ANALOG_STATE_WAIT_FOR_DATA_READ) &&
      (global_data_A36487.state_analog_data_read != ANALOG_STATE_WAIT_FOR_PLC_SET) &&
      (global_data_A36487.state_analog_data_read != ANALOG_STATE_WAIT_READ_COMPLETE)) {
    global_data_A36487.state_analog_data_read = ANALOG_STATE_DATA_READ_FAULT;
  }


  if ((PIN_PACKAGE_VALID_IN == ILL_PACKAGE_VALID) && (global_data_A36487.state_analog_data_read == ANALOG_STATE_WAIT_FOR_DATA_READ)) {
    // INITIATE Data Read
    global_data_A36487.analog_interface_attempts++;


    PIN_ANALOG_READ_COMPLETE_OUT = !OLL_ANALOG_READ_COMPLETE;
    global_data_A36487.analog_register_select = 0;
    global_data_A36487.analog_interface_timer = 0;
    global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_FOR_PLC_SET;
  }
  
  if ((global_data_A36487.state_analog_data_read == ANALOG_STATE_WAIT_FOR_PLC_SET) && (global_data_A36487.analog_interface_timer >= 25)) {
    if (PIN_PACKAGE_VALID_IN == !ILL_PACKAGE_VALID) {
      global_data_A36487.state_analog_data_read = ANALOG_STATE_DATA_READ_FAULT;
    } else if (global_data_A36487.analog_register_select >= 3) {
      global_data_A36487.state_analog_data_read = ANALOG_STATE_DATA_READ_FAULT;
    } else {
      ReadAnalogRegister(); 	// read the analog data
      PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;
      global_data_A36487.analog_register_select++;
      global_data_A36487.analog_interface_timer = 0;
      global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_READ_COMPLETE;
    }
  }
  
  if ((global_data_A36487.state_analog_data_read == ANALOG_STATE_WAIT_READ_COMPLETE) && (global_data_A36487.analog_interface_timer >= 25)) {
    if (PIN_PACKAGE_VALID_IN == ILL_PACKAGE_VALID) {
      global_data_A36487.state_analog_data_read = ANALOG_STATE_DATA_READ_FAULT;
    } else {
      if (global_data_A36487.analog_register_select >= 3) {
	global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_FOR_DATA_READ;
	global_data_A36487.counter_config_received = 0b1111;
	PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;
      } else {
	PIN_ANALOG_READ_COMPLETE_OUT = !OLL_ANALOG_READ_COMPLETE;
	global_data_A36487.analog_interface_timer = 0;
	global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_FOR_PLC_SET;
      }

    }
  }
  
  if (global_data_A36487.state_analog_data_read == ANALOG_STATE_DATA_READ_FAULT) {
    global_data_A36487.analog_register_select = 0;
    global_data_A36487.analog_interface_timer = 0;
    global_data_A36487.state_analog_data_read = ANALOG_STATE_WAIT_FOR_DATA_READ;
    PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;
  }
}





void ReadAnalogRegister(void) {
  /*
    Order of analog values
    0 = Nothing, Can Not Implement
    1 = High Energy Grid Start 3
    2 = High Energy Grid Start 2
    3 = High Energy Grid Start 1
    4 = High Energy Grid Start 0
    5 = Low Energy Grid Start 3
    6 = Low Energy Grid Start 2
    7 = Low Energy Grid Start 1
    8 = Low Energy Grid Start 0
    9 = High Energy Grid Stop 3
    10 = High Energy Grid Stop 2
    11 = High Energy Grid Stop 1
    12 = High Energy Grid Stop 0
    13 = Low Energy Grid Stop 3
    14 = Low Energy Grid Stop 2
    15 = Low Energy Grid Stop 1
    16 = Low Energy Grid Stop 0
    17 = High Energy PFN Delay
    18 = High Energy RF Delay
    19 = Low Energy PFN Delay
    20 = Low Energy RF Delay
    19 = High Energy AFC Delay
    20 = Not Implemented
    21 = Low Energy AFC Delay
    22 = Not Implemented
  */

  switch (global_data_A36487.analog_register_select) {
  case 0:
    grid_start_high3 = (global_data_A36487.analog_1.filtered_adc_reading >> 8);
    grid_start_high2 = (global_data_A36487.analog_2.filtered_adc_reading >> 8);
    grid_start_high1 = (global_data_A36487.analog_3.filtered_adc_reading >> 8);
    grid_start_high0 = (global_data_A36487.analog_4.filtered_adc_reading >> 8);
    grid_start_low3  = (global_data_A36487.analog_5.filtered_adc_reading >> 8);
    grid_start_low2  = (global_data_A36487.analog_6.filtered_adc_reading >> 8);
    grid_start_low1  = (global_data_A36487.analog_7.filtered_adc_reading >> 8);
    grid_start_low0  = (global_data_A36487.analog_8.filtered_adc_reading >> 8);
    break;

  case 1:
    grid_stop_high3  = (global_data_A36487.analog_1.filtered_adc_reading >> 8);
    grid_stop_high2  = (global_data_A36487.analog_2.filtered_adc_reading >> 8);
    grid_stop_high1  = (global_data_A36487.analog_3.filtered_adc_reading >> 8);
    grid_stop_high0  = (global_data_A36487.analog_4.filtered_adc_reading >> 8);
    grid_stop_low3   = (global_data_A36487.analog_5.filtered_adc_reading >> 8);
    grid_stop_low2   = (global_data_A36487.analog_6.filtered_adc_reading >> 8);
    grid_stop_low1   = (global_data_A36487.analog_7.filtered_adc_reading >> 8);
    grid_stop_low0   = (global_data_A36487.analog_8.filtered_adc_reading >> 8);
    break;

  case 2:
    pfn_delay_high          = (global_data_A36487.analog_1.filtered_adc_reading >> 8);
    dose_sample_delay_high  = (global_data_A36487.analog_2.filtered_adc_reading >> 8);
    pfn_delay_low           = (global_data_A36487.analog_3.filtered_adc_reading >> 8);
    dose_sample_delay_low   = (global_data_A36487.analog_4.filtered_adc_reading >> 8);
    afc_delay_high          = (global_data_A36487.analog_5.filtered_adc_reading >> 8);
    magnetron_current_sample_delay_high  = (global_data_A36487.analog_6.filtered_adc_reading >> 8);
    afc_delay_low           = (global_data_A36487.analog_7.filtered_adc_reading >> 8);
    magnetron_current_sample_delay_low  = (global_data_A36487.analog_8.filtered_adc_reading >> 8);
    break;

  case 3:
    // Unused at this time
    break;

  default:
    // Error : do not update anything
    break;
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
  ETMCanSlavePulseSyncSendNextPulseLevel(GetThisPulseLevel(), global_data_A36487.pulses_on, log_data_rep_rate_deci_hertz);
  
  ProgramShiftRegistersDelays();  // Load the shift registers

  global_data_A36487.pulses_on++; // This counts the pulses

  ProgramShiftRegistersGrid(global_data_A36487.this_pulse_width);

  // Update the PRF
  global_data_A36487.period_filtered = RCFilterNTau(global_data_A36487.period_filtered, global_data_A36487.last_period, RC_FILTER_4_TAU);
  
  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    // Log Pulse by Pulse data
    ETMCanSlaveLogPulseData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_LOG_0,
			    (global_data_A36487.pulses_on - 1),
			    *(unsigned int*)&trigger_width,
			    *(unsigned int*)&data_grid_start,
			    0);
  }

#ifndef __INTERNAL_TRIGGER
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
#endif

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
      
    case PULSE_LEVEL_CAB_HIGH:
      hv_trigger_delay = dose_sample_delay_high;
      pfn_trigger_delay = pfn_delay_high;
      magnetron_sample_delay = magnetron_current_sample_delay_high;
      afc_sample_delay = afc_delay_high;
      break;
      
    case PULSE_LEVEL_CAB_LOW:
      hv_trigger_delay = dose_sample_delay_low;
      pfn_trigger_delay = pfn_delay_low;
      magnetron_sample_delay = magnetron_current_sample_delay_low;
      afc_sample_delay = afc_delay_low;
      break;

    case PULSE_LEVEL_OFF:
      // DPARKER - Disable Pulsing
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
      
    case PULSE_LEVEL_CAB_HIGH:
      data_grid_start = CAB_SCAN_GRID_START_HIGH;
      data_grid_stop  = CAB_SCAN_GRID_STOP_HIGH;
      break;
      
    case PULSE_LEVEL_CAB_LOW:
      data_grid_start = CAB_SCAN_GRID_START_LOW;
      data_grid_stop  = CAB_SCAN_GRID_STOP_LOW;
      break;

    case PULSE_LEVEL_OFF:
      data_grid_start = 0;
      data_grid_stop  = 0;
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
#ifdef __PLC_INTERFACE
  // CARGO SCAN Mode  
  if ((PIN_LOW_MODE_IN == ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED)) {
    if (global_data_A36487.this_pulse_level == DOSE_COMMAND_HIGH_ENERGY) {
      return PULSE_LEVEL_CARGO_HIGH;
    } else {
      return PULSE_LEVEL_CARGO_LOW;
    }
  }

  // CAB SCAN Mode
  if ((PIN_LOW_MODE_IN == ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == !ILL_MODE_BIT_SELECTED)) {
    if (global_data_A36487.this_pulse_level == DOSE_COMMAND_HIGH_ENERGY) {
      return PULSE_LEVEL_CAB_HIGH;
    } else {
      return PULSE_LEVEL_CAB_LOW;
    }
  }


  // HIGH MODE ONLY  
  if ((PIN_LOW_MODE_IN == !ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED)) {
    return PULSE_LEVEL_CARGO_HIGH;
  }
  

  // X Ray Off
  return PULSE_LEVEL_OFF;

#else
  
  // DPARKER THIS NEEDS MORE WORK I THINK FOR ALL OF THE NON-PLC Based systems
  
  if (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED) {
    return PULSE_LEVEL_CARGO_HIGH;
  }
  
  return PULSE_LEVEL_CARGO_LOW;
#endif
  
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



void UpdateEnergyAndPolarityOutputs(void) {
  switch (GetThisPulseLevel())
    {
    case PULSE_LEVEL_CARGO_HIGH:
      PIN_GUN_CAB_SCAN_FIBER_OUT = !OLL_GUN_CAB_SCAN_SELECTED;
      PIN_ENERGY_CPU_OUT = OLL_ENERGY_LEVEL_HIGH;
      PIN_HVPS_POLARITY_OUT = OLL_POLARITY_NORMAL;
      PIN_AFC_TRIGGER_ENABLE_OUT = OLL_AFC_TRIGGER_ENABLE;
      break;
      
    case PULSE_LEVEL_CARGO_LOW:
      PIN_GUN_CAB_SCAN_FIBER_OUT = !OLL_GUN_CAB_SCAN_SELECTED;
      PIN_ENERGY_CPU_OUT = !OLL_ENERGY_LEVEL_HIGH;      
      PIN_HVPS_POLARITY_OUT = OLL_POLARITY_NORMAL;
      PIN_AFC_TRIGGER_ENABLE_OUT = OLL_AFC_TRIGGER_ENABLE;
      if (global_data_A36487.this_pulse_level != global_data_A36487.next_pulse_level) {
	PIN_AFC_TRIGGER_ENABLE_OUT = !OLL_AFC_TRIGGER_ENABLE;
      }
      break;
      
    case PULSE_LEVEL_CAB_HIGH:
      PIN_GUN_CAB_SCAN_FIBER_OUT = OLL_GUN_CAB_SCAN_SELECTED;
      PIN_ENERGY_CPU_OUT = OLL_ENERGY_LEVEL_HIGH;
      PIN_HVPS_POLARITY_OUT = OLL_POLARITY_NORMAL;
      PIN_AFC_TRIGGER_ENABLE_OUT = OLL_AFC_TRIGGER_ENABLE;
      break;
      
    case PULSE_LEVEL_CAB_LOW:
      PIN_GUN_CAB_SCAN_FIBER_OUT = OLL_GUN_CAB_SCAN_SELECTED;
      PIN_ENERGY_CPU_OUT = OLL_ENERGY_LEVEL_HIGH;
      PIN_HVPS_POLARITY_OUT = !OLL_POLARITY_NORMAL;
      PIN_AFC_TRIGGER_ENABLE_OUT = OLL_AFC_TRIGGER_ENABLE;
      if (global_data_A36487.this_pulse_level != global_data_A36487.next_pulse_level) {
	PIN_AFC_TRIGGER_ENABLE_OUT = !OLL_AFC_TRIGGER_ENABLE;
      }
      break;
      
    case PULSE_LEVEL_OFF:
      // DPARKER - Disable Pulsing
      break;
      
    default:
      break;
    }

  // DPARKER - Should this move somewhere else???
  // Adjust trigger frequency for self trigger mode
  if (trigger_set_point_active_decihertz != trigger_set_high_energy_decihertz) {
    trigger_set_point_active_decihertz = trigger_set_high_energy_decihertz;
    SetupT3PRFDeciHertz(trigger_set_point_active_decihertz);
  }
}



int SendPersonalityToPLC(unsigned char id) {

  //The PIC will use the gun driver,PFN, and RF faults to the PLC
  //as outputs momentarily to tell the PLC which personality
  //module is installed
  
  int ret = 0;
  
  /*
  _G0
  _A6
  _A7
  */
  unsigned int tris_A_bak;
  unsigned int tris_G_bak;

  tris_A_bak = TRISA;
  tris_G_bak = TRISG;

  _TRISG0 = 0;
  _TRISA6 = 0;
  _TRISA7 = 0;

#define PIN_CPU_PFN_OK_OUT              _LATG0
#define PIN_CPU_GUNDRIVER_OK_OUT        _LATA6
#define PIN_CPU_RF_OK_OUT               _LATA7
  
  if (id == 0) {
    PIN_CPU_PFN_OK_OUT = 0;
    PIN_CPU_GUNDRIVER_OK_OUT = 0;
    PIN_CPU_RF_OK_OUT = 0;
  } else if (id == 1) {
    PIN_CPU_PFN_OK_OUT = 1;
    PIN_CPU_GUNDRIVER_OK_OUT = 1;
    PIN_CPU_RF_OK_OUT = 0;
  } else if (id == 2) {
    PIN_CPU_PFN_OK_OUT = 1;
    PIN_CPU_GUNDRIVER_OK_OUT = 0;
    PIN_CPU_RF_OK_OUT = 1;
  } else if (id == 3) {
    PIN_CPU_PFN_OK_OUT = 0;
    PIN_CPU_GUNDRIVER_OK_OUT = 1;
    PIN_CPU_RF_OK_OUT = 1;
  } else {
    PIN_CPU_PFN_OK_OUT = 1;
    PIN_CPU_GUNDRIVER_OK_OUT = 1;
    PIN_CPU_RF_OK_OUT = 1;
  }

  PIN_ANALOG_READ_COMPLETE_OUT = !OLL_ANALOG_READ_COMPLETE;   //PLCin = 1
  __delay32(DELAY_PLC); // 250ms for 10M TCY
  if (PIN_PACKAGE_VALID_IN == !ILL_PACKAGE_VALID) {
    ret = 1;       //Failure to Communicate
  }
  PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;    //PLCin = 0
  __delay32(DELAY_PLC); // 250ms for 10M TCY
  if (PIN_PACKAGE_VALID_IN == ILL_PACKAGE_VALID) {
    ret = 1;       //Failure to Communicate
  }
  
  if (id == 0) {
    PIN_CPU_PFN_OK_OUT = 1;
    PIN_CPU_GUNDRIVER_OK_OUT = 1;
    PIN_CPU_RF_OK_OUT = 1;
  } else if (id == 1) {
    PIN_CPU_PFN_OK_OUT = 0;
    PIN_CPU_GUNDRIVER_OK_OUT = 0;
    PIN_CPU_RF_OK_OUT = 1;
  } else if (id == 2) {
    PIN_CPU_PFN_OK_OUT = 0;
    PIN_CPU_GUNDRIVER_OK_OUT = 1;
    PIN_CPU_RF_OK_OUT = 0;
  } else if (id == 3) {
    PIN_CPU_PFN_OK_OUT = 1;
    PIN_CPU_GUNDRIVER_OK_OUT = 0;
    PIN_CPU_RF_OK_OUT = 0;
  } else {
    PIN_CPU_PFN_OK_OUT = 1;
    PIN_CPU_GUNDRIVER_OK_OUT = 1;
    PIN_CPU_RF_OK_OUT = 1;
  }
  
  PIN_ANALOG_READ_COMPLETE_OUT = !OLL_ANALOG_READ_COMPLETE;   //PLCin = 1
  __delay32(DELAY_PLC); // 250ms for 10M TCY
  if (PIN_PACKAGE_VALID_IN == !ILL_PACKAGE_VALID) {
    ret = 1;       //Failure to Communicate
  }
  PIN_ANALOG_READ_COMPLETE_OUT = OLL_ANALOG_READ_COMPLETE;    //PLCin = 0
  __delay32(DELAY_PLC); // 250ms for 10M TCY
  if (PIN_PACKAGE_VALID_IN == ILL_PACKAGE_VALID) {
    ret = 1;       //Failure to Communicate
  }


  TRISA = tris_A_bak;
  TRISG = tris_G_bak;

  return ret;       //Communication Successful = 0
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
#ifndef __PLC_INTERFACE
  unsigned int index_word;
  //unsigned int temp;
  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
            
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
      
    default:
      ETMCanSlaveIncrementInvalidIndex();
      break;
    }
#endif
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
#ifndef __INTERNAL_TRIGGER
  unsigned int previous_level;
  unsigned int previous_width;

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
      
      // Prepare for future pulses
      previous_level = global_data_A36487.this_pulse_level;
      previous_width = global_data_A36487.this_pulse_width;
      
      global_data_A36487.this_pulse_level = global_data_A36487.next_pulse_level;
      global_data_A36487.this_pulse_width = global_data_A36487.next_pulse_width;
      
      global_data_A36487.next_pulse_level = previous_level;
      global_data_A36487.next_pulse_width = previous_width;
      
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
      U2MODEbits.UARTEN = 0;
      uart2_next_byte = 0;
      uart2_input_buffer[0] = 0x1F;
      uart2_input_buffer[1] = 0x2F;
      uart2_input_buffer[2] = 0x3F;
      U2MODEbits.UARTEN = 1;
      if (U2STAbits.OERR) {
	U2STAbits.OERR = 0;
      }
      
      if (global_data_A36487.message_received == 0) {
	global_data_A36487.bad_message_count++;
	global_data_A36487.total_missed_messages++;
      } else {
	if (global_data_A36487.bad_message_count) {
	  global_data_A36487.bad_message_count--;
	}
      }
      global_data_A36487.message_received = 0;
      
      
      // DPARKER - REDUCE THIS DELAY TO ACCOUNT FOR THE TIME IT TAKES TO GET HERE
      __delay32(300);  // Delay 30 uS
      
      UpdateEnergyAndPolarityOutputs();

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
#endif
  _INT1IF = 0;
}

  

void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _U2RXInterrupt(void) {
#ifndef __INTERNAL_TRIGGER
  unsigned int crc_check;
  unsigned char data;
  while (U2STAbits.URXDA) {
    data = U2RXREG;
    uart2_input_buffer[uart2_next_byte] = data;
    uart2_next_byte++;
    uart2_next_byte &= 0x000F;
  }
  
  if (uart2_next_byte >= 6) {
    PIN_40US_TEST_POINT = 1;
    crc_check   = uart2_input_buffer[5];
    crc_check <<= 8;
    crc_check  += uart2_input_buffer[4];
    if (ETMCRCModbus(&uart2_input_buffer[0],4) == crc_check) {
      if (uart2_input_buffer[2] & 0x01) {
	global_data_A36487.next_pulse_level = DOSE_COMMAND_LOW_ENERGY;
	PIN_40US_TEST_POINT = 0;
      } else {
	global_data_A36487.next_pulse_level = DOSE_COMMAND_HIGH_ENERGY;
	PIN_40US_TEST_POINT = 1;
      }
      
      global_data_A36487.uart_message_type = (uart2_input_buffer[0] >> 4);
      global_data_A36487.uart_sequence_id  = (uart2_input_buffer[0] & 0x0F);
      
      trigger_width = uart2_input_buffer[1];
      trigger_width_filtered = uart2_input_buffer[1];
      global_data_A36487.this_pulse_width = trigger_width_filtered;
      global_data_A36487.prf_from_concentrator = uart2_input_buffer[3];

      
      global_data_A36487.trigger_width_update_ready = 1;
      global_data_A36487.message_received = 1;
      global_data_A36487.message_received_count++;
    }
    uart2_next_byte = 0;
    uart2_input_buffer[0] = 0x1F;
    uart2_input_buffer[1] = 0x2F;
    uart2_input_buffer[2] = 0x3F;
  }
#endif
  _U2RXIF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
#ifdef __PLC_INTERFACE
  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A36487.analog_1.adc_accumulator += ADCBUF0;
    global_data_A36487.analog_2.adc_accumulator += ADCBUF1;
    global_data_A36487.analog_3.adc_accumulator += ADCBUF2;
    global_data_A36487.analog_4.adc_accumulator += ADCBUF3;
    global_data_A36487.analog_5.adc_accumulator += ADCBUF4;
    global_data_A36487.analog_6.adc_accumulator += ADCBUF5;
    global_data_A36487.analog_7.adc_accumulator += ADCBUF6;
    global_data_A36487.analog_8.adc_accumulator += ADCBUF7;
  } else {
    // read ADCBUF 8-15
    global_data_A36487.analog_1.adc_accumulator += ADCBUF8;
    global_data_A36487.analog_2.adc_accumulator += ADCBUF9;
    global_data_A36487.analog_3.adc_accumulator += ADCBUFA;
    global_data_A36487.analog_4.adc_accumulator += ADCBUFB;
    global_data_A36487.analog_5.adc_accumulator += ADCBUFC;
    global_data_A36487.analog_6.adc_accumulator += ADCBUFD;
    global_data_A36487.analog_7.adc_accumulator += ADCBUFE;
    global_data_A36487.analog_8.adc_accumulator += ADCBUFF;
  }
  
  global_data_A36487.accumulator_counter++ ;
  
  if (global_data_A36487.accumulator_counter >= 128) {

    // convert accumulators to 16 bit integer
    global_data_A36487.analog_1.adc_accumulator >>= 3;
    global_data_A36487.analog_2.adc_accumulator >>= 3;
    global_data_A36487.analog_3.adc_accumulator >>= 3;
    global_data_A36487.analog_4.adc_accumulator >>= 3;
    global_data_A36487.analog_5.adc_accumulator >>= 3;
    global_data_A36487.analog_6.adc_accumulator >>= 3;
    global_data_A36487.analog_7.adc_accumulator >>= 3;
    global_data_A36487.analog_8.adc_accumulator >>= 3;

    // Store the values
    global_data_A36487.analog_1.filtered_adc_reading = global_data_A36487.analog_1.adc_accumulator;
    global_data_A36487.analog_2.filtered_adc_reading = global_data_A36487.analog_2.adc_accumulator;
    global_data_A36487.analog_3.filtered_adc_reading = global_data_A36487.analog_3.adc_accumulator;
    global_data_A36487.analog_4.filtered_adc_reading = global_data_A36487.analog_4.adc_accumulator;
    global_data_A36487.analog_5.filtered_adc_reading = global_data_A36487.analog_5.adc_accumulator;
    global_data_A36487.analog_6.filtered_adc_reading = global_data_A36487.analog_6.adc_accumulator;
    global_data_A36487.analog_7.filtered_adc_reading = global_data_A36487.analog_7.adc_accumulator;
    global_data_A36487.analog_8.filtered_adc_reading = global_data_A36487.analog_8.adc_accumulator;

    // Clear the accumlators
    global_data_A36487.analog_1.adc_accumulator = 0;
    global_data_A36487.analog_2.adc_accumulator = 0;
    global_data_A36487.analog_3.adc_accumulator = 0;
    global_data_A36487.analog_4.adc_accumulator = 0;
    global_data_A36487.analog_5.adc_accumulator = 0;
    global_data_A36487.analog_6.adc_accumulator = 0;
    global_data_A36487.analog_7.adc_accumulator = 0;
    global_data_A36487.analog_8.adc_accumulator = 0;
    
    global_data_A36487.accumulator_counter = 0;
  }
#endif
  _ADIF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    __asm__ ("Reset");
}



