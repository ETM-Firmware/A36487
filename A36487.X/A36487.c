#include "A36487.h"
#define __COMPILE_MODE_2_5



#define DOSE_COMMAND_LOW_ENERGY   0  // DPARKER - Move these to the CAN Module - All boards need to know if the energy is commanded high or low
#define DOSE_COMMAND_HIGH_ENERGY  1

// Global Variables
TYPE_GLOBAL_DATA_A36487 global_data_A36487;
unsigned char uart2_input_buffer[16];
unsigned int  uart2_next_byte;


void DoStateMachine(void);
void InitializeA36487(void);
void InitPins(void); // DPARKER Change this to standard defenitions
unsigned char ReadDosePersonality(void);
void DoA36487(void);
void DoStartupLEDs(void);
void DoPostTriggerProcess(void);
void ProgramShiftRegistersDelays(void);
void ProgramShiftRegistersGrid(unsigned char dose_command);
unsigned int GetThisPulseLevel(void);
unsigned char InterpolateValue(unsigned int val_0, unsigned int val_1, unsigned int val_2, unsigned int val_3, unsigned char index);
void ResetCounter(void);


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
      
      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV() == 0) {
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
      
      if ((ETMCanSlaveGetSyncMsgPulseSyncDisableXray() == 0) && (PIN_CUSTOMER_BEAM_ENABLE_IN == ILL_CUSTOMER_BEAM_ENABLE)) {
	global_data_A36487.control_state = STATE_X_RAY_ENABLE;
      }
      
      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV()) {
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

      if (PIN_LOW_MODE_IN == PIN_HIGH_MODE_IN) {
	PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
      } else {
	PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
      }
      
      if (PIN_CUSTOMER_XRAY_ON_IN) {
	PIN_CPU_WARNING_LAMP_OUT = OLL_CPU_WARNING_LAMP;
      } else {
	PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
      }

      if (ETMCanSlaveGetSyncMsgPulseSyncDisableXray() || ETMCanSlaveGetSyncMsgPulseSyncDisableHV() || (PIN_CUSTOMER_BEAM_ENABLE_IN == !ILL_CUSTOMER_BEAM_ENABLE)) {
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
	global_data_A36487.control_state = STATE_WAIT_FOR_CONFIG;
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
  
  InitPins();
  
  global_data_A36487.counter_config_received = 0;
  
  // Set up external INT3 */
  // This is the trigger interrupt
  _INT3IF = 0;		// Clear Interrupt flag
  _INT3IE = 1;		// Enable INT3 Interrupt
  _INT3EP = 1; 	        // Interrupt on falling edge
  _INT3IP = 7;		// Set interrupt to highest priority
  

  // Setup Timer 1 to measure interpulse period.
  T1CON = T1CON_VALUE
  PR1 = PR1_VALUE_400mS
  
  // 10mS Period
  T2CON = T2CON_VALUE;
  PR2 = PR2_VALUE_10mS;
  TMR2 = 0;
  _T2IF = 0;
  


  global_data_A36487.personality = 0;
  global_data_A36487.personality = ReadDosePersonality(); // DPARKER UPDATE THIS FUNCTION IT DOESN'T WORK
  global_data_A36487.personality = 0; // DPARKER ADDED FOR TESTING TO MAKE IT WORK

  _STATUS_PERSONALITY_READ_COMPLETE = 1;

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


  ETMEEPromUseInternal();
  ETMCanSlaveInitialize(CAN_PORT_1, FCY, ETM_CAN_ADDR_PULSE_SYNC_BOARD, _PIN_RG14, 4, _PIN_RG12, _PIN_RC1);
  ETMCanSlaveLoadConfiguration(36487, 251, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);
  
  _STATUS_CUSTOMER_HV_DISABLE = 1;
  
  _STATUS_CUSTOMER_X_RAY_DISABLE = 1;
  
  _STATUS_LOW_MODE_OVERRIDE = 1;
  
  _STATUS_HIGH_MODE_OVERRIDE = 0;
  
  ETMDigitalInitializeInput(&global_data_A36487.pfn_fan_fault, ILL_PIN_PFN_FAULT, 50);

  ConfigureSPI(ETM_SPI_PORT_2, ETM_DEFAULT_SPI_CON_VALUE, ETM_DEFAULT_SPI_CON2_VALUE, ETM_DEFAULT_SPI_STAT_VALUE, SPI_CLK_5_MBIT, FCY);
  // DPARKER CONSIDER WRITING MODULE FOR SHIFT REGISTER DELAY LINE

  // Configure UART2 for communicating with Customer
  uart2_next_byte = 0;  
  _U2RXIF = 0;
  _U2RXIP = 6;
  _U2RXIE = 1;


  U1BRG = A36487_U2_BRG_VALUE;
  U1STA = A36487_U2_STA_VALUE;
  U1MODE = A36487_U2_MODE_VALUE;

  ADCON1 = ADCON1_SETTING;  
  ADPCFG = ADPCFG_SETTING;
}


void InitPins() {

  //Trigger Measurement Pins
  TRIS_PIN_TRIG_INPUT             = TRIS_INPUT_MODE;
  PIN_PW_SHIFT_OUT                = !OLL_PW_SHIFT;
  PIN_PW_CLR_CNT_OUT              = !OLL_PW_CLR_CNT;                // clear active
  PIN_PW_HOLD_LOWRESET_OUT        = !OLL_PW_HOLD_LOWRESET;	 // reset active
  TRIS_PIN_PW_SHIFT_OUT           = TRIS_OUTPUT_MODE;
  TRIS_PIN_PW_CLR_CNT_OUT         = TRIS_OUTPUT_MODE;
  TRIS_PIN_PW_HOLD_LOWRESET_OUT   = TRIS_OUTPUT_MODE;
  TRIS_PIN_40US_IN1               = TRIS_INPUT_MODE;
  TRIS_PIN_40US_IN2               = TRIS_INPUT_MODE;
  TRIS_PIN_TRIG_INPUT             = TRIS_INPUT_MODE;
  
  // Personality ID Pins
  PIN_ID_SHIFT_OUT            = !OLL_ID_SHIFT;
  TRIS_PIN_ID_SHIFT_OUT       = TRIS_OUTPUT_MODE;
  PIN_ID_CLK_OUT              = !OLL_ID_CLK;
  TRIS_PIN_ID_CLK_OUT         = TRIS_OUTPUT_MODE;
  TRIS_PIN_ID_DATA_IN         = TRIS_INPUT_MODE;
  
  //Spare pins (not used in current application)
  TRIS_PIN_PACKAGE_ID1_IN         = TRIS_INPUT_MODE;
  TRIS_PIN_READY_FOR_ANALOG_OUT   = TRIS_OUTPUT_MODE;
  PIN_READY_FOR_ANALOG_OUT        = OLL_READY_FOR_ANALOG;
  
  //Control to PFN control board for Gantry/Portal Selection
  TRIS_PIN_MODE_OUT           = TRIS_OUTPUT_MODE;
  
  //Hardware Status
  TRIS_PIN_KEY_LOCK_IN            = TRIS_INPUT_MODE;
  TRIS_PIN_PANEL_IN               = TRIS_INPUT_MODE;
  TRIS_PIN_XRAY_CMD_MISMATCH_IN   = TRIS_INPUT_MODE;
  //    PIN_CUSTOMER_BEAM_ENABLE_IN     = !ILL_CUSTOMER_BEAM_ENABLE;
  TRIS_PIN_CUSTOMER_BEAM_ENABLE_IN = TRIS_INPUT_MODE;
  PIN_CUSTOMER_XRAY_ON_IN         = !ILL_CUSTOMER_XRAY_ON;
  TRIS_PIN_CUSTOMER_XRAY_ON_IN    = TRIS_INPUT_MODE;
  
  //Energy Select Pins
  TRIS_PIN_LOW_MODE_IN        = TRIS_INPUT_MODE;
  TRIS_PIN_HIGH_MODE_IN 	= TRIS_INPUT_MODE;
  PIN_ENERGY_CPU_OUT          = !OLL_ENERGY_CPU;
  TRIS_PIN_ENERGY_CPU_OUT     = TRIS_OUTPUT_MODE;
  TRIS_PIN_AFC_TRIGGER_OK_OUT = TRIS_OUTPUT_MODE;
  PIN_AFC_TRIGGER_OK_OUT      = OLL_AFC_TRIGGER_OK;
  PIN_RF_POLARITY_OUT         = OLL_RF_POLARITY;
  TRIS_PIN_RF_POLARITY_OUT    = TRIS_OUTPUT_MODE;
  PIN_HVPS_POLARITY_OUT       = !OLL_HVPS_POLARITY;
  TRIS_PIN_HVPS_POLARITY_OUT  = TRIS_OUTPUT_MODE;
  PIN_GUN_POLARITY_OUT        = !OLL_GUN_POLARITY;
  TRIS_PIN_GUN_POLARITY_OUT   = TRIS_OUTPUT_MODE;
  TRIS_PIN_ENERGY_CMD_IN1     = TRIS_INPUT_MODE;
  TRIS_PIN_ENERGY_CMD_IN2     = TRIS_INPUT_MODE;
  
  //State Hardware Control
  TRIS_PIN_CPU_HV_ENABLE_OUT      = TRIS_OUTPUT_MODE;
  PIN_CPU_HV_ENABLE_OUT           = !OLL_CPU_HV_ENABLE;
  TRIS_PIN_CPU_XRAY_ENABLE_OUT    = TRIS_OUTPUT_MODE;
  PIN_CPU_XRAY_ENABLE_OUT         = !OLL_CPU_XRAY_ENABLE;
  TRIS_PIN_CPU_WARNING_LAMP_OUT   = TRIS_OUTPUT_MODE;
  PIN_CPU_WARNING_LAMP_OUT        = !OLL_CPU_WARNING_LAMP;
  TRIS_PIN_CPU_STANDBY_OUT        = TRIS_OUTPUT_MODE;
  PIN_CPU_STANDBY_OUT             = !OLL_CPU_STANDBY;
  TRIS_PIN_CPU_READY_OUT          = TRIS_OUTPUT_MODE;
  PIN_CPU_READY_OUT               = !OLL_CPU_READY;
  TRIS_PIN_CPU_SUMFLT_OUT         = TRIS_OUTPUT_MODE;
  PIN_CPU_SUMFLT_OUT              = !OLL_CPU_SUMFLT;
  TRIS_PIN_CPU_WARMUP_OUT         = TRIS_OUTPUT_MODE;
  //    PIN_CPU_WARMUP_OUT              = !OLL_CPU_WARMUP;
  
  //LEDs
  TRIS_PIN_LED_READY          = TRIS_OUTPUT_MODE;
  PIN_LED_READY               = !OLL_LED_ON;
  //TRIS_PIN_LED_STANDBY        = TRIS_OUTPUT_MODE;
  //PIN_LED_STANDBY             = !OLL_LED_ON;
  //TRIS_PIN_LED_WARMUP         = TRIS_OUTPUT_MODE;
  //    PIN_LED_WARMUP              = !OLL_LED_ON;
  TRIS_PIN_LED_XRAY_ON        = TRIS_OUTPUT_MODE;
  PIN_LED_XRAY_ON             = !OLL_LED_ON;
  //TRIS_PIN_LED_SUMFLT         = TRIS_OUTPUT_MODE;
  //PIN_LED_SUMFLT              = !OLL_LED_ON;
  
  // Pins for loading the delay lines
  PIN_SPI_CLK_OUT             = 0;
  TRIS_PIN_SPI_CLK_OUT        = TRIS_OUTPUT_MODE;
  PIN_SPI_DATA_OUT            = 0;
  TRIS_PIN_SPI_DATA_OUT       = TRIS_OUTPUT_MODE;
  TRIS_PIN_SPI_DATA_IN        = TRIS_INPUT_MODE;
  TRIS_PIN_LD_DELAY_PFN_OUT   = TRIS_OUTPUT_MODE;
  PIN_LD_DELAY_PFN_OUT        = 0;
  TRIS_PIN_LD_DELAY_AFC_OUT   = TRIS_OUTPUT_MODE;
  PIN_LD_DELAY_AFC_OUT        = 0;
  TRIS_PIN_LD_DELAY_GUN_OUT   = TRIS_OUTPUT_MODE;
  PIN_LD_DELAY_GUN_OUT        = 0;
  
  //Bypass these to allow xray on
  TRIS_PIN_RF_OK             = TRIS_INPUT_MODE;
  TRIS_PIN_GUN_OK            = TRIS_INPUT_MODE;
  TRIS_PIN_PFN_OK            = TRIS_INPUT_MODE;
  
  //Communications
  COMM_DRIVER_ENABLE_TRIS = TRIS_OUTPUT_MODE;
  COMM_DRIVER_ENABLE_PIN = 0;
  COMM_RX_TRIS = TRIS_INPUT_MODE;
  COMM_TX_TRIS = TRIS_OUTPUT_MODE;
  
}

unsigned char ReadDosePersonality() {
  unsigned int data;
  unsigned char i, data1, data2;
  
  PIN_ID_CLK_OUT   = !OLL_ID_CLK;
  PIN_ID_SHIFT_OUT = !OLL_ID_SHIFT; // load the reg
  __delay32(1); // 100ns for 10M TCY
  PIN_ID_SHIFT_OUT = OLL_ID_SHIFT;  // enable shift
  __delay32(1); // 100ns for 10M TCY
  
  data = PIN_ID_DATA_IN;
  
  for (i = 0; i < 8; i++) {
    PIN_ID_CLK_OUT = OLL_ID_CLK;
    data <<= 1;
    data |= PIN_ID_DATA_IN;
    PIN_ID_CLK_OUT = !OLL_ID_CLK;
    __delay32(1); // 100ns for 10M TCY
  }
  
  //if bits do not match then bad module
  data1 = data & 0x01;
  data2 = data & 0x10;
  if (data1 != (data2 >> 4))
    return 0xFF;
  data1 = data & 0x02;
  data2 = data & 0x20;
  if (data1 != (data2 >> 4))
    return 0xFF;
  data1 = data & 0x04;
  data2 = data & 0x40;
  if (data1 != (data2 >> 4))
    return 0xFF;
  data1 = data & 0x08;
  data2 = data & 0x80;
  if (data1 != (data2 >> 4))
    return 0xFF;
  
  //bit 3 is 1 except when 0,1,2 are 1
  data1 = data & 0x08;
  data2 = data & 0x07;
  if (data1 != data2)
    return 0xFF;
  
  if (data == ULTRA_LOW_DOSE)
    return 0x02;
  else if (data == LOW_DOSE)
    return 0x04;
  else if (data == MEDIUM_DOSE)
    return 0x08;
  else if (data == HIGH_DOSE)
    return 0x10;
  else if (data == 0xFF)
    return data;
  else
    return 0;
}

void DoA36487(void) {
  unsigned long temp32;

  ETMCanSlaveDoCan();
  
  if (global_data_A36487.trigger_complete) {
    DoPostTriggerProcess();
    global_data_A36487.trigger_complete = 0;
    
  }


  // ---------- UPDATE LOCAL FAULTS ------------------- //
  
  if ((global_data_A36487.control_state == STATE_FAULT) && ETMCanSlaveGetSyncMsgResetEnable()) {
    _FAULT_REGISTER = 0;
  }
  
  if (PIN_PANEL_IN == ILL_PANEL_OPEN) {
    _FAULT_PANEL_OPEN = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_PANEL_OPEN = 0;
    }
  }
  
  if (PIN_KEY_LOCK_IN == ILL_KEY_LOCK_FAULT) {
    _FAULT_KEYLOCK_OPEN = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_KEYLOCK_OPEN = 0;
    }
  }

  ETMDigitalUpdateInput(&global_data_A36487.pfn_fan_fault, PIN_PFN_OK);
  
  if (ETMDigitalFilteredOutput(&global_data_A36487.pfn_fan_fault) == ILL_PIN_PFN_FAULT) {
    _FAULT_PFN_STATUS = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_PFN_STATUS = 0;
    }
  }
  
  if (PIN_RF_OK == ILL_PIN_RF_FAULT) {
    _FAULT_RF_STATUS = 1;
  } else {
    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      _FAULT_RF_STATUS = 0;
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

  if (ETMCanSlaveGetSyncMsgResetEnable() && (PIN_TRIG_INPUT != ILL_TRIG_ON)) {
    _FAULT_TRIGGER_STAYED_ON = 0;
    _STATUS_TRIGGER_STAYED_ON = 0;
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
    if (_T1IF) {
      ResetCounter();
    }
  
    // Update the debugging Data
    //ETMCanSlaveSetDebugRegister(0, (grid_start_high3 << 8) + grid_start_high2);
    //ETMCanSlaveSetDebugRegister(1, (grid_start_high1 << 8) + grid_start_high0);
    //ETMCanSlaveSetDebugRegister(2, (pfn_delay_high << 8) + dose_sample_delay_high);
    ETMCanSlaveSetDebugRegister(3, (grid_stop_high3 << 8) + grid_stop_high2);
    ETMCanSlaveSetDebugRegister(4, (grid_stop_high1 << 8) + grid_stop_high0);
    ETMCanSlaveSetDebugRegister(5, (afc_delay_high << 8) + magnetron_current_sample_delay_high);
    ETMCanSlaveSetDebugRegister(6, (grid_start_low3 << 8) + grid_start_low2);
    ETMCanSlaveSetDebugRegister(7, (grid_start_low1 << 8) + grid_start_low0);
    ETMCanSlaveSetDebugRegister(8, (pfn_delay_low << 8) + dose_sample_delay_low);
    ETMCanSlaveSetDebugRegister(9, data_grid_start);
    ETMCanSlaveSetDebugRegister(10, data_grid_stop);
    ETMCanSlaveSetDebugRegister(0xb, global_data_A36487.pulses_on);
    ETMCanSlaveSetDebugRegister(0xC, global_data_A36487.last_period);
    ETMCanSlaveSetDebugRegister(0xD, log_data_rep_rate_deci_hertz);
    ETMCanSlaveSetDebugRegister(0xE, 0);
    ETMCanSlaveSetDebugRegister(0xF, global_data_A36487.period_filtered);
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
  global_data_A36487.pulses_on++;       // This counts the pulses
  ETMCanSlavePulseSyncSendNextPulseLevel(global_data_A36487.this_pulse_level_energy_command, global_data_A36487.pulses_on, log_data_rep_rate_deci_hertz);

  global_data_A36487.period_filtered = RCFilterNTau(global_data_A36487.period_filtered, global_data_A36487.last_period, RC_FILTER_4_TAU);  // Update the PFN
  ProgramShiftRegistersDelays();  // Load the shift registers

  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    // Log Pulse by Pulse data
    ETMCanSlaveLogPulseData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_LOG_0,
			    (global_data_A36487.pulses_on - 1),
			    *(unsigned int*)&trigger_width,
			    *(unsigned int*)&data_grid_start,
			    log_data_rep_rate_deci_hertz);
  }
    
  // This is used to detect if the trigger is high (which would cause constant pulses to the system)
  if (PIN_TRIG_INPUT == ILL_TRIG_ON) {
    // if pulse trig stays on, disable pulsing and flag fault
    _STATUS_TRIGGER_STAYED_ON = 1;
    _FAULT_TRIGGER_STAYED_ON = 1;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
  }
}


void ProgramShiftRegistersDelays(void) {
  unsigned int data;
  unsigned char hv_trigger_delay;
  unsigned char pfn_trigger_delay;
  unsigned char magnetron_sample_delay;
  unsigned char afc_sample_delay;
  
  if (GetThisPulseLevel() == DOSE_COMMAND_HIGH_ENERGY) {
    hv_trigger_delay = dose_sample_delay_high;
    pfn_trigger_delay = pfn_delay_high;
    magnetron_sample_delay = magnetron_current_sample_delay_high;
    afc_sample_delay = afc_delay_high;
  } else {
    hv_trigger_delay = dose_sample_delay_low;
    pfn_trigger_delay = pfn_delay_low;
    magnetron_sample_delay = magnetron_current_sample_delay_low;
    afc_sample_delay = afc_delay_low;
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
  PIN_LD_DELAY_PFN_OUT = 0;
  Nop();
  PIN_LD_DELAY_PFN_OUT = 1;
  Nop();
}


void ProgramShiftRegistersGrid(unsigned char dose_command) {
  unsigned int data;

  if (GetThisPulseLevel() == DOSE_COMMAND_HIGH_ENERGY) {
    data_grid_stop  = InterpolateValue(grid_stop_high0, grid_stop_high1, grid_stop_high2, grid_stop_high3, dose_command);
    data_grid_start = InterpolateValue(grid_start_high0, grid_start_high1, grid_start_high2, grid_start_high3, dose_command);
  } else {
    data_grid_stop  = InterpolateValue(grid_stop_low0, grid_stop_low1, grid_stop_low2, grid_stop_low3, dose_command);
    data_grid_start = InterpolateValue(grid_start_low0, grid_start_low1, grid_start_low2, grid_start_low3, dose_command);
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
#ifdef __COMPILE_MODE_2_5
  if ((PIN_LOW_MODE_IN != ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN != ILL_MODE_BIT_SELECTED)) {
    // Neither High or Low Mode selected
    return DOSE_COMMAND_HIGH_ENERGY;
  }
  
  if ((PIN_LOW_MODE_IN == ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED)) {
    // High and low mode selected
    return DOSE_COMMAND_HIGH_ENERGY;
  }
  
  if ((PIN_LOW_MODE_IN != ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED)) {
    // High mode selected
    return DOSE_COMMAND_HIGH_ENERGY;
  }
  
  if ((PIN_LOW_MODE_IN == ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN != ILL_MODE_BIT_SELECTED)) {
    // Low mode selected
    return DOSE_COMMAND_LOW_ENERGY;
  }

  return DOSE_COMMAND_HIGH_ENERGY;
  
#else
  // Place code for 6/4 system here - When 6/4 system moves to CAN
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


void ResetCounter(void) {
  PIN_PW_CLR_CNT_OUT = OLL_PW_CLR_CNT;
  __delay32(100);
  PIN_PW_CLR_CNT_OUT = !OLL_PW_CLR_CNT;
}


void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _U2RXInterrupt(void) {
  unsigned int crc_check;
  _U2RXIF = 0;
  while (U2STAbits.URXDA) {
    uart2_input_buffer[uart2_next_byte] = U2RXREG;
    uart2_next_byte++;
    uart2_next_byte &= 0x000F;
  }
  
  if (uart2_next_byte >= 6) {
    crc_check   = uart2_input_buffer[4];
    crc_check <<= 8;
    crc_check  += uart2_input_buffer[5];

    if (ETMCRC16(&uart2_input_buffer[0],4) == crc_check) {
      if (uart2_input_buffer[1] & 0x01) {
	global_data_A36487.next_pulse_level_energy_command = DOSE_COMMAND_LOW_ENERGY;
      } else {
	global_data_A36487.next_pulse_level_energy_command = DOSE_COMMAND_HIGH_ENERGY;
      }
      trigger_width = uart2_input_buffer[0];
      trigger_width_filtered = uart2_input_buffer[0];
      ProgramShiftRegistersGrid(uart2_input_buffer[0]);
    }
    uart2_next_byte = 0;
  }
}


#define MIN_PERIOD 150 // 960uS 1041 Hz// 
void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _INT3Interrupt(void) {
  // A trigger was recieved.
  // THIS DOES NOT MEAN THAT A PULSE WAS GENERATED
  // If (PIN_CPU_XRAY_ENABLE_OUT == OLL_CPU_XRAY_ENABLE)  && (PIN_CUSTOMER_XRAY_ON_IN == ILL_CUSTOMER_BEAM_ENABLE) then we "Probably" generated a pulse
  if ((TMR1 > MIN_PERIOD) || _T1IF) {
    // Calculate the Trigger PRF
    // TMR1 is used to time the time between INT3 interrupts
    global_data_A36487.last_period = TMR1;
    TMR1 = 0;
    if (_T1IF) {
      // The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
      global_data_A36487.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
    }
    /*
      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV() || ETMCanSlaveGetSyncMsgPulseSyncDisableXray()) {
      // We are not pulsing so set the PRF to the minimum
      global_data_A36487.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
      }
    */
    _T1IF = 0;
    
    global_data_A36487.trigger_complete = 1;
    global_data_A36487.this_pulse_level_energy_command = global_data_A36487.next_pulse_level_energy_command;
    uart2_next_byte = 0;
  }
  _INT3IF = 0;		// Clear Interrupt flag
}  


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    Nop();
    Nop();
    __asm__ ("Reset");
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
      global_data_A36487.led_state = message_ptr->word0;
      break;
      
    default:
      ETMCanSlaveIncrementInvalidIndex();
      break;
    }
}




