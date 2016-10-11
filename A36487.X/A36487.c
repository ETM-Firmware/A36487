#include "A36487.h"
//#define __COMPILE_MODE_2_5

unsigned char ReverseBits(unsigned char byte_to_reverse);

#define DOSE_COMMAND_LOW_ENERGY   0  
#define DOSE_COMMAND_HIGH_ENERGY  1
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

#ifdef __COMPILE_MODE_2_5
      // For the 2.5 Do Not Pulse in both Mode bits are in the same state
      if (PIN_LOW_MODE_IN == PIN_HIGH_MODE_IN) {
	PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
      } else {
	PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
      }
#else
      // For the 6/4, Do not pulse if neither Mode Bits are selected
      if ((PIN_LOW_MODE_IN != ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN != ILL_MODE_BIT_SELECTED)) {
	PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
      } else {
	PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
      }
#endif      

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

  // Initialize Pins
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
  
  // Initialize all I/O Registers
  TRISA = A36487_TRISA_VALUE;
  TRISB = A36487_TRISB_VALUE;
  TRISC = A36487_TRISC_VALUE;
  TRISD = A36487_TRISD_VALUE;
  TRISF = A36487_TRISF_VALUE;
  TRISG = A36487_TRISG_VALUE;

  // Set up external INT3 */
  // This is the trigger interrupt
  _INT3IF = 0;		// Clear Interrupt flag
  _INT3IE = 1;		// Enable INT3 Interrupt
  _INT3EP = 1; 	        // Interrupt on falling edge
  _INT3IP = 6;		// Set interrupt to high priority
  

  // Set up the TM3 interrupt
  _T3IF = 0;
  _T3IE = 0;
  _T3IP = 7;            // Set interrupt to highest priority
  
  // Setup Timer 1 to measure interpulse period.
  T1CON = T1CON_VALUE;
  PR1 = PR1_VALUE_400mS;
  
  // Setupt Timer 2 to generate 10mS Period
  T2CON = T2CON_VALUE;
  PR2 = PR2_VALUE_10mS;
  TMR2 = 0;
  _T2IF = 0;
  

  // Setupt Timer 3 to measure the Pulse Holdoff time to prevent over PRF
  T3CON = T3CON_VALUE;
  TMR3 = 0;
  _T3IF = 0;


  // Read the personality module
#ifdef __COMPILE_MODE_2_5
  global_data_A36487.personality = 0;
#else
  global_data_A36487.personality = ReadDosePersonality();
  global_data_A36487.personality = 0; // DPARKER THIS HAS BEEN ADDED FOR TESTING ONLY - REMOVE
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
  ETMCanSlaveLoadConfiguration(36487, 251, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);
  

  // Configure SPI Module to write to the delay line sift Registers
  ConfigureSPI(ETM_SPI_PORT_2, ETM_DEFAULT_SPI_CON_VALUE, ETM_DEFAULT_SPI_CON2_VALUE, ETM_DEFAULT_SPI_STAT_VALUE, SPI_CLK_5_MBIT, FCY);
  // DPARKER CONSIDER WRITING MODULE FOR SHIFT REGISTER DELAY LINE

  
  // Configure UART2 for communicating with Customer
  U2BRG = A36487_U2_BRG_VALUE;
  U2STA = A36487_U2_STA_VALUE;
  U2MODE = A36487_U2_MODE_VALUE;
  uart2_next_byte = 0;  
  _U2RXIF = 0;
  _U2RXIP = 5;
  _U2RXIE = 1;

  
  // Initialize the ADC to be off with all pins digital inputs
  ADCON1 = ADCON1_SETTING;  
  ADPCFG = ADPCFG_SETTING;

  // Initialize the digital faults
  ETMDigitalInitializeInput(&global_data_A36487.pfn_fan_fault, ILL_PIN_PFN_FAULT, 50);

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
  global_data_A36487.pulses_on++; // This counts the pulses
  ETMCanSlavePulseSyncSendNextPulseLevel(GetThisPulseLevel(), global_data_A36487.pulses_on, log_data_rep_rate_deci_hertz);
  
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
  PIN_PIC_PRF_OK = OLL_PIC_PRF_INHIBIT;
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
  PIN_LD_DELAY_AFC_OUT = 0;
  Nop();
  PIN_LD_DELAY_AFC_OUT = 1;
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
  /*
    If High Mode is Selected and Low Mode is not, Return High Mode
    If Low Mode is Selected and High Mode is not, Return low Mode
    If Neither mode is selected, return high mode (the system will be disabled by another function)
    If Both modes are selected
       - For the 6/4 use the software level select
       - For the 2.5 return high mode, but X-Rays will be inhibited by another function
   */

  if ((PIN_LOW_MODE_IN == ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN == ILL_MODE_BIT_SELECTED)) {
    // High and low mode selected
#ifdef __COMPILE_MODE_2_5
    return DOSE_COMMAND_HIGH_ENERGY;
#else
    return global_data_A36487.this_pulse_level_energy_command;

#endif
  }
  
  if ((PIN_LOW_MODE_IN != ILL_MODE_BIT_SELECTED) && (PIN_HIGH_MODE_IN != ILL_MODE_BIT_SELECTED)) {
    // Neither High or Low Mode selected - X_Rays will be disabled
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
  unsigned char data;
  _U2RXIF = 0;
  while (U2STAbits.URXDA) {
    data = U2RXREG;
    //uart2_input_buffer[uart2_next_byte] = ReverseBits(data);
    uart2_input_buffer[uart2_next_byte] = data;
    uart2_next_byte++;
    uart2_next_byte &= 0x000F;
  }
  
  if (uart2_next_byte >= 6) {
    crc_check   = uart2_input_buffer[5];
    crc_check <<= 8;
    crc_check  += uart2_input_buffer[4];

    if (ETMCRC16(&uart2_input_buffer[0],4) == crc_check) {

      _TRISC14 = 0;
      if (uart2_input_buffer[1] & 0x01) {
	global_data_A36487.next_pulse_level_energy_command = DOSE_COMMAND_LOW_ENERGY;
      } else {
	global_data_A36487.next_pulse_level_energy_command = DOSE_COMMAND_HIGH_ENERGY;
      }
      
      
      trigger_width = uart2_input_buffer[0];
      trigger_width_filtered = uart2_input_buffer[0];
      ProgramShiftRegistersGrid(uart2_input_buffer[0]);
      // DPARKER - Debugging Only
      if (global_data_A36487.next_pulse_level_energy_command == DOSE_COMMAND_HIGH_ENERGY) {
	_LATC14 = 1;
      } else {
	_LATC14 = 0;
      }

    }
    uart2_next_byte = 0;
  }
}

unsigned char ReverseBits(unsigned char byte_to_reverse) {
  unsigned char output = 0;
  if (byte_to_reverse & 0b00000001) {
    output |= 0b10000000;
  }

  if (byte_to_reverse & 0b00000010) {
    output |= 0b01000000;
  }

  if (byte_to_reverse & 0b00000100) {
    output |= 0b00100000;
  }

  if (byte_to_reverse & 0b00001000) {
    output |= 0b00010000;
  }

  if (byte_to_reverse & 0b00010000) {
    output |= 0b00001000;
  }

  if (byte_to_reverse & 0b00100000) {
    output |= 0b00000100;
  }

  if (byte_to_reverse & 0b01000000) {
    output |= 0b00000010;
  }
  
  if (byte_to_reverse & 0b10000000) {
    output |= 0b00000001;
  }

  return output;
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
    // Start The Holdoff Timer for the next pulse
    TMR3 = 0;
    PR3 = TMR3_DELAY_2400US;
    _T3IF = 0;
    _T3IE = 1;
    if (_T1IF) {
      // The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
      global_data_A36487.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
    }
    _T1IF = 0;
    
    global_data_A36487.trigger_complete = 1;
    global_data_A36487.this_pulse_level_energy_command = global_data_A36487.next_pulse_level_energy_command;
    uart2_next_byte = 0;
  }
  _INT3IF = 0;		// Clear Interrupt flag
}  

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
  /*
    We Can't enable XRays when if 40uS Line is high or we could get an error in pulse timing
    If 40us line is high reset the timer to check again in 200uS
  */

  if (PIN_40US_PULSE == ILL_PIN_40US_PULSE_ACTIVE) {
    // Renable this interrupt for 200us from now
    PR3 = TMR3_DELAY_200US;
    TMR3 = 0;
    _T3IF = 0;
    _T3IE = 1;
  } else {
    PIN_PIC_PRF_OK = !OLL_PIC_PRF_INHIBIT;
    _T3IF = 0;
    _T3IE = 0;
  }
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




