#include "A36487.h"

// Global Variables
PSB_DATA psb_data;

//Limited scope variables
unsigned char change_pulse_width_counter;
const unsigned int  dose_intensities[4] = {15, 95, 175, 255};  // fixed constants


void DoStateMachine(void);
void InitializeA36487(void);
void DoA36487(void);
void DoStartupLEDs(void);
void DoPostTriggerProcess(void);
void ReadTrigPulseWidth(void);
unsigned char FilterTrigger(unsigned char param);
void ReadAndSetEnergy(void);
void ProgramShiftRegisters(void);
unsigned int GetInterpolationValue(unsigned int low_point, unsigned int high_point, unsigned low_value, unsigned high_value, unsigned point);
unsigned char ReadDosePersonality(void);
/*
  Return:
  0x00 = No dose personailty installed
  0x02 = Ultra Low Dose Personality Installed
  0x04 = Low Dose Personailty Installed
  0x08 = Medium Dose Personality Installed
  0x10 = High Dose Personailty Installed
  0xFF = Problem reading personailty module
*/
void InitPins(void); // DPARKER Change this to standard defenitions






//Processor Setup
_FOSC(EC & CSW_FSCM_OFF); // Primary Oscillator without PLL and Startup with User Selected Oscillator Source, CLKOUT 10MHz is used to measure trigger width.
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_27 & PBOR_ON & MCLR_EN); // Brown out and Power on Timer settings
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);








int main(void) {
  psb_data.state_machine = STATE_INIT;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  // LOOP Through the state machine is around 30uS Nominally, 50uS when it processes a sync command
  // Need to determine how much time processing a trigger takes.

  switch (psb_data.state_machine) {


  case STATE_INIT:
    InitializeA36487();
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    psb_data.state_machine = STATE_WAIT_FOR_CONFIG;
    break;

  case STATE_WAIT_FOR_CONFIG:
    _CONTROL_NOT_READY = 1;
    _CONTROL_NOT_CONFIGURED = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    while (psb_data.state_machine == STATE_WAIT_FOR_CONFIG) {
      DoA36487();
      DoStartupLEDs();

      PIN_CPU_WARMUP_OUT = !OLL_CPU_WARMUP;
      PIN_CPU_STANDBY_OUT = !OLL_CPU_STANDBY;
      PIN_CPU_READY_OUT = !OLL_CPU_READY;
      PIN_CPU_SUMFLT_OUT = OLL_CPU_SUMFLT;
      
      if ((psb_data.led_flash_counter >= LED_STARTUP_FLASH_TIME) && (psb_data.counter_config_received == 0b1111)) {
	psb_data.state_machine = STATE_HV_OFF;
      }
    }
    break;

  case STATE_HV_OFF:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    while (psb_data.state_machine == STATE_HV_OFF) {
      DoA36487();
      
      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV() == 0) {
	psb_data.state_machine = STATE_HV_ENABLE;
      }
      
      if (_FAULT_REGISTER) {
	psb_data.state_machine = STATE_FAULT;
      }
    }
    break;


  case STATE_HV_ENABLE:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    while (psb_data.state_machine == STATE_HV_ENABLE) {
      DoA36487();
      
      if ((ETMCanSlaveGetSyncMsgPulseSyncDisableXray() == 0) && (PIN_CUSTOMER_BEAM_ENABLE_IN == ILL_CUSTOMER_BEAM_ENABLE)) {
	psb_data.state_machine = STATE_X_RAY_ENABLE;
      }
      
      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV()) {
	psb_data.state_machine = STATE_HV_OFF;
      }

      if (_FAULT_REGISTER) {
	psb_data.state_machine = STATE_FAULT;
      }
    }
    break;


  case STATE_X_RAY_ENABLE:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 0;
    PIN_CPU_HV_ENABLE_OUT = OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    while (psb_data.state_machine == STATE_X_RAY_ENABLE) {
      DoA36487();
      
      if (PIN_CUSTOMER_XRAY_ON_IN) {
	PIN_CPU_WARNING_LAMP_OUT = OLL_CPU_WARNING_LAMP;
      } else {
	PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
      }

      if (ETMCanSlaveGetSyncMsgPulseSyncDisableXray() || ETMCanSlaveGetSyncMsgPulseSyncDisableHV() || (PIN_CUSTOMER_BEAM_ENABLE_IN == !ILL_CUSTOMER_BEAM_ENABLE)) {
	psb_data.state_machine = STATE_HV_ENABLE;
      }
      
      if (_FAULT_REGISTER) {
	psb_data.state_machine = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    while (psb_data.state_machine == STATE_FAULT) {
      DoA36487();
      
      if (_FAULT_REGISTER == 0) {
	psb_data.state_machine = STATE_WAIT_FOR_CONFIG;
      }
    }
    break;

      
  default:
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    PIN_CPU_HV_ENABLE_OUT = !OLL_CPU_HV_ENABLE;
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    PIN_CPU_WARNING_LAMP_OUT = !OLL_CPU_WARNING_LAMP;
    psb_data.state_machine = STATE_UNKNOWN;
    while (1) {
      DoA36487();
    }
    break;
    
  } // switch (psb_data.state_machine) {

}



void InitializeA36487(void) {
  
  InitPins();
  
  psb_data.counter_config_received = 0;
  
  // Set up external INT3 */
  // This is the trigger interrupt
  _INT3IF = 0;		// Clear Interrupt flag
  _INT3IE = 1;		// Enable INT3 Interrupt
  _INT3EP = 1; 	        // Interrupt on falling edge
  _INT3IP = 7;		// Set interrupt to highest priority
  
  
  // 10mS Period
  T2CON = (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_SOURCE_INT & T2_32BIT_MODE_OFF);
  PR2 = 12500;
  TMR2 = 0;
  _T2IF = 0;
  

  // Setup Timer 1 to measure interpulse period.
  T1CON = (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT);
  PR1 = 62500;  // 400mS
  

  psb_data.personality = 0;
  psb_data.personality = ReadDosePersonality(); // DPARKER UPDATE THIS FUNCTION IT DOESN'T WORK
  psb_data.personality = 0; // DPARKER ADDED FOR TESTING TO MAKE IT WORK

  _STATUS_PERSONALITY_READ_COMPLETE = 1;

  _PERSONALITY_BIT_0 = 0;
  _PERSONALITY_BIT_1 = 0;
  _PERSONALITY_BIT_2 = 0;
  _PERSONALITY_BIT_3 = 0;

  if (psb_data.personality & 0x0001) {
    _PERSONALITY_BIT_0 = 1;
  }

  if (psb_data.personality & 0x0002) {
    _PERSONALITY_BIT_1 = 1;
  }

  if (psb_data.personality & 0x0004) {
    _PERSONALITY_BIT_2 = 1;
  }

  if (psb_data.personality & 0x0008) {
    _PERSONALITY_BIT_3 = 1;
  }


#define AGILE_REV      77
#define SERIAL_NUMBER  100
  
  ETMCanSlaveInitialize(CAN_PORT_1, FCY, ETM_CAN_ADDR_PULSE_SYNC_BOARD, _PIN_RG14, 4, _PIN_RG12, _PIN_RC1);
  ETMCanSlaveLoadConfiguration(36487, 251, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);
}

unsigned int trigger_counter;

void DoA36487(void) {
  unsigned long temp32;

  ETMCanSlaveDoCan();
  
  if (psb_data.trigger_complete) {
    DoPostTriggerProcess();
    trigger_counter++;
    psb_data.trigger_complete = 0;
    
  }


  // ---------- UPDATE LOCAL FAULTS ------------------- //
  
  if ((psb_data.state_machine == STATE_FAULT) && ETMCanSlaveGetSyncMsgResetEnable()) {
    _FAULT_REGISTER = 0;
  }
  
  if (PIN_PANEL_IN == ILL_PANEL_OPEN) {
    _FAULT_PANEL = 1;
  }
  
  if (PIN_KEY_LOCK_IN == ILL_KEY_LOCK_FAULT) {
    _FAULT_KEYLOCK = 1;
  }

  if (PIN_PFN_OK == ILL_PIN_PFN_FAULT) {
    _FAULT_PFN_STATUS = 1;
  }
  
  if (PIN_RF_OK == ILL_PIN_RF_FAULT) {
    _FAULT_RF_STATUS = 1;
  }

  if (PIN_XRAY_CMD_MISMATCH_IN == !ILL_XRAY_CMD_MISMATCH) {
    _FAULT_TIMING_MISMATCH = 1;
  }

  // _FAULT_TRIGGER_STAYED_ON is set by INT3 Interrupt // DPARKER Look at this more

  if ((PIN_CUSTOMER_XRAY_ON_IN) && (!PIN_CUSTOMER_BEAM_ENABLE_IN)) {
    _FAULT_X_RAY_ON_WIHTOUT_HV = 1;
  }
  


  // ------------- UPDATE STATUS -------------------- //

  _STATUS_CUSTOMER_HV_DISABLE = !PIN_CUSTOMER_BEAM_ENABLE_IN;

  _STATUS_CUSTOMER_X_RAY_DISABLE = !PIN_CUSTOMER_XRAY_ON_IN;

  _STATUS_LOW_MODE_OVERRIDE = PIN_LOW_MODE_IN;
  
  _STATUS_HIGH_MODE_OVERRIDE = PIN_HIGH_MODE_IN;
  
  
  
  if (_T2IF) {
    // Run these once every 10ms
    _T2IF = 0;

    psb_data.led_flash_counter++;
    //PIN_LED_STANDBY = ((psb_data.led_flash_counter >> 5) & 0b1);


    
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
      //PIN_LED_SUMFLT = OLL_LED_ON;
      PIN_CPU_SUMFLT_OUT = OLL_CPU_SUMFLT;
    } else {
      //PIN_LED_SUMFLT = !OLL_LED_ON;
      PIN_CPU_SUMFLT_OUT = !OLL_CPU_SUMFLT;
    }
    
    
    // DPARKER - NEED TO UPDATE THE REP RATE WHEN NOT PULSING!!!!!
    // Calculate the rep rate
    temp32 = 1562500;
    temp32 /= psb_data.period_filtered;
    log_data_rep_rate_deci_hertz = temp32;
    if (_T1IF || (log_data_rep_rate_deci_hertz < 25)) {
      // We are pulseing at less than 2.5Hz
      // Set the rep rate to zero
      log_data_rep_rate_deci_hertz = 0;
    }
  
    // Update the debugging Data
    ETMCanSlaveSetDebugRegister(0, (grid_start_high3 << 8) + grid_start_high2);
    ETMCanSlaveSetDebugRegister(1, (grid_start_high1 << 8) + grid_start_high0);
    ETMCanSlaveSetDebugRegister(2, (pfn_delay_high << 8) + dose_sample_delay_high);
    ETMCanSlaveSetDebugRegister(3, (grid_stop_high3 << 8) + grid_stop_high2);
    ETMCanSlaveSetDebugRegister(4, (grid_stop_high1 << 8) + grid_stop_high0);
    ETMCanSlaveSetDebugRegister(5, (afc_delay_high << 8) + magnetron_current_sample_delay_high);
    ETMCanSlaveSetDebugRegister(6, (grid_start_low3 << 8) + grid_start_low2);
    ETMCanSlaveSetDebugRegister(7, (grid_start_low1 << 8) + grid_start_low0);
    ETMCanSlaveSetDebugRegister(8, (pfn_delay_low << 8) + dose_sample_delay_low);
    ETMCanSlaveSetDebugRegister(9, (grid_stop_low3 << 8) + grid_stop_low2);
    ETMCanSlaveSetDebugRegister(10, (grid_stop_low1 << 8) + grid_stop_low0);
    ETMCanSlaveSetDebugRegister(0xb, psb_data.pulses_on);
    ETMCanSlaveSetDebugRegister(0xC, psb_data.last_period);
    ETMCanSlaveSetDebugRegister(0xD, log_data_rep_rate_deci_hertz);
    ETMCanSlaveSetDebugRegister(0xE, trigger_counter);
    ETMCanSlaveSetDebugRegister(0xF, psb_data.period_filtered);

    

  }
}


void DoStartupLEDs(void) {
  switch ((psb_data.led_flash_counter >> 4) & 0b11)
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
  
  // This is used to detect if the trigger is high (which would cause constant pulses to the system)
  if (PIN_TRIG_INPUT != ILL_TRIG_ON) {
    ReadTrigPulseWidth();
    ReadAndSetEnergy();
  } else {  // if pulse trig stays on, set to minimum dose and flag fault
    _FAULT_TRIGGER_STAYED_ON = 1;
    trigger_width_filtered = 0;
  }
  
  ProgramShiftRegisters();
  psb_data.period_filtered = RCFilterNTau(psb_data.period_filtered, psb_data.last_period, RC_FILTER_4_TAU);
    
  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    // Log Pulse by Pulse data
    ETMCanSlaveLogPulseData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_LOG_0,
			    psb_data.pulses_on,
			    *(unsigned int*)&trigger_width,
			    *(unsigned int*)&data_grid_start,
			    log_data_rep_rate_deci_hertz);
  }
  
  psb_data.pulses_on++;       // This counts the pulses
  ETMCanSlavePulseSyncSendNextPulseLevel(psb_data.energy, psb_data.pulses_on, log_data_rep_rate_deci_hertz);
}


void ReadTrigPulseWidth(void) {
  unsigned int data;
  unsigned char i;
  
  PIN_SPI_CLK_OUT  = 0;
  Nop();
  PIN_PW_SHIFT_OUT = !OLL_PW_SHIFT; // load the reg
  Nop();
  __delay32(1); // 100ns for 10M TCY
  PIN_PW_SHIFT_OUT = OLL_PW_SHIFT;  // enable shift
  Nop();
  __delay32(1); // 100ns for 10M TCY
  
  data = PIN_SPI_DATA_IN;
  
  for (i = 0; i < 8; i++) {
    PIN_SPI_CLK_OUT = 1;
    Nop();
    data <<= 1;
    data |= PIN_SPI_DATA_IN;
    PIN_SPI_CLK_OUT = 0;
    Nop();
    __delay32(1); // 100ns for 10M TCY
  }
  
  PIN_PW_SHIFT_OUT = !OLL_PW_SHIFT; // make load active when idle
  Nop();
  
  if (data & 0x0100) {  // counter overflow
    trigger_width = 0xFF;
  } else {
    trigger_width = data & 0xFF;
  }
  trigger_width_filtered = FilterTrigger(trigger_width);
  
  if (trigger_width_filtered < 245) {   //signify to pfn control board what mode to expect
    PIN_MODE_OUT = OLL_MODE_PORTAL;   //so it can use a different target
  } else {                                  //current setpoint for low energy
    PIN_MODE_OUT = OLL_MODE_GANTRY;
  }
}

unsigned char FilterTrigger(unsigned char param) {
  int x;
  
  //Establish Dose Levels to reduce jitter and provide consistent dose vs trigger width
  //Every bit represents 20ns pulse width change on the electron gun
  //Every bit also represents a 200ns pulse width change from the customer
  if (param > (DOSE_LEVELS - 1)) {
    for (x = 0; x <= (param % DOSE_LEVELS); x++)
      param--;
  } else {
    param = 0;
  }
  
  //Ensure that at least 15 of the same width pulses in a row only change the sampled width
  if (param != psb_data.last_trigger_filtered) {
    change_pulse_width_counter++;
    if (change_pulse_width_counter < 15) {
      param = psb_data.last_trigger_filtered;
    } else {
      psb_data.last_trigger_filtered = param;
    }
  } else {
    change_pulse_width_counter = 0;
  }
  
  return param;
}


void ReadAndSetEnergy() {
  /*
    DPARKER - Explain what happens in each mode . . . 
    We should not need the AFC trigger select because the AFC knows if we are high or low
    DPAPKER - Adjust for single energy mode???

  */
  if ((PIN_LOW_MODE_IN == HI) && (PIN_HIGH_MODE_IN == HI)) {
    if (PIN_ENERGY_CMD_IN1 == HI) {
      PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC
      PIN_GUN_POLARITY_OUT = !OLL_GUN_POLARITY;
      PIN_ENERGY_CPU_OUT = !OLL_ENERGY_CPU;
      psb_data.energy = HI;
    } else {
      PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC
      PIN_GUN_POLARITY_OUT = !OLL_GUN_POLARITY;
      PIN_ENERGY_CPU_OUT = OLL_ENERGY_CPU;
      psb_data.energy = LOW;
    }
  } else {
    if (PIN_HIGH_MODE_IN == HI) {
      PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC
      PIN_GUN_POLARITY_OUT = OLL_GUN_POLARITY;
      PIN_ENERGY_CPU_OUT = OLL_ENERGY_CPU;
      psb_data.energy = LOW;
    } else {
      PIN_AFC_TRIGGER_OK_OUT = OLL_AFC_TRIGGER_OK;    //Trigger the AFC
      PIN_GUN_POLARITY_OUT = OLL_GUN_POLARITY;
      PIN_ENERGY_CPU_OUT = !OLL_ENERGY_CPU;
      psb_data.energy = HI;
    }
  }
}


void ProgramShiftRegisters(void) {
  unsigned int p;
  unsigned int q;
  unsigned long temp;
  unsigned long bittemp;
  
  PIN_PW_CLR_CNT_OUT = OLL_PW_CLR_CNT;			 // clear width count
  Nop();
  PIN_PW_HOLD_LOWRESET_OUT = !OLL_PW_HOLD_LOWRESET;	 // reset start to disable pulse
  Nop();
  
  if (psb_data.energy == HI) {
    psb_data.dose_sample_delay    = dose_sample_delay_high;
    psb_data.pfn_delay   = pfn_delay_high;
    psb_data.afc_delay   = afc_delay_high;
    psb_data.magnetron_current_sample_delay = magnetron_current_sample_delay_high;
  } else {
    psb_data.dose_sample_delay    = dose_sample_delay_low;
    psb_data.pfn_delay   = pfn_delay_low;
    psb_data.afc_delay   = afc_delay_low;
    psb_data.magnetron_current_sample_delay = magnetron_current_sample_delay_low;
  }
     
  
    // do inteplation for grid delay and grid width
  for (p = 0; p < 4; p++) {
    if (trigger_width_filtered <= dose_intensities[p]) break;
  }
    
  if (p == 0) {
    if (psb_data.energy == HI) {
      data_grid_start = grid_start_high0;
      data_grid_stop = grid_stop_high0;
    } else {
      data_grid_start = grid_start_low0;
      data_grid_stop = grid_stop_low0;
    }
  } else if (p >= 4) {
    if (psb_data.energy == HI) {
      data_grid_start = grid_start_high3;
      data_grid_stop = grid_stop_high3;
    } else {
      data_grid_start = grid_start_low3;
      data_grid_stop = grid_stop_low3;
    }
  } else { // interpolation
    if (p == 1) {
      if (psb_data.energy == HI) {
	data_grid_start = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_start_high0, grid_start_high1, trigger_width_filtered);
	data_grid_stop = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_stop_high0, grid_stop_high1, trigger_width_filtered);
      } else {
	data_grid_start = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_start_low0, grid_start_low1, trigger_width_filtered);
	data_grid_stop = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_stop_low0, grid_stop_low1, trigger_width_filtered);
      }
    } else if (p == 2) {
      if (psb_data.energy == HI) {
	data_grid_start = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_start_high1, grid_start_high2, trigger_width_filtered);
	data_grid_stop = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_stop_high1, grid_stop_high2, trigger_width_filtered);
      }
      else {
	data_grid_start = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_start_low1, grid_start_low2, trigger_width_filtered);
	data_grid_stop = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_stop_low1, grid_stop_low2, trigger_width_filtered);
      }
    }
    else if (p == 3) {
      if (psb_data.energy == HI) {
	data_grid_start = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_start_high2, grid_start_high3, trigger_width_filtered);
	data_grid_stop = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_stop_high2, grid_stop_high3, trigger_width_filtered);
      } else {
	data_grid_start = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_start_low2, grid_start_low3, trigger_width_filtered);
	data_grid_stop = GetInterpolationValue(dose_intensities[p - 1], dose_intensities[p], grid_stop_low2, grid_stop_low3, trigger_width_filtered);
      }
    }
  }
  
  for (p = 0; p < 6; p++) {
    if (p == 0) {
      temp = data_grid_stop;     //Grid Width
    } else if (p == 1) {
      temp = data_grid_start;     //Grid Delay
    } else if (p == 2) {
      temp = psb_data.dose_sample_delay;       //RF PCB Delay  // This is not the current monitor trigger
    } else if (p == 3) {
      temp = psb_data.pfn_delay;   //PFN Delay
    } else if (p == 4) {
      temp = psb_data.magnetron_current_sample_delay;      //Dosimeter delay (not used)
    } else if (p == 5) {
      temp = psb_data.afc_delay;   //AFC Delay
    } else {
      temp = 0;
    }
    for (q = 0; q < 8; q++) {
      PIN_SPI_CLK_OUT = 0;
      Nop();
      
      bittemp = temp & 0x80;
      temp = temp << 1;
      
      if (bittemp == 0x80) {
	PIN_SPI_DATA_OUT = 1;
	Nop();
      } else {
	PIN_SPI_DATA_OUT = 0;
	Nop();
      }
      
      PIN_SPI_CLK_OUT = 1;
      Nop();
    }
    
    if (p == 1)	{					//Latch Gun delay and width data into shift registers
      PIN_LD_DELAY_GUN_OUT = 0;
      Nop();
      PIN_LD_DELAY_GUN_OUT = 1;
      Nop();
    } else if (p == 3) {				//Latch PFN/RF delay data into shift registers
      PIN_LD_DELAY_PFN_OUT = 0;
      Nop();
      PIN_LD_DELAY_PFN_OUT = 1;
      Nop();
    } else if (p == 5) {				//Latch AFC/Dose delay data into shift registers
      PIN_LD_DELAY_AFC_OUT = 0;
      Nop();
      PIN_LD_DELAY_AFC_OUT = 1;
      Nop();
    }
  }

  PIN_PW_CLR_CNT_OUT = !OLL_PW_CLR_CNT;			 // enable width count
  Nop();
  if (PIN_TRIG_INPUT != ILL_TRIG_ON) {
    PIN_PW_HOLD_LOWRESET_OUT = OLL_PW_HOLD_LOWRESET;   // clear reset only when trig pulse is low
    Nop();
    _FAULT_TRIGGER_STAYED_ON = 0;
  } else {
    _FAULT_TRIGGER_STAYED_ON = 1;
  }
}


// calculate the interpolation value
unsigned int GetInterpolationValue(unsigned int low_point, unsigned int high_point, unsigned low_value, unsigned high_value, unsigned point)
{
   double dtemp, dslope;
   unsigned int ret = low_value;

   if (high_point > low_point)  // high point has to be bigger
   {
   	dslope = ((double)high_value - (double)low_value) / ((double)high_point - (double)low_point);
        dtemp = (double)point - (double)low_point;
        dtemp *= dslope;
        dtemp += low_value;
        ret = (unsigned)dtemp;
   }
   return (ret);
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

      for (i = 0; i < 8; i++)
      {
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
  
  ADPCFG = 0b0000000011111111;
  ADCON1 = 0x0000;
}

#define MIN_PERIOD 150 // 960uS 1041 Hz// 
void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _INT3Interrupt(void) {
  // A trigger was recieved.
  // THIS DOES NOT MEAN THAT A PULSE WAS GENERATED
  // If (PIN_CPU_XRAY_ENABLE_OUT == OLL_CPU_XRAY_ENABLE)  && (PIN_CUSTOMER_XRAY_ON_IN == ILL_CUSTOMER_BEAM_ENABLE) then we "Probably" generated a pulse
  if ((TMR1 > MIN_PERIOD) || _T1IF) {
    // Calculate the Trigger PRF
    // TMR1 is used to time the time between INT3 interrupts
    psb_data.last_period = TMR1;
    TMR1 = 0;
    if (_T1IF) {
      // The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
      psb_data.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
    }
    /*
      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV() || ETMCanSlaveGetSyncMsgPulseSyncDisableXray()) {
      // We are not pulsing so set the PRF to the minimum
      psb_data.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
      }
    */
    _T1IF = 0;
    
    psb_data.trigger_complete = 1;
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
      psb_data.counter_config_received |= 0b0001;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_1:
      *(unsigned int*)&grid_stop_high3 = message_ptr->word2;
      *(unsigned int*)&grid_stop_high1 = message_ptr->word1;
      *(unsigned int*)&magnetron_current_sample_delay_high = message_ptr->word0;
      psb_data.counter_config_received |=0b0010;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_0:
      *(unsigned int*)&grid_start_low3 = message_ptr->word2;
      *(unsigned int*)&grid_start_low1 = message_ptr->word1;
      *(unsigned int*)&dose_sample_delay_low = message_ptr->word0;
      psb_data.counter_config_received |= 0b0100;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_1:
      *(unsigned int*)&grid_stop_low3 = message_ptr->word2;
      *(unsigned int*)&grid_stop_low1 = message_ptr->word1;
      *(unsigned int*)&magnetron_current_sample_delay_low = message_ptr->word0;
      psb_data.counter_config_received |= 0b1000;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_CUSTOMER_LED_OUTPUT:
      psb_data.led_state = message_ptr->word0;
      break;
      
    default:
      ETMCanSlaveIncrementInvalidIndex();
      break;
    }
}
