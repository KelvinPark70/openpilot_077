#define ADDR_MDPS11             897
#define ADDR_MDPS12             593

#define ADDR_E_EMS11            881
#define ADDR_EMS11              790
#define ADDR_EMS16              608

#define ADDR_LKAS11             832
#define ADDR_WHL_SPD11          902
#define ADDR_TCS13              916
#define ADDR_LFAHDA_MFC         1157
#define ADDR_CLU11              1265

#define ADDR_SCC11              1056
#define ADDR_SCC12              1057
#define ADDR_SCC13              1290
#define ADDR_SCC14              905


const int HYUNDAI_MAX_STEER = 255;             // like stock
const int HYUNDAI_MAX_RT_DELTA = 112;          // max delta torque allowed for real time checks
const uint32_t HYUNDAI_RT_INTERVAL = 250000;   // 250ms between real time checks
const int HYUNDAI_MAX_RATE_UP = 3;
const int HYUNDAI_MAX_RATE_DOWN = 7;
const int HYUNDAI_DRIVER_TORQUE_ALLOWANCE = 50;
const int HYUNDAI_DRIVER_TORQUE_FACTOR = 2;
const int HYUNDAI_STANDSTILL_THRSLD = 30;  // ~1kph
const CanMsg HYUNDAI_TX_MSGS[] = {
  {ADDR_LKAS11, 0, 8}, //{ADDR_LKAS11, 1, 8}, // LKAS11 Bus 0,1
  {ADDR_CLU11, 0, 4}, //{ADDR_CLU11, 1, 4}, {ADDR_CLU11, 2, 4},  // CLU11 Bus 0
 // {ADDR_LFAHDA_MFC, 0, 4}, // LFAHDA_MFC Bus 0
  {ADDR_MDPS12, 2, 8},  // MDPS12, Bus 2
  // {1056, 0, 8}, //   SCC11,  Bus 0
  // {1057, 0, 8}, //   SCC12,  Bus 0
  // {1290, 0, 8}, //   SCC13,  Bus 0
  // {905, 0, 8},  //   SCC14,  Bus 0
  // {1186, 0, 8}  //   4a2SCC, Bus 0
 };

// TODO: missing checksum for wheel speeds message,worst failure case is
//       wheel speeds stuck at 0 and we don't disengage on brake press
AddrCheckStruct hyundai_rx_checks[] = {
  //{.msg = {{ADDR_EMS16, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U}}},
  {.msg = {{ADDR_WHL_SPD11, 0, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 10000U}}},
  {.msg = {{ADDR_TCS13, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}}},
  {.msg = {{ADDR_SCC12, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_RX_CHECK_LEN = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]);

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_rx_checks[] = {
  {.msg = {{ADDR_EMS16, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {ADDR_E_EMS11, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{ADDR_WHL_SPD11, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{ADDR_TCS13, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{ADDR_SCC12, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_LEGACY_RX_CHECK_LEN = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]);

bool hyundai_legacy = false;

static uint8_t hyundai_get_counter(CAN_FIFOMailBox_TypeDef *to_push) 
{
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == ADDR_EMS16) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3;
  } else if (addr == ADDR_WHL_SPD11) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == ADDR_TCS13) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
  } else if (addr == ADDR_SCC12) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint8_t hyundai_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) 
{
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == ADDR_EMS16) {
    chksum = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == ADDR_TCS13) {
    chksum = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == ADDR_SCC12) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

static uint8_t hyundai_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) 
{
  int addr = GET_ADDR(to_push);

  uint8_t chksum = 0;
  // same algorithm, but checksum is in a different place
  for (int i = 0; i < 8; i++) 
  {
    uint8_t b = GET_BYTE(to_push, i);
    if (((addr == ADDR_EMS16) && (i == 7)) || ((addr == ADDR_TCS13) && (i == 6)) || ((addr == ADDR_SCC12) && (i == 7))) 
    {
      b &= (addr == ADDR_SCC12) ? 0x0FU : 0xF0U; // remove checksum
    }
    chksum += (b % 16U) + (b / 16U);
  }
  return (16U - (chksum %  16U)) % 16U;
}

bool hyundai_has_scc = false;
int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int OP_EMS_live = 0;
int hyundai_mdps_bus = 0;
bool hyundai_LCAN_on_bus1 = false;
bool hyundai_forward_bus1 = false;


static int hyundai_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) 
{
  bool valid = false;
  if( hyundai_legacy )
  {
    valid = addr_safety_check(to_push, hyundai_legacy_rx_checks, HYUNDAI_LEGACY_RX_CHECK_LEN,
                              hyundai_get_checksum, hyundai_compute_checksum,
                              hyundai_get_counter);
  } else {
    valid = addr_safety_check(to_push, hyundai_rx_checks, HYUNDAI_RX_CHECK_LEN,
                              hyundai_get_checksum, hyundai_compute_checksum,
                              hyundai_get_counter);
  }

  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);


  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 1296 || addr == 524)) 
  {
    if (hyundai_forward_bus1 || !hyundai_LCAN_on_bus1) 
    {
      hyundai_LCAN_on_bus1 = true;
      hyundai_forward_bus1 = false;
    }
  }
  // check if we have a MDPS on Bus1 and LCAN not on the bus
  if (bus == 1 && (addr == 593 || addr == 897) && !hyundai_LCAN_on_bus1) 
  {
    if (hyundai_mdps_bus != bus || !hyundai_forward_bus1) 
    {
      hyundai_mdps_bus = bus;
      hyundai_forward_bus1 = true;
    }
  }
  // check if we have a SCC on Bus1 and LCAN not on the bus
  if (bus == 1 && addr == 1057 && !hyundai_LCAN_on_bus1) 
  {
    if (!hyundai_forward_bus1) 
    {
      hyundai_forward_bus1 = true;
    }
  }

    if( valid == 0 ) return valid;  

    if( addr == ADDR_MDPS12 && bus == hyundai_mdps_bus ) {   // MDPS12
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if( OP_SCC_live && addr == ADDR_SCC12 && (bus != 1 || !hyundai_LCAN_on_bus1) )  // SCC12
    {   
      hyundai_has_scc = true;
      car_SCC_live = 50;
      // 2 bits: 13-14
      int cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3;
      if(cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    if( !OP_SCC_live && addr == ADDR_SCC11 && (bus != 1 || !hyundai_LCAN_on_bus1) ) 
    {
       hyundai_has_scc = true;
      // for cars without long control
      // 2 bits: 13-14
      int cruise_engaged = GET_BYTES_04(to_push) & 0x1; // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // cruise control for car without SCC
    if (addr == 608 && bus == 0 && !hyundai_has_scc && !OP_SCC_live) {
      // bit 25
      int cruise_engaged = (GET_BYTES_04(to_push) >> 25 & 0x1); // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    // engage for Cruise control disabled car
    if (addr == 1265 && bus == 0 && OP_SCC_live && !car_SCC_live) {
      // first byte
      int cruise_button = (GET_BYTES_04(to_push) & 0x7);
      // enable on both accel and decel buttons falling edge
      if (!cruise_button && (cruise_engaged_prev == 1 || cruise_engaged_prev == 2)) {
        controls_allowed = 1;
      }
      // disable on cancel rising edge
      if (cruise_button == 4) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_button;
    }



    // TODO: check gas pressed
    // exit controls on rising edge of gas press for cars with long control
    if (addr == ADDR_EMS16 && OP_SCC_live && bus == 0) 
    {
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0;
      if (!unsafe_allow_gas && gas_pressed && !gas_pressed_prev) {
        controls_allowed = 0;
      }
      gas_pressed_prev = gas_pressed;
    }

    // sample subaru wheel speed, averaging opposite corners
    if (addr == ADDR_WHL_SPD11 && bus == 0) 
    {
      int hyundai_speed = GET_BYTES_04(to_push) & 0x3FFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 16) & 0x3FFF;  // RL
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    // exit controls on rising edge of brake press for cars with long control
    if (addr == ADDR_TCS13 && OP_SCC_live && bus == 0) 
    {
      brake_pressed = (GET_BYTE(to_push, 6) >> 7) != 0;
      if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
        controls_allowed = 0;
      }
      brake_pressed_prev = brake_pressed;
    }

    // check if stock camera ECU is on bus 0
    if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && bus == 0 && addr == 832) 
    {
      relay_malfunction_set();
    }




    /*
    // TODO: check gas pressed
    // exit controls on rising edge of gas press for cars with long control
    if( (OP_SCC_live && bus == 0) && (addr == ADDR_EMS16 || (hyundai_legacy && (addr == ADDR_E_EMS11))) )  // 608:EMS16 , 881:E_EMS11   
    {   
      if (addr == ADDR_EMS16) {
        gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0;
      } else {
        gas_pressed = (((GET_BYTE(to_push, 4) & 0x7F) << 1) | GET_BYTE(to_push, 3) >> 7) != 0;
      }
    }

    // sample wheel speed, averaging opposite corners
    if( bus == 0 && addr == ADDR_WHL_SPD11 )   // WHL_SPD11
    {  
      int hyundai_speed = GET_BYTES_04(to_push) & 0x3FFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 16) & 0x3FFF;  // RL
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    if (OP_SCC_live && bus == 0 && addr == ADDR_TCS13) {   // TCS13
      brake_pressed = (GET_BYTE(to_push, 6) >> 7) != 0;
    }

    if( bus == 0 )
    {
      generic_rx_checks( (addr == ADDR_LKAS11) );  
    }
    */



  return valid;
}

static int hyundai_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, HYUNDAI_TX_MSGS, sizeof(HYUNDAI_TX_MSGS)/sizeof(HYUNDAI_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // LKA STEER: safety check
  if (addr == ADDR_LKAS11) {  //LKAS11
    OP_LKAS_live = 20;  
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (!controls_allowed) { // a reset worsen the issue of Panda blocking some valid LKAS messages
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if((addr == ADDR_CLU11) && !controls_allowed && (bus != hyundai_mdps_bus || !hyundai_mdps_bus) ) {
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  // add  program
  if (addr == ADDR_MDPS12) {OP_MDPS_live = 20;}
  if (addr == ADDR_CLU11 && bus == 1) {OP_CLU_live = 20;} // check if OP create clu11 for MDPS
  if (addr == ADDR_SCC12) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;} }
  if (addr == ADDR_EMS11) {OP_EMS_live = 20;}  
  // 1 allows the message through
  return tx;
}

static int hyundai_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  int fwd_to_bus1 = -1;
  if (hyundai_forward_bus1) {fwd_to_bus1 = 1;}

  // forward cam to ccan and viceversa, except lkas cmd
  if (!relay_malfunction) {
    if (bus_num == 0) 
    {
      if (!OP_CLU_live || addr != ADDR_CLU11 || !hyundai_mdps_bus) {
        if (!OP_MDPS_live || addr != ADDR_MDPS12) {
          if (!OP_EMS_live || addr != ADDR_EMS11) {
            bus_fwd = hyundai_forward_bus1 ? 12 : 2;
          } else {
            bus_fwd = 2;  // EON create EMS11 for MDPS
            OP_EMS_live -= 1;
          }
        } else {
          bus_fwd = fwd_to_bus1;  // EON create MDPS for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = 2; // EON create CLU12 for MDPS
        OP_CLU_live -= 1;
      }
    }
    else if (bus_num == 1 && hyundai_forward_bus1) 
    {
      if (!OP_MDPS_live || addr != ADDR_MDPS12) {
        if (!OP_SCC_live || (addr != ADDR_SCC11 && addr != ADDR_SCC12 && addr != ADDR_SCC13 && addr != ADDR_SCC14)) {
          bus_fwd = 20;
        } else {
          bus_fwd = 2;  // EON create SCC11 SCC12 SCC13 SCC14 for Car
          OP_SCC_live -= 1;
        }
      } else {
        bus_fwd = 0;  // EON create MDPS for LKAS
        OP_MDPS_live -= 1;
      }
    }
    else if (bus_num == 2) 
    {
      if (!OP_LKAS_live || (addr != ADDR_LKAS11 || addr != ADDR_LFAHDA_MFC) ) 
      {
        if (!OP_SCC_live || (addr != ADDR_SCC11 && addr != ADDR_SCC12 && addr != ADDR_SCC13 && addr != ADDR_SCC14 )) 
        {
          bus_fwd = hyundai_forward_bus1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // EON create SCC12 for Car
          OP_SCC_live -= 1;
        }
      } 
      else if ( !hyundai_mdps_bus ) 
      {
        bus_fwd = fwd_to_bus1; // EON create LKAS and LFA for Car
        OP_LKAS_live -= 1; 
      } else {
        OP_LKAS_live -= 1; // EON create LKAS and LFA for Car and MDPS
      }
    }

  }
  else 
  {
    if (bus_num == 0) {
      bus_fwd = fwd_to_bus1;
    }
    else if (bus_num == 1 && hyundai_forward_bus1) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static void hyundai_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = false;
}

static void hyundai_legacy_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = true;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
  .addr_check = hyundai_rx_checks,
  .addr_check_len = sizeof(hyundai_rx_checks) / sizeof(hyundai_rx_checks[0]),
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
  .addr_check = hyundai_legacy_rx_checks,
  .addr_check_len = sizeof(hyundai_legacy_rx_checks) / sizeof(hyundai_legacy_rx_checks[0]),
};
