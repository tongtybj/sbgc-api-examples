/* 
   SimpleBGC Serial API  library - helpers to pack and parse command data
   More info: http://www.basecamelectronics.com/serialapi/

   Copyright (c) 2014-2015 Aleksei Moskalenko
   All rights reserved.
	
   See license info in the SBGC.h
*/   

#include <string.h>
#include "SBGC.h"

/* Packs command structure to SerialCommand object */
void SBGC_cmd_control_pack(SBGC_cmd_control_t &p, SerialCommand &cmd) {
  cmd.init(SBGC_CMD_CONTROL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
  memcpy(cmd.data, &p, sizeof(p));
  cmd.len = sizeof(p);
#else
  cmd.writeByte(p.mode);
  cmd.writeWord(p.speedROLL);
  cmd.writeWord(p.angleROLL);
  cmd.writeWord(p.speedPITCH);
  cmd.writeWord(p.anglePITCH);
  cmd.writeWord(p.speedYAW);
  cmd.writeWord(p.angleYAW);
#endif
}


/* Packs command structure to SerialCommand object */
void SBGC_cmd_control_ext_pack(SBGC_cmd_control_ext_t &p, SerialCommand &cmd) {
  cmd.init(SBGC_CMD_CONTROL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
  memcpy(cmd.data, &p, sizeof(p));
  cmd.len = sizeof(p);
#else
  cmd.writeBuf(p.mode, 3);
  for(uint8_t i=0; i<3; i++) {
    cmd.writeWord(p.data[i].speed);
    cmd.writeWord(p.data[i].angle);
  }
#endif
}





/* Packs command structure to SerialCommand object */
void SBGC_cmd_api_virt_ch_control_pack(SBGC_cmd_api_virt_ch_control_t &p, SerialCommand &cmd) {
  cmd.init(SBGC_CMD_API_VIRT_CH_CONTROL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
  memcpy(cmd.data, &p, sizeof(p));
  cmd.len = sizeof(p);
#else
  for(uint8_t i=0; i<SBGC_API_VIRT_NUM_CHANNELS; i++) {
    cmd.writeByte(p.data[i]);
  }
#endif
}






/* Packs command structure to SerialCommand object */
void SBGC_cmd_trigger_pack(SBGC_cmd_trigger_t &p, SerialCommand &cmd) {
  cmd.init(SBGC_CMD_TRIGGER_PIN);
#ifdef SBGC_CMD_STRUCT_ALIGNED
  memcpy(cmd.data, &p, sizeof(p));
  cmd.len = sizeof(p);
#else
  cmd.writeByte(p.pin);
  cmd.writeByte(p.state);
#endif
}






/* Packs command structure to SerialCommand object */
void SBGC_cmd_servo_out_pack(SBGC_cmd_servo_out_t &p, SerialCommand &cmd) {
  cmd.init(SBGC_CMD_SERVO_OUT);
#ifdef SBGC_CMD_STRUCT_ALIGNED
  memcpy(cmd.data, &p, sizeof(p));
  cmd.len = sizeof(p);
#else
  for(uint8_t i=0; i<8; i++) {
    cmd.writeWord(p.servo[i]);
  }
#endif
}





/* Packs command structure to SerialCommand object */
void SBGC_cmd_set_adj_vars_pack(SBGC_cmd_set_adj_vars_var_t vars[], uint8_t vars_num, SerialCommand &cmd) {
  cmd.init(SBGC_CMD_SET_ADJ_VARS_VAL);
  cmd.writeByte(vars_num); // number of variables
	
#ifdef SBGC_CMD_STRUCT_ALIGNED
  cmd.writeBuf(vars, sizeof(SBGC_cmd_set_adj_vars_var_t)*vars_num);
#else
  for(uint8_t i=0; i<vars_num; i++) {
    cmd.writeByte(vars[i].id);
    cmd.writeLong(vars[i].val);
  }
#endif
}

/*
 * Unpacks SerialCommand object to vars_buf[var_num].
 * 'var_num' specifies the buffer capacity.
 * On return, 'var_num' will be set to actual number of received variables.
 * Returns 0 on success, PARSER_ERROR_XX code on fail.
 */
uint8_t SBGC_cmd_set_adj_vars_unpack(SBGC_cmd_set_adj_vars_var_t vars_buf[], uint8_t &vars_num, SerialCommand &cmd) {
  uint8_t num = cmd.readByte(); // actual number of variables
  if(num <= vars_num) {
    vars_num = num;
#ifdef SBGC_CMD_STRUCT_ALIGNED
    cmd.readBuf(vars_buf, sizeof(SBGC_cmd_set_adj_vars_var_t)*vars_num);
#else
    for(uint8_t i=0; i<num; i++) {
      vars_buf[i].id = cmd.readByte();
      vars_buf[i].val = cmd.readLong();
    }
#endif
		
    if(cmd.checkLimit()) return 0;
    else return PARSER_ERROR_WRONG_DATA_SIZE;
  } else {
    return PARSER_ERROR_BUFFER_IS_FULL;
  }
}




/*
 * Unpacks SerialCommand object to command structure.
 * Returns 0 on success, PARSER_ERROR_XX code on fail.
 */
uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd) {
#ifdef SBGC_CMD_STRUCT_ALIGNED
  if(cmd.len <= sizeof(p)) {
    memcpy(&p, cmd.data, cmd.len);
    return 0;
  } else {
    return PARSER_ERROR_WRONG_DATA_SIZE;
  }
#else
  for(uint8_t i=0; i<3; i++) {
    p.sensor_data[i].acc_data = cmd.readWord();
    p.sensor_data[i].gyro_data = cmd.readWord();
  }
  p.serial_error_cnt = cmd.readWord();
  p.system_error = cmd.readWord();
  cmd.skipBytes(4); // reserved
  cmd.readWordArr(p.rc_raw_data, SBGC_RC_NUM_CHANNELS);
  cmd.readWordArr(p.imu_angle, 3);
  cmd.readWordArr(p.frame_imu_angle, 3);
  cmd.readWordArr(p.target_angle, 3);
  p.cycle_time_us = cmd.readWord();
  p.i2c_error_count = cmd.readWord();
  cmd.readByte(); // reserved
  p.battery_voltage = cmd.readWord();
  p.state_flags1 = cmd.readByte();
  p.cur_imu = cmd.readByte();
  p.cur_profile = cmd.readByte();
  cmd.readBuf(p.motor_power, 3);
		
  if(cmd.id == SBGC_CMD_REALTIME_DATA_4) {
    cmd.readWordArr(p.rotor_angle, 3);
    cmd.readByte(); // reserved
    cmd.readWordArr(p.balance_error, 3);
    p.current = cmd.readWord();
    cmd.readWordArr(p.magnetometer_data, 3);
    p.imu_temp_celcius = cmd.readByte();
    p.frame_imu_temp_celcius = cmd.readByte();
    cmd.skipBytes(38);
  }
		
		
  if(cmd.checkLimit()) return 0;
  else return PARSER_ERROR_WRONG_DATA_SIZE;
#endif
}

uint8_t SBGC_cmd_board_info_unpack(SBGC_cmd_board_info_t &p, SerialCommand &cmd)
{
#ifdef SBGC_CMD_STRUCT_ALIGNED
  if(cmd.len <= sizeof(p)) {
    memcpy(&p, cmd.data, cmd.len);
    return 0;
  } else {
    return PARSER_ERROR_WRONG_DATA_SIZE;
  }
#else
  p.board_ver = cmd.readByte();
  p.firmware_ver = cmd.readWord();
  p.debug_mode = cmd.readByte();
  p.board_features = cmd.readWord();
  p.connection_flags = cmd.readByte();
  cmd.skipBytes(11); // reserved

  if(cmd.checkLimit()) return 0;
  else return PARSER_ERROR_WRONG_DATA_SIZE;

#endif
}


uint8_t SBGC_cmd_get_angle_unpack(SBGC_cmd_get_angles_t &p, SerialCommand &cmd)
{
#ifdef SBGC_CMD_STRUCT_ALIGNED
  if(cmd.len <= sizeof(p)) {
    memcpy(&p, cmd.data, cmd.len);
    return 0;
  } else {
    return PARSER_ERROR_WRONG_DATA_SIZE;
  }
#else
  for(int i = 0; i < 3; i++)
    {
      p.angle_data[i].angle = cmd.readWord();
      p.angle_data[i].rc_angle = cmd.readWord();
      p.angle_data[i].rc_speed = cmd.readWord();
    }
  if(cmd.checkLimit()) return 0;
  else return PARSER_ERROR_WRONG_DATA_SIZE;

#endif
}

//
uint8_t SBGC_cmd_realtime_data_v1_unpack(SBGC_cmd_realtime_data_v1_t &p, SerialCommand &cmd) {
#ifdef SBGC_CMD_STRUCT_ALIGNED
  if(cmd.len <= sizeof(p)) {
    memcpy(&p, cmd.data, cmd.len);
    return 0;
  } else {
    return PARSER_ERROR_WRONG_DATA_SIZE;
  }
#else
  for(uint8_t i=0; i<3; i++) {
    p.sensor_data[i].acc_data = cmd.readWord();
    p.sensor_data[i].gyro_data = cmd.readWord();
  }
  p.serial_error_cnt = cmd.readWord();
  p.system_error = cmd.readWord();
  cmd.skipBytes(4); // reserved
  cmd.readWordArr(p.rc_raw_data, SBGC_RC_NUM_CHANNELS);
  cmd.readWordArr(p.imu_angle, 3);
  cmd.readWordArr(p.rc_angle, 3);
  p.cycle_time_us = cmd.readWord();
  p.i2c_error_count = cmd.readWord();
  p.error_code = cmd.readByte();
  p.battery_voltage = cmd.readWord();
  p.other_flags = cmd.readByte();
  p.cur_profile = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    p.motor_power[i] = cmd.readByte();
  if(cmd.checkLimit()) return 0;
  else return PARSER_ERROR_WRONG_DATA_SIZE;
#endif
}

uint8_t SBGC_cmd_param_unpack(SBGC_cmd_param_t &p, SerialCommand &cmd)
{
#ifdef SBGC_CMD_STRUCT_ALIGNED
  if(cmd.len <= sizeof(p)) {
    memcpy(&p, cmd.data, cmd.len);
    return 0;
  } else {
    return PARSER_ERROR_WRONG_DATA_SIZE;
  }
#else
  p.profile_id = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    {
      p.base_param[i].p = cmd.readByte();
      p.base_param[i].i = cmd.readByte();
      p.base_param[i].d = cmd.readByte();
      p.base_param[i].power = cmd.readByte();
      p.base_param[i].invert = cmd.readByte();
      p.base_param[i].poles = cmd.readByte();
    }
  p.acc_limiter = cmd.readByte();
  for(uint8_t i=0; i<2; i++)
    p.ext_fc_gain[i] = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    {
      p.rc_param[i].rc_min_angle = cmd.readWord();
      p.rc_param[i].rc_max_angle = cmd.readWord();
      p.rc_param[i].rc_mode = cmd.readByte();
      p.rc_param[i].rc_lpf = cmd.readByte();
      p.rc_param[i].rc_speed = cmd.readByte();
      p.rc_param[i].rc_follow = cmd.readByte();
    }
  p.gyro_trust = cmd.readByte();
  p.use_model = cmd.readByte();
  p.pwm_freq = cmd.readByte();
  p.serial_speed = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    p.rc_trim[i] = cmd.readByte();
  p.rc_deadband = cmd.readByte();
  p.rc_expo_rate = cmd.readByte();
  p.rc_virt_mode = cmd.readByte();
  for(uint8_t i=0; i<6; i++)
    p.rc_map[i] = cmd.readByte();
  for(uint8_t i=0; i<2; i++)
    p.rc_mix_fc[i] = cmd.readByte();
  p.follow_mode = cmd.readByte();
  p.follow_deadband = cmd.readByte();
  p.follow_expo_rate = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    p.follow_offset[i] = cmd.readByte();
  p.axis_top = cmd.readByte();
  p.axis_right = cmd.readByte();
  p.gyro_lpf = cmd.readByte();
  p.gyro_sens = cmd.readByte();
  p.i2c_internal_pullups = cmd.readByte();
  p.skip_gyro_calib = cmd.readByte();
  p.rc_cmd_low = cmd.readByte();
  p.rc_cmd_mid = cmd.readByte();
  p.rc_cmd_high = cmd.readByte();
  for(uint8_t i=0; i<5; i++)
    p.menu_cmd[i] = cmd.readByte();
  p.menu_cmd_long = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    p.output[i] = cmd.readByte();
  p.bat_thresh_alarm = cmd.readWord();
  p.bat_thresh_motors = cmd.readWord();
  p.bat_comp_ref = cmd.readWord();
  p.beeper_modes = cmd.readByte();
  p.follow_roll_mix_start = cmd.readByte();
  p.follow_roll_mix_range = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    p.booster_power[i] = cmd.readByte();
  for(uint8_t i=0; i<3; i++)
    p.follow_speed[i] = cmd.readByte();
  p.frame_angle_from_motors = cmd.readByte();
  p.cur_profile_id = cmd.readByte();
#endif

  if(cmd.checkLimit()) return 0;
  else return PARSER_ERROR_WRONG_DATA_SIZE;
}

uint8_t SBGC_cmd_param_pack(SBGC_cmd_param_t &p, SerialCommand &cmd)
{
  cmd.init(SBGC_CMD_WRITE_PARAMS);
#ifdef SBGC_CMD_STRUCT_ALIGNED
  memcpy(cmd.data, &p, sizeof(p));
  cmd.len = sizeof(p);
#else
  cmd.writeByte(p.profile_id);
  for(uint8_t i=0; i<3; i++)
    {
      cmd.writeByte(p.base_param[i].p);
      cmd.writeByte(p.base_param[i].i);
      cmd.writeByte(p.base_param[i].d);
      cmd.writeByte(p.base_param[i].power);
      cmd.writeByte(p.base_param[i].invert);
      cmd.writeByte(p.base_param[i].poles);
    }
  cmd.writeByte(p.acc_limiter);
  for(uint8_t i=0; i<2; i++)
    cmd.writeByte(p.ext_fc_gain[i]);
  for(uint8_t i=0; i<3; i++)
    {
      cmd.writeWord(p.rc_param[i].rc_min_angle);
      cmd.writeWord(p.rc_param[i].rc_max_angle);
      cmd.writeByte(p.rc_param[i].rc_mode);
      cmd.writeByte(p.rc_param[i].rc_lpf);
      cmd.writeByte(p.rc_param[i].rc_speed);
      cmd.writeByte(p.rc_param[i].rc_follow);
    }
  cmd.writeByte(p.gyro_trust);
  cmd.writeByte(p.use_model);
  cmd.writeByte(p.pwm_freq);
  cmd.writeByte(p.serial_speed);
  for(uint8_t i=0; i<3; i++)
    cmd.writeByte(p.rc_trim[i]);
  cmd.writeByte(p.rc_deadband);
  cmd.writeByte(p.rc_expo_rate);
  cmd.writeByte(p.rc_virt_mode);
  for(uint8_t i=0; i<6; i++)
    cmd.writeByte(p.rc_map[i]);
  for(uint8_t i=0; i<2; i++)
    cmd.writeByte(p.rc_mix_fc[i]);
  cmd.writeByte(p.follow_mode);
  cmd.writeByte(p.follow_deadband);
  cmd.writeByte(p.follow_expo_rate);
  for(uint8_t i=0; i<3; i++)
    cmd.writeByte(p.follow_offset[i]);
  cmd.writeByte(p.axis_top);
  cmd.writeByte(p.axis_right);
  cmd.writeByte(p.gyro_lpf);
  cmd.writeByte(p.gyro_sens);
  cmd.writeByte(p.i2c_internal_pullups);
  cmd.writeByte(p.skip_gyro_calib);
  cmd.writeByte(p.rc_cmd_low);
  cmd.writeByte(p.rc_cmd_mid);
  cmd.writeByte(p.rc_cmd_high);
  for(uint8_t i=0; i<5; i++)
    cmd.writeByte(p.menu_cmd[i]);
  cmd.writeByte(p.menu_cmd_long);
  for(uint8_t i=0; i<3; i++)
    cmd.writeByte(p.output[i]);
  cmd.writeWord(p.bat_thresh_alarm);
  cmd.writeWord(p.bat_thresh_motors);
  cmd.writeWord(p.bat_comp_ref);
  cmd.writeByte(p.beeper_modes);
  cmd.writeByte(p.follow_roll_mix_start);
  cmd.writeByte(p.follow_roll_mix_range);
  for(uint8_t i=0; i<3; i++)
    cmd.writeByte(p.booster_power[i]);
  for(uint8_t i=0; i<3; i++)
    cmd.writeByte(p.follow_speed[i]);
  cmd.writeByte(p.frame_angle_from_motors);
  cmd.writeByte(p.cur_profile_id);
#endif
}
