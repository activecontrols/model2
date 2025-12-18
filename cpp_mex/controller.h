#pragma once

struct Controller_Output {
  float thrust_N;
  float roll_rad_sec_squared;
  float gimbal_pitch_deg;
  float gimbal_yaw_deg;
};

struct Controller_Input {
  // System Status
  float GND_val;
  bool new_imu_packet;
  bool new_gps_packet;

  // Sensor Inputs
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  float mag_x;
  float mag_y;
  float mag_z;
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;
};

namespace Controller {
void begin();
void reset_controller_state();
Controller_Output get_controller_output(Controller_Input ci);
}; // namespace Controller
