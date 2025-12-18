#include "mex.h"
#include "controller.h"

// MEX gateway function
// cpp_controller_mex_wrap(Vector15 z, Vector3 target, double GND, double dT)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Check number of inputs
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:nrhs", "4 inputs required.");
  }

  // Check number of outputs
  if (nlhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:nlhs", "1 outputs required.");
  }

  if (!mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 15) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:inputSize",
                      "First input must be a 15-element real double vector.");
  }

  if (!mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 3) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:inputSize",
                      "Second input must be a 3-element real double vector.");
  }

  if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:inputSize",
                      "Second input must be a real double.");
  }

  if (!mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != 1) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:inputSize",
                      "Second input must be a real double.");
  }

  Controller_Input ci;
  double *z_vec = mxGetPr(prhs[0]);
  double *target_vec = mxGetPr(prhs[1]);

  ci.accel_x = z_vec[0];
  ci.accel_y = z_vec[1];
  ci.accel_z = z_vec[2];
  ci.gyro_yaw = z_vec[3];
  ci.gyro_pitch = z_vec[4];
  ci.gyro_roll = z_vec[5];
  ci.mag_x = z_vec[6];
  ci.mag_y = z_vec[7];
  ci.mag_z = z_vec[8];
  ci.gps_pos_north = z_vec[9];
  ci.gps_pos_west = z_vec[10];
  ci.gps_pos_up = z_vec[11];
  ci.gps_vel_north = z_vec[12];
  ci.gps_vel_west = z_vec[13];
  ci.gps_vel_up = z_vec[14];

  ci.target_pos_north = target_vec[0];
  ci.target_pos_west = target_vec[1];
  ci.target_pos_up = target_vec[2];

  ci.GND_val = *mxGetPr(prhs[2]);
  ci.dT = *mxGetPr(prhs[3]);

  ci.new_imu_packet = true; // TODO - set these properly
  ci.new_gps_packet = true;

  Controller_Output co = Controller::get_controller_output(ci);

  // Create output variables
  plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);

  double *co_vec = mxGetPr(plhs[0]);
  co_vec[0] = co.gimbal_yaw_deg;
  co_vec[1] = co.gimbal_pitch_deg;
  co_vec[2] = co.thrust_N;
  co_vec[3] = co.roll_rad_sec_squared;
}