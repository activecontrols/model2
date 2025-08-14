#include <Arduino.h>
#include "matlab_funcs.hpp"
#include "sample_data.hpp"

void setup() {
  Serial.begin(115200);
  Serial.println("Connected - starting benchmark");

  // remains constant
  Matrix10_18 C = Matrix10_18::Zero();
  C(0, 0) = 1;
  C(1, 1) = 1;
  C(2, 2) = 1;
  C(3, 3) = 1;
  C(4, 4) = 1;
  C(5, 5) = 1;
  C(6, 6) = 1;
  C(7, 9) = 1;
  C(8, 10) = 1;
  C(9, 11) = 1;

  // updated internally by `EstimateState2`
  Matrix18_18 P;
  bool p_init = false;

  long long start_t = millis();

  for (int idx = 0; idx < MAX_IDX; idx++) {
    Vector10 Y(Y_arr[idx]);
    Vector18 X_hat(X_hat_arr[idx]);
    Vector4 U(U_arr[idx]);
    float t = idx * 0.002;

    Vector18 ret_xhat_aug = EstimateState2(Y, X_hat, U, t, C, P, p_init);

    // for (int i = 0; i < 18; i++) {
    //   Serial.print(ret_xhat_aug[i], 4); // 4 digits of precision
    //   Serial.print(" ");
    // }
    // Serial.println();
    // for (int i = 0; i < 18; i++) {
    //   Serial.print(exp_xhat_aug[idx][i], 4); // 4 digits of precision
    //   Serial.print(" ");
    // }
    // Serial.println();
    // Serial.println();

    for (int i = 0; i < 18; i++) {
      if (abs(ret_xhat_aug[i] - exp_xhat_aug[idx][i]) > 0.0002) {
        Serial.print("Error on idx: ");
        Serial.print(idx);
        Serial.print(" subidx: ");
        Serial.print(i);
        Serial.print(" expected: ");
        Serial.print(exp_xhat_aug[idx][i], 4);
        Serial.print(" received ");
        Serial.println(ret_xhat_aug[i], 4);
      }
    }
  }

  long long end_t = millis();

  Serial.print("Finished in ");
  Serial.print(end_t - start_t);
  Serial.println(" ms.");
}

void loop() {
}