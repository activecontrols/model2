#include <Arduino.h>
#include "matlab_funcs.hpp"

void setup() {
  Serial.begin(115200);
  Serial.println("Connected - starting benchmark");

  long long start_t = millis();

  for (int i = 0; i < 1000; i++) {
    Vector10 Y;
    Y << 0, 0, 0, 0, 0, 0, -0.0434, 0.0331, 0.0059, -0.0754;
    Vector18 X_hat;
    X_hat << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0200, 0.0200, 0.0200, 0.0200, 0.0200, 0.0200;
    Vector4 U;
    U << 0, 0, 5.88000000000000, 0;
    float t = 0;
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

    Vector18 ret_xhat_aug = EstimateState2(Y, X_hat, U, t, C);

    // for (int i = 0; i < 18; i++) {
    //   Serial.print(ret_xhat_aug[i], 4); // 4 digits of precision
    //   Serial.print(" ");
    // }
    // Serial.println();
  }

  long long end_t = millis();

  Serial.print("Finished in ");
  Serial.print(end_t - start_t);
  Serial.println(" ms.");
}

void loop() {
}