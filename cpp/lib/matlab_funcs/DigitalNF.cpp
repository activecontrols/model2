// Digital Sequential Notch Filter implementation for ASTRAv2.
// All IMU sensors are meant to be passed to this array always. Includes a
// GND switch option so the filter does not run on the ground when no
// vibrations are present.

#include "matlab_funcs.hpp"
#include "math.h"

Vector9 FilterStage(Vector9 IN, float f0, float fs, float width, int stage, Matrix9_6 &X, Matrix9_6 &Y) {
  float w0 = 2 * M_PI * f0 / fs;
  float r = exp(-M_PI * width / fs);

  // Calculate Normalization Gain
  float G_num = 1 - 2 * r * cos(w0) + r * r;
  float G_den = 2 - 2 * cos(w0);
  float G = G_num / G_den;

  // Filter Out
  Vector9 NUM = G * (IN - 2 * cos(w0) * X.block<9, 1>(0, 2 * stage) + X.block<9, 1>(0, 2 * stage + 1));
  Vector9 DEN = 2 * r * cos(w0) * Y.block<9, 1>(0, 2 * stage) - r * r * Y.block<9, 1>(0, 2 * stage + 1);
  Vector9 OUT = NUM + DEN;

  // Update memory
  X.block<9, 1>(0, 2 * stage + 1) = X.block<9, 1>(0, 2 * stage);
  X.block<9, 1>(0, 2 * stage) = IN;
  Y.block<9, 1>(0, 2 * stage + 1) = Y.block<9, 1>(0, 2 * stage);
  Y.block<9, 1>(0, 2 * stage) = OUT;

  return OUT;
}

Vector9 DigitalNF(Vector9 IN, float GND, float THRUST, float dT, Matrix9_6 &X, Matrix9_6 &Y) {
  Vector9 OUT;

  Matrix4_4 TRACK;
  TRACK << 1.7054, 50.9375, 3.2139, 117.1451; // TODO - check this insertion order

  float fs = 1 / dT;
  float width1 = 16; // Hz
  float width2 = width1 + 0.7 * (THRUST - 15);

  // Sequential Notch Filter
  if (GND == 0) {
    // Notch #1
    // Setup first notch at constant frequency
    float f0 = 94; // Hz
    OUT = FilterStage(IN, f0, fs, width1, 0, X, Y);

    // Notch #2
    // Setup second notch following track #1
    f0 = TRACK(0, 0) * THRUST + TRACK(0, 1); // Hz
    OUT = FilterStage(IN, f0, fs, width2, 1, X, Y);

    // Notch 3 as a 1st order LPF at 120Hz
    float cut = 35 * 2 * M_PI;
    float C1 = cut / (2 * fs + cut);
    float C2 = (cut - 2 * fs) / (cut + 2 * fs);
    OUT = C1 * IN + C1 * X.block<9, 1>(0, 4) - C2 * Y.block<9, 1>(0, 4);
    Y.block<9, 1>(0, 4) = OUT;
    X.block<9, 1>(0, 4) = IN;
  } else {
    // The output is just the input(passthrough)
    OUT = IN;

    // --- Update states sequentially for a smooth switch-on ---

    // Stage 1: History is updated with IN and OUT
    X.block<9, 1>(0, 1) = X.block<9, 1>(0, 0);
    X.block<9, 1>(0, 0) = IN;
    Y.block<9, 1>(0, 1) = Y.block<9, 1>(0, 0);
    Y.block<9, 1>(0, 0) = OUT;

    // The input to stage 2 is the output of stage 1
    Vector9 IN_2 = OUT;
    Vector9 OUT_2 = IN_2; // Passthrough for stage 2

    // Stage 2: History is updated
    X.block<9, 1>(0, 3) = X.block<9, 1>(0, 2);
    X.block<9, 1>(0, 2) = IN_2;
    Y.block<9, 1>(0, 3) = Y.block<9, 1>(0, 2);
    Y.block<9, 1>(0, 2) = OUT_2;

    // The input to stage 3 is the output of stage 2
    Vector9 IN_3 = OUT_2;
    Vector9 OUT_3 = IN_3; // Passthrough for stage 3

    // Stage 3: History is updated
    X.block<9, 1>(0, 5) = X.block<9, 1>(0, 4);
    X.block<9, 1>(0, 4) = IN_3;
    Y.block<9, 1>(0, 5) = Y.block<9, 1>(0, 4);
    Y.block<9, 1>(0, 4) = OUT_3;

    // The final output is the passthrough from the last stage
    OUT = OUT_3;
  }

  return OUT;
}