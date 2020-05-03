// (c) Sergey Staroletov
// Cruise control using Takagi-Sugeno
// Based on idea of Fuzzy Logic Library for Microsoft .NET (C) 2008 Dmitry
// Kaluzhny

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <thread>

using namespace std;

double TriangleGetValue(double x1, double x2, double x3, double x) {
  double result = 0;
  if (x == x1 && x == x2) {
    result = 1.0;
  } else if (x == x2 && x == x3) {
    result = 1.0;
  } else if (x <= x1 || x >= x3) {
    result = 0;
  } else if (x == x2) {
    result = 1;
  } else if ((x > x1) && (x < x2)) {
    result = (x / (x2 - x1)) - (x1 / (x2 - x1));
  } else {
    result = (-x / (x3 - x2)) + (x3 / (x3 - x2));
  }
  return result;
}

void CarAndControl() {
  double m = 1355 + 72 + 10;  // vehicle mass
  double sigma = 1.04;        // asfphalt
  double g = 9.81;
  double k = 0.20;
  double he = 1.435;
  double we = 1.780;
  double f0 = 0.01;
  double f;

  double v0 = 50 / 3.6;       // initial speed
  double speed_r = 50 / 3.6;  // desired speed

  double t = 0;
  double dt = 0.1;

  double v = v0;

  double a = 0;
  double v_old = v;
  double angle = std::atan(0.01);  // slope, %

  int i = 0;
  double a_old = 0;
  while (true) {
    // physics
    double Pj = 0;  //+sigma * (a - a_old) * m; //inertia - IDK
    double Pi = +m * g * std::sin(angle);
    double Pw = -k * 0.8 * we * he * v * v;
    if (v <= 50)
      f = f0;
    else
      f = f0 * (1 + 0.01 * (50 - v));

    double Pf = -f * m * g;
    double F = (Pi + Pj + Pw + Pf) / m;
    a_old = a;
    a = F;
    v_old = v;

    //--- Sugeno Control

    double SpeedError = v - speed_r;
    double SpeedErrorDot = (v - speed_r) / dt;  // a;

    // Fuzzification step

    double FSpeedErrorSlower = TriangleGetValue(-35.0, -20.0, -5.0, SpeedError);
    double FSpeedErrorZero = TriangleGetValue(-15.0, -0.0, 15.0, SpeedError);
    double FSpeedErrorFaster = TriangleGetValue(5.0, 20.0, 35.0, SpeedError);

    double FSpeedErrorDotSlower =
        TriangleGetValue(-9.0, -5.0, -1.0, SpeedErrorDot);
    double FSpeedErrorDotZero =
        TriangleGetValue(-4.0, -0.0, 4.0, SpeedErrorDot);
    double FSpeedErrorDotFaster =
        TriangleGetValue(5.0, 20.0, 35.0, SpeedErrorDot);

    // Evaluate the conditions
    double ruleWeights0 = std::min(FSpeedErrorSlower, FSpeedErrorDotSlower);
    double ruleWeights1 = std::min(FSpeedErrorSlower, FSpeedErrorDotZero);
    double ruleWeights2 = std::min(FSpeedErrorSlower, FSpeedErrorDotFaster);
    double ruleWeights3 = std::min(FSpeedErrorZero, FSpeedErrorDotSlower);
    double ruleWeights4 = std::min(FSpeedErrorZero, FSpeedErrorDotZero);
    double ruleWeights5 = std::min(FSpeedErrorZero, FSpeedErrorDotFaster);
    double ruleWeights6 = std::min(FSpeedErrorFaster, FSpeedErrorDotSlower);
    double ruleWeights7 = std::min(FSpeedErrorFaster, FSpeedErrorDotZero);
    double ruleWeights8 = std::min(FSpeedErrorFaster, FSpeedErrorDotFaster);

    // Functions evaluation
    double zeroResult = 0 * SpeedError + 0 * SpeedErrorDot + 0;
    double fasterResult = 0 * SpeedError + 0 * SpeedErrorDot + 1;
    double slowerResult = 0 * SpeedError + 0 * SpeedErrorDot - 1;
    double funcResult = -0.04 * SpeedError - 0.1 * SpeedErrorDot + 0;

    // Combine output
    double numerator = 0;
    double denomerator = 0;

    numerator += fasterResult * ruleWeights0 + fasterResult * ruleWeights1 +
                 zeroResult * ruleWeights2 + fasterResult * ruleWeights3 +
                 funcResult * ruleWeights4 + slowerResult * ruleWeights5 +
                 zeroResult * ruleWeights6 + slowerResult * ruleWeights7 +
                 slowerResult * ruleWeights8;
    denomerator = ruleWeights0 + ruleWeights1 + ruleWeights2 + ruleWeights3 +
                  ruleWeights4 + ruleWeights5 + ruleWeights6 + ruleWeights7 +
                  ruleWeights8;

    double result = 0;
    if (denomerator != 0) result = numerator / denomerator;
    cout << "\n result of control = " << result * 100 << "\n";

    if (result > 0)
      a += (v * result) / dt;  // we control only the accelerator pedal!

    //--- Sugeno Control End

    v = v + a * dt;
    t = t + dt;
    if (++i % 10 == 0) {
      this_thread::sleep_for(chrono::milliseconds(1000));
      cout << "a= " << a << "; Pj = " << Pj << "; Pi = " << Pi
           << "; Pw = " << Pw << "; Pf = " << Pf << endl;
      cout << "[" << t << "]"
           << " v = " << v * 3.6 << endl;
    }

    if (v <= 0) {
      cout << "stopped at t = " << t << endl;
      break;
    }
  }
}

/*
void oneStep() {
  double v0 = 49 / 3.6;       // initial speed
  double speed_r = 50 / 3.6;  // desired speed
  double dt = 0.1;
  double v = v0;
  double a = 0;
  double v_old = v;

  //--- Sugeno Control

  double SpeedError = v - speed_r;
  double SpeedErrorDot = 0;  // a;

  // Fuzzification step

  double FSpeedErrorSlower = TriangleGetValue(-35.0, -20.0, -5.0, SpeedError);
  double FSpeedErrorZero = TriangleGetValue(-15.0, -0.0, 15.0, SpeedError);
  double FSpeedErrorFaster = TriangleGetValue(5.0, 20.0, 35.0, SpeedError);

  double FSpeedErrorDotSlower =
      TriangleGetValue(-9.0, -5.0, -1.0, SpeedErrorDot);
  double FSpeedErrorDotZero = TriangleGetValue(-4.0, -0.0, 4.0, SpeedErrorDot);
  double FSpeedErrorDotFaster =
      TriangleGetValue(5.0, 20.0, 35.0, SpeedErrorDot);

  // Evaluate the conditions
  double ruleWeights0 = std::min(FSpeedErrorSlower, FSpeedErrorDotSlower);
  double ruleWeights1 = std::min(FSpeedErrorSlower, FSpeedErrorDotZero);
  double ruleWeights2 = std::min(FSpeedErrorSlower, FSpeedErrorDotFaster);
  double ruleWeights3 = std::min(FSpeedErrorZero, FSpeedErrorDotSlower);
  double ruleWeights4 = std::min(FSpeedErrorZero, FSpeedErrorDotZero);
  double ruleWeights5 = std::min(FSpeedErrorZero, FSpeedErrorDotFaster);
  double ruleWeights6 = std::min(FSpeedErrorFaster, FSpeedErrorDotSlower);
  double ruleWeights7 = std::min(FSpeedErrorFaster, FSpeedErrorDotZero);
  double ruleWeights8 = std::min(FSpeedErrorFaster, FSpeedErrorDotFaster);

  // Functions evaluation
  double zeroResult = 0 * SpeedError + 0 * SpeedErrorDot + 0;
  double fasterResult = 0 * SpeedError + 0 * SpeedErrorDot + 1;
  double slowerResult = 0 * SpeedError + 0 * SpeedErrorDot - 1;
  double funcResult = -0.04 * SpeedError - 0.1 * SpeedErrorDot + 0;

  // Combine output
  double numerator = 0;
  double denomerator = 0;

  numerator += fasterResult * ruleWeights0 + fasterResult * ruleWeights1 +
               zeroResult * ruleWeights2 + fasterResult * ruleWeights3 +
               funcResult * ruleWeights4 + slowerResult * ruleWeights5 +
               zeroResult * ruleWeights6 + slowerResult * ruleWeights7 +
               slowerResult * ruleWeights8;
  denomerator = ruleWeights0 + ruleWeights1 + ruleWeights2 + ruleWeights3 +
                ruleWeights4 + ruleWeights5 + ruleWeights6 + ruleWeights7 +
                ruleWeights8;

  double result = 0;
  if (denomerator != 0) result = numerator / denomerator;
  cout << "\n result of control = " << result * 100 << "\n";

  if (result > 0)
    a += (v * result) / dt;  // we control only the accelerator pedal!

  //--- Sugeno Control End

  v = v + a * dt;

  cout << " v = " << v * 3.6 << endl;
}
*/

int main() {
  // run
  CarAndControl();
  // oneStep();
}
