// (c) Sergey Staroletov
// Cruise control using Takagi-Sugeno
// Based on idea of Fuzzy Logic Library for Microsoft .NET (C) 2008 Dmitry
// Kaluzhny

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <thread>

using namespace std;

int active;
int kol;
int max_kol = 990;
int n;
int size;
double XX[1000][201]; // kol vectors of n
int what[1000];       // teaching result
double alph = 2;
double delt = 0.1;

double angle = std::atan(-0.5); // slope, %

double W[6][201][201];

const int N = 2, M = 100, L = 100, K = 2;
// int N = 32, M = 25, L = 15, K = 200;

double X[6][1000];
double Y[6][1000];
double E[1000];
int MaxEpoha = 1000;
double Eps = 0.06;

double Sg(double x)
{
    return 1.0 / (1.0 + exp(-alph * x));
}

double *neuronetWork(double *x)
{
    for (int i = 0; i < N; i++)
        Y[0][i] = x[i];

    for (int m = 0; m < M; m++) {
        X[1][m] = 0;
        for (int n = 0; n < N; n++)
            X[1][m] += W[0][n][m] * Y[0][n];
    }

    for (int m = 0; m < M; m++)
        Y[1][m] = Sg(X[1][m]);

    for (int l = 0; l < L; l++) {
        X[2][l] = 0;
        for (int m = 0; m < M; m++)
            X[2][l] += W[1][m][l] * Y[1][m];
    }

    for (int l = 0; l < L; l++)
        Y[2][l] = Sg(X[2][l]);

    for (int k = 0; k < K; k++) {
        X[3][k] = 0;
        for (int l = 0; l < L; l++)
            X[3][k] += W[2][l][k] * Y[2][l];
    }

    for (int k = 0; k < K; k++)
        Y[3][k] = Sg(X[3][k]);

    return Y[3];
}

void learn()
{
    double EXk3[201];
    double El2[201];
    double Em1[201];
    double EXm[201];
    double EXl[201];

    srand(time(NULL));

    for (int a = 0; a < 5; a++)
        for (int b = 0; b < 200; b++)
            for (int c = 0; c < 200; c++) {
                W[a][b][c] = 0.5 - 1.0 * (rand() % 200) / 200.0; //-2..2
            }

    double SumE;
    int Npop = 0;

    double *X = new double[N + 1];

    do {
        SumE = 0;

        for (int i = 0; i < kol; i++) {
            for (int j = 0; j < N; j++)
                X[j] = XX[i][j];

            double *outp = neuronetWork(X); // result
            int res = 100 + 100 * outp[0] - 100 * outp[1];

            double d[N];
            memset(d, 0, sizeof(d));

            //needed class
            int nado = what[i];
            // put 1 into place
            d[0] = double(nado) / 200;
            d[1] = 1. - d[0];

            // cout << "nado: " << nado << " est: " << res << endl;

            //net error
            E[i] = 0;
            for (int k = 0; k < K; k++)
                E[i] += (Y[3][k] - (double) d[k]) * (Y[3][k] - (double) d[k]);
            // E[i] = sqrt(E[i]);
            E[i] *= 0.5;
            SumE += E[i];

            for (int k = 0; k < K; k++) {
                double Eyk3 = Y[3][k] - d[k];
                EXk3[k] = Eyk3 * alph * Y[3][k] * (1.0 - Y[3][k]);
                for (int l = 0; l < L; l++) {
                    double D_E_po_wml = EXk3[k] * Y[2][l];
                    W[2][l][k] -= delt * D_E_po_wml;
                }
            }
            // wml
            for (int l = 0; l < L; l++) {
                El2[l] = 0;
                for (int k = 0; k < K; k++)
                    El2[l] += EXk3[k] * W[2][l][k];
                EXl[l] = El2[l] * alph * Y[2][l] * (1.0 - Y[2][l]);
                for (int m = 0; m < M; m++) {
                    W[1][m][l] -= delt * EXl[l] * Y[1][m];
                }
            }
            // wnm
            for (int m = 0; m < M; m++) {
                Em1[m] = 0;
                for (int l = 0; l < L; l++)
                    Em1[m] += El2[l] * W[1][m][l];
                EXm[m] = Em1[m] * alph * Y[1][m] * (1.0 - Y[1][m]);
                for (int n = 0; n < N; n++) {
                    W[0][n][m] -= delt * EXm[m] * Y[0][n];
                }
            }
        } // for
        Npop++;
        //cout << "Step " << Npop << " Error: " << SumE << endl;
        cout << Npop << ";" << SumE << ";" << endl;
    } while (SumE > Eps && Npop < MaxEpoha);
    cout << "Fin with error " << SumE << " for " << Npop << " steps" << endl;
}

double TriangleGetValue1(double x1, double x2, double x3, double x)
{
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

/*
 * @returns Acceleration value for given speeds
 */
double SugenoStep1(double v, double speed_r, double dt, double accel)
{
    double a = 0;
    double SpeedError = v - speed_r;
    double SpeedErrorDot = accel; // a;

    // Fuzzification step
    double FSpeedErrorSlower = TriangleGetValue1(-35.0, -20.0, -5.0, SpeedError);
    double FSpeedErrorZero = TriangleGetValue1(-15.0, -0.0, 15.0, SpeedError);
    double FSpeedErrorFaster = TriangleGetValue1(5.0, 20.0, 35.0, SpeedError);

    double FSpeedErrorDotSlower = TriangleGetValue1(-9.0, -5.0, -1.0, SpeedErrorDot);
    double FSpeedErrorDotZero = TriangleGetValue1(-4.0, -0.0, 4.0, SpeedErrorDot);
    double FSpeedErrorDotFaster = TriangleGetValue1(5.0, 20.0, 35.0, SpeedErrorDot);

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
    double numerator = fasterResult * ruleWeights0 + fasterResult * ruleWeights1
                       + zeroResult * ruleWeights2 + fasterResult * ruleWeights3
                       + funcResult * ruleWeights4 + slowerResult * ruleWeights5
                       + zeroResult * ruleWeights6 + slowerResult * ruleWeights7
                       + slowerResult * ruleWeights8;
    double denomerator = ruleWeights0 + ruleWeights1 + ruleWeights2 + ruleWeights3 + ruleWeights4
                         + ruleWeights5 + ruleWeights6 + ruleWeights7 + ruleWeights8;

    double result = 0;
    if (denomerator != 0)
        result = numerator / denomerator;
    cout << "\n " << kol << " result of control = " << result * 1000 << "\n";

    int rez = (int) (result * 1000) + 100;

    if (kol < max_kol && rez > 0 && rez < 200) {
        // add a learning point
        cout << "(" << SpeedError << "," << SpeedErrorDot << ")->" << rez << endl;
        what[kol] = rez;
        XX[kol][0] = SpeedError;
        XX[kol][1] = SpeedErrorDot;
        kol++;
    }

    if (result > 0)
        a = (v * result) / dt; // we control only the accelerator pedal!

    return a;
}

/*
 * Car movement simulation with forces and Sugeno control
 */
void CarAndControl1()
{
    double m = 1355 + 72 + 10; // vehicle mass
    //double sigma = 1.04;        // koeff for good asfphalt
    double g = 9.81;
    double k = 0.20;
    double he = 1.435;
    double we = 1.780;
    double f0 = 0.01;
    double f;

    double v0 = 50 / 3.6;      // initial speed
    double speed_r = 50 / 3.6; // desired speed

    double t = 0;
    double dt = 0.1;

    double v = v0;

    double a = 0;

    while (true) {
        // physics
        double Pj = 0; //+sigma * (a - a_old) * m; //inertia - not used for now
        double Pi = +m * g * std::sin(angle); // acceleration/deceleration due to downhill or uphill
        double Pw = -k * 0.8 * we * he * v * v; // deceleration due to wind
        if (v <= 50)
            f = f0;
        else
            f = f0 * (1 + 0.01 * (50 - v));

        double Pf = -f * m * g; // deceleration due to friction on asphalt
        double F = (Pi + Pj + Pw + Pf) / m;
        a = F;

        //--- NET Control

        // a += SugenoStep1(v, speed_r, dt, (v - speed_r) / dt);
        double X[3];
        X[0] = v - speed_r;
        X[1] = (v - speed_r) / dt;
        double *outp = neuronetWork(X);
        int rez = int(100 + 100 * outp[0] - 100 * outp[1]);
        cout << "Net returned: " << rez << endl;

        double rezz = ((rez - 100.0) / 1000.0);
        if (rezz > 0)
            a += v * rezz / dt;

        //--- NET Control End

        v = v + a * dt;
        t = t + dt;
        this_thread::sleep_for(chrono::milliseconds(500));
        cout << "a= " << a << "; Pj = " << Pj << "; Pi = " << Pi << "; Pw = " << Pw
             << "; Pf = " << Pf << endl;
        cout << "[" << t << "]"
             << " v = " << v * 3.6 << endl;

        if (v <= 0) {
            cout << "stopped at t = " << t << endl;
            break;
        }
    }
}

void CarAndControl2()
{
    double m = 1355 + 72 + 10; // vehicle mass
    //double sigma = 1.04;        // koeff for good asfphalt
    double g = 9.81;
    double k = 0.20;
    double he = 1.435;
    double we = 1.780;
    double f0 = 0.01;
    double f;

    double v0 = 50 / 3.6;      // initial speed
    double speed_r = 50 / 3.6; // desired speed

    double t = 0;
    double dt = 0.1;

    double v = v0;

    double a = 0;

    int i = 0;

    while (true) {
        // physics
        double Pj = 0; //+sigma * (a - a_old) * m; //inertia - not used for now
        double Pi = +m * g * std::sin(angle); // acceleration/deceleration due to downhill or uphill
        double Pw = -k * 0.8 * we * he * v * v; // deceleration due to wind
        if (v <= 50)
            f = f0;
        else
            f = f0 * (1 + 0.01 * (50 - v));

        double Pf = -f * m * g; // deceleration due to friction on asphalt
        double F = (Pi + Pj + Pw + Pf) / m;
        a = F;

        //--- Sugeno Control
        a += SugenoStep1(v, speed_r, dt, (v - speed_r) / dt);
        //--- Sugeno Control End

        if (kol >= max_kol)
            break;

        v = v + a * dt;
        t = t + dt;
        if (++i % 10 == 0) {
            // this_thread::sleep_for(chrono::milliseconds(1000));
            cout << "a= " << a << "; Pj = " << Pj << "; Pi = " << Pi << "; Pw = " << Pw
                 << "; Pf = " << Pf << endl;
            cout << "[" << t << "]"
                 << " v = " << v * 3.6 << endl;
        }

        if (v <= 0) {
            cout << "stopped at t = " << t << endl;
            break;
        }
    }

    learn();
}

int main()
{
    kol = 0;
    // run
    CarAndControl2();
    CarAndControl1();
}
