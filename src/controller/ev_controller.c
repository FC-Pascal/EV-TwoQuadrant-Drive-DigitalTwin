#include "ev_controller.h"

// Discrete Sample Time & Bus Voltage
#define TS_CTRL     0.0001      // 100 us (10 kHz execution rate)
#define V_BUS       48.0        // 48V DC Bus

// Analytical Outer Speed Loop PI Gains
#define KP_W        2.14
#define KI_W        21.4
#define I_MAX       20.0        // +20A Quadrant 1 Motoring Limit
#define I_MIN      -20.0        // -20A Quadrant 2 Regen Braking Limit

// Analytical Inner Current Loop PI Gains
#define KP_I        4.71
#define KI_I        785.4

// Static persistent registers (simulating MCU RAM across interrupts)
static double int_w = 0.0;
static double int_i = 0.0;

static double clamp(double val, double min_val, double max_val) {
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

void EV_Drive_Controller_Step(double w_ref, double w_meas, double Ia_meas, 
                              double *duty_cycle, double *Ia_ref_out) 
{
    // --- 1. OUTER SPEED LOOP (Outputs Target Current Ia_ref) ---
    double err_w = w_ref - w_meas;
    
    // Trapezoidal / Backward Euler integration with clamping (Anti-Windup)
    int_w += (KI_W * err_w) * TS_CTRL;
    int_w = clamp(int_w, I_MIN, I_MAX);
    
    double Ia_ref = (KP_W * err_w) + int_w;
    Ia_ref = clamp(Ia_ref, I_MIN, I_MAX);
    *Ia_ref_out = Ia_ref;   // Export for Scope telemetry

    // --- 2. INNER CURRENT LOOP (Outputs Required Terminal Voltage Va_ref) ---
    double err_i = Ia_ref - Ia_meas;
    
    int_i += (KI_I * err_i) * TS_CTRL;
    int_i = clamp(int_i, 0.0, V_BUS);
    
    double Va_ref = (KP_I * err_i) + int_i;
    Va_ref = clamp(Va_ref, 0.0, V_BUS);

    // --- 3. PWM DUTY CYCLE MODULATION ---
    double D = Va_ref / V_BUS;
    *duty_cycle = clamp(D, 0.02, 0.98); // Clamp between 2% and 98%
}