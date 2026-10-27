#ifndef EV_CONTROLLER_H
#define EV_CONTROLLER_H

// Main 10 kHz (100 us) Discrete-Time Control ISR Function
void EV_Drive_Controller_Step(double w_ref, double w_meas, double Ia_meas, 
                              double *duty_cycle, double *Ia_ref_out);

#endif