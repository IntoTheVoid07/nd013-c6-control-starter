/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <iostream>
#include <math.h>
#include <vector>

PID::PID() :
    tau_p_(0.0),
    tau_i_(0.0),
    tau_d_(0.0),
    output_lim_max_(0.0),
    output_lim_min_(0.0),
    diff_cte_(0.0),
    prev_cte_(0.0),
    int_cte_(0.0)
{ }

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    tau_p_ = Kpi;
    tau_i_ = Kii;
    tau_d_ = Kdi;
    output_lim_max_ = output_lim_maxi;
    output_lim_min_ = output_lim_mini;
}

void PID::UpdateError(double cte) {
    if (delta_time_ > 0.0) {
        diff_cte_ = (cte - prev_cte_) / delta_time_;
    }
    else {
        diff_cte_ = 0.0;
    }

    int_cte_ += cte;
    prev_cte_ = cte;
}

double PID::TotalError() {
    // control = (-tau_p * CTE) - (tau_d * (d/dt CTE)) - (tau_i * ∫ CTE)
    //                 ^                   ^                   ^
    //           Proportional (P)      Differential (D)     Integral (I)
    double control = -tau_p_ * prev_cte_ - tau_d_ * diff_cte_ - tau_i_ * int_cte_;
    return PID::clamp(control, output_lim_min_, output_lim_max_);
}

double PID::UpdateDeltaTime(double new_delta_time) {
    double prev_delta_time = delta_time_;
    delta_time_ = new_delta_time;
    return prev_delta_time; // Uncertain if this is what's expected right now...
}

template<class T>
constexpr const T& PID::clamp( const T& x, const T& lo, const T& hi)
{
    return std::min(hi, std::max(x, lo));
}
