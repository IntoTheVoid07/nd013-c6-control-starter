/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:
    /*
     * Constructor
     */
    PID(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini);

    /*
     * Destructor.
     */
    ~PID() = default;

    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);

    /*
     * Calculate the total PID error within the bounds of [output_lim_mini, output_lim_maxi]
     */
    double TotalError();
  
    /*
     * Update the delta time.
     */
    void UpdateDeltaTime(double new_delta_time);

    /*
     * Clamps the value between a minimum and maximum value.
     * @note: If compiler was C++17 with std::clamp, would use that instead
     */
    //
    template<class T>
    constexpr const T& clamp(const T& x, const T& lo, const T& hi);

private:
    /*
     * Coefficients
     */
    double tau_p_;
    double tau_i_;
    double tau_d_;

    /*
     * Output limits
     */
    double output_lim_max_;
    double output_lim_min_;

    /*
     * Errors
     */
    double diff_cte_;
    double prev_cte_;
    double int_cte_;

    /*
     * Delta time
     */
    double delta_time_;
};

#endif //PID_CONTROLLER_H
