#include <cmath>

#include "controller_lqr.hpp"

namespace rosplane
{

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

ControllerLQR::ControllerLQR()
{
  // Declare parameters associated with this controller, controller_state_machine
  declare_parameters();
  // Set parameters according to the parameters in the launch file, otherwise use the default values
  params_.set_parameters();
}

void ControllerLQR::take_off(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  take_off_lateral_control(input, output);
  take_off_longitudinal_control(input, output);
}

void ControllerLQR::take_off_exit()
{
  
}

void ControllerLQR::climb(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  climb_lateral_control(input, output);
  climb_longitudinal_control(input, output);
}

void ControllerLQR::climb_exit()
{
  
}

void ControllerLQR::altitude_hold(const Input & input, Output & output)
{

}

void ControllerLQR::altitude_hold_exit()
{

}

void ControllerLQR::alt_hold_lateral_control(const Input & input, Output & output)
{

}

void ControllerLQR::alt_hold_longitudinal_control(const Input & input, Output & output)
{

}

void ControllerLQR::climb_lateral_control(const Input & input, Output & output)
{

}

void ControllerLQR::climb_longitudinal_control(const Input & input, Output & output)
{

}

void ControllerLQR::take_off_lateral_control(const Input & input, Output & output)
{

}

void ControllerLQR::take_off_longitudinal_control(const Input & input, Output & output)
{

}

void ControllerLQR::declare_parameters()
{

}

} // namespace rosplane
