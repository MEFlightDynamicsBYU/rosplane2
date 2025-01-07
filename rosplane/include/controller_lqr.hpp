#ifndef CONTROLLER_LQR_H
#define CONTROLLER_LQR_H

#include "controller_state_machine.hpp"

#include <pybind11/embed.h>

namespace py = pybind11;

namespace rosplane
{

class ControllerLQR : public ControllerStateMachine
{
public:
  /**
   * Constructor to initialize node.
   */
  ControllerLQR();

protected:
  /**
   * This function continually loops while the aircraft is in the take-off zone. The lateral and longitudinal control
   * for the take-off zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off(const Input & input, Output & output);

  /**
   * This function continually loops while the aircraft is in the climb zone. The lateral and longitudinal control
   * for the climb zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb(const Input & input, Output & output);

  /**
   * This function continually loops while the aircraft is in the altitude hold zone. The lateral and longitudinal 
   * control for the altitude hold zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void altitude_hold(const Input & input, Output & output);

  /**
   * This function runs when the aircraft exits the take-off zone. Any changes to the controller that need to happen
   * only once as the aircraft exits take-off mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void take_off_exit();

  /**
   * This function runs when the aircraft exits the climb zone. Any changes to the controller that need to happen
   * only once as the aircraft exits climb mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void climb_exit();

  /**
   * This function runs when the aircraft exits the altitude hold zone (usually a crash). Any changes to the controller that 
   * need to happen only once as the aircraft exits altitude mode should be placed here. This sets differentiators and
   * integrators to 0.
   */
  virtual void altitude_hold_exit();

  /**
   * This function runs the lateral control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_longitudinal_control(const Input & input, Output & output);

  /**
   * This function runs the lateral control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_longitudinal_control(const Input & input, Output & output);

  /**
   * This function runs the lateral control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_longitudinal_control(const Input & input, Output & output);

private:
  /**
   * Declares the parameters associated to this controller, controller_successive_loop, so that ROS2 can see them.
   * Also declares default values before they are set to the values set in the launch script.
  */
  void declare_parameters();

  void pack_for_python(const Input & input);
  void unpack_from_python(Output & output);

};
} // namespace rosplane

#endif // CONTROLLER_LQR_H
