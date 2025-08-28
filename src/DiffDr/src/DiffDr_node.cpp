#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{

DiffDriveArduino::DiffDriveArduino()
: logger_(rclcpp::get_logger("DiffDriveArduino")) {}

CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring DiffDriveArduino...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Serial to Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  RCLCPP_INFO(logger_, "Finished configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos);
  state_interfaces.emplace_back(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel);
  state_interfaces.emplace_back(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos);
  state_interfaces.emplace_back(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd);
  command_interfaces.emplace_back(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd);
  return command_interfaces;
}

CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating DiffDriveArduino...");
  arduino_.sendEmptyMsg();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating DiffDriveArduino...");
  return CallbackReturn::SUCCESS;
}

return_type DiffDriveArduino::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double dt = diff.count();
  time_ = new_time;

  if (!arduino_.connected()) {
    return return_type::ERROR;
  }

  // --- Read encoder counts from Arduino ---
  long enc_left, enc_right;
  if (!arduino_.readEncoderValues(enc_left, enc_right)) {
    RCLCPP_WARN(logger_, "Failed to read encoder values");
    return return_type::ERROR;
  }

  // Update wheel states
  double pos_prev = l_wheel_.pos;
  l_wheel_.enc = enc_left;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / dt;

  pos_prev = r_wheel_.pos;
  r_wheel_.enc = enc_right;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / dt;

  return return_type::OK;
}

return_type DiffDriveArduino::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!arduino_.connected()) {
    return return_type::ERROR;
  }

  // Convert wheel velocity commands [rad/s] to RPM or PWM before sending
  double rpm_left  = (l_wheel_.cmd * 60.0) / (2.0 * M_PI);   // rad/s → RPM
  double rpm_right = (r_wheel_.cmd * 60.0) / (2.0 * M_PI);

  // Build command string: V<left>,<right>
  std::stringstream ss;
  ss << "V" << rpm_left << "," << rpm_right << "\n";

  arduino_.writeMsg(ss.str());

  return return_type::OK;
}

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduino,
  hardware_interface::SystemInterface
)
