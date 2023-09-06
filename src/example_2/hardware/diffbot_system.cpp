

#include "include/ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //device = info_.hardware_parameters["device"];
  baudrate = std::stoi(info_.hardware_parameters["baud_rate"]);
  left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name = info_.hardware_parameters["right_wheel_name"];

  //dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
  //dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);




  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}





std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name, hardware_interface::HW_IF_POSITION, &left_wheel_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name, hardware_interface::HW_IF_POSITION, &right_wheel_position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity));

  return state_interfaces;
}




std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
     left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_wheel_command));
  
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
     right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_wheel_command));
  
  return command_interfaces;
}




hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configuring ...please wait...");

  
  // Open port
  if (portHandler->openPort()) {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "port opened Succesfully");
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "error open port");
    return hardware_interface::CallbackReturn::ERROR;
    
    
  }
  // Set port baudrate
  if (portHandler->setBaudRate(baudrate)) {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "baudrate set Succesfully");
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "error set baudrate");
    return hardware_interface::CallbackReturn::ERROR;
    
  }
  
  
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Succesfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;

}




hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Cleaning up ...please wait...");
  
  
  portHandler->closePort();

  
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Succesfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}






hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, left_dxl_id, addr_torque_enable, 1, &left_dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "DXL1 TORQUE ENABLE FAILED");
    //printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  /*else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }*/
  else {
    printf("Succeeded enabling DYNAMIXEL Torque.\n");
  }




  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, right_dxl_id, addr_torque_enable, 1, &right_dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "DXL2 TORQUE ENABLE FAILED");
    //printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  /*else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }*/
  else {
    printf("Succeeded enabling DYNAMIXEL Torque.\n");
  }
  
  
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
  
}







hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  packetHandler->write1ByteTxRx(portHandler, left_dxl_id, addr_torque_enable, 0, &left_dxl_error);
  packetHandler->write1ByteTxRx(portHandler, right_dxl_id, addr_torque_enable, 0, &right_dxl_error);

  
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}





hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  int32_t raw_left_wheel_position = 0;
  int32_t raw_right_wheel_position = 0;

  int32_t raw_left_wheel_velocity = 0;
  int32_t raw_right_wheel_velocity = 0;


  packetHandler->read4ByteTxRx(portHandler, left_dxl_id, addr_present_position, (uint32_t*)&raw_left_wheel_position, &left_dxl_error);
  packetHandler->read4ByteTxRx(portHandler, right_dxl_id, addr_present_position, (uint32_t*)&raw_right_wheel_position, &right_dxl_error);

  packetHandler->read4ByteTxRx(portHandler, left_dxl_id, addr_present_velocity, (uint32_t*)&raw_left_wheel_velocity, &left_dxl_error);
  packetHandler->read4ByteTxRx(portHandler, right_dxl_id, addr_present_velocity, (uint32_t*)&raw_right_wheel_velocity, &right_dxl_error);
  
  
 
  if (raw_left_wheel_position != 0) {
    left_wheel_position = 1.0 * ((raw_left_wheel_position / 4095) * 6.28);
  } else {
    left_wheel_position = 0.0; // Set to a default value if needed
  }

  if (raw_right_wheel_position != 0) {
    right_wheel_position = 1.0 * ((raw_right_wheel_position / 4095) * 6.28);
  } else {
    right_wheel_position = 0.0; // Set to a default value if needed
  }

  if (raw_left_wheel_velocity != 0) {
    left_wheel_velocity = 1.0 * ((raw_left_wheel_velocity / 60) * 6.28);
  } else {
    left_wheel_velocity = 0.0; // Set to a default value if needed
  }

  if (raw_right_wheel_velocity != 0) {
    right_wheel_velocity = 1.0 * ((raw_right_wheel_velocity / 60) * 6.28);
  } else {
    right_wheel_velocity = 0.0; // Set to a default value if needed
  }

  return hardware_interface::return_type::OK;
}









hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double modified_left_wheel_command = 0.0;
  double modified_right_wheel_command = 0.0;

  if (left_wheel_command != 0.0) {
    modified_left_wheel_command = ((left_wheel_command / 6.28) * 60) / 0.229;// Calculate and write modified_left_wheel_command
  } else {
    modified_left_wheel_command = 0.0;// Handle the case when left_wheel_command is zero
    // For example, set a default value or handle it differently
  }

  if (right_wheel_command != 0.0) {
    modified_right_wheel_command = ((right_wheel_command / 6.28) * 60) / 0.229;// Calculate and write modified_right_wheel_command
  } else {
    modified_right_wheel_command = 0.0;// Handle the case when right_wheel_command is zero
    // For example, set a default value or handle it differently
  }

  packetHandler->write4ByteTxRx(portHandler, left_dxl_id, addr_goal_velocity, modified_left_wheel_command, &left_dxl_error);
  packetHandler->write4ByteTxRx(portHandler, right_dxl_id, addr_goal_velocity, modified_right_wheel_command, &right_dxl_error);



  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
