// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/FireCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__FIRE_COMMAND__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__FIRE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/fire_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_FireCommand_pitch
{
public:
  explicit Init_FireCommand_pitch(::auto_aim_interfaces::msg::FireCommand & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::FireCommand pitch(::auto_aim_interfaces::msg::FireCommand::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::FireCommand msg_;
};

class Init_FireCommand_yaw
{
public:
  explicit Init_FireCommand_yaw(::auto_aim_interfaces::msg::FireCommand & msg)
  : msg_(msg)
  {}
  Init_FireCommand_pitch yaw(::auto_aim_interfaces::msg::FireCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_FireCommand_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::FireCommand msg_;
};

class Init_FireCommand_header
{
public:
  Init_FireCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FireCommand_yaw header(::auto_aim_interfaces::msg::FireCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FireCommand_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::FireCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::FireCommand>()
{
  return auto_aim_interfaces::msg::builder::Init_FireCommand_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__FIRE_COMMAND__BUILDER_HPP_
