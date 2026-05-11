/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2023 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <functional>  // 需包含函数对象头文件

namespace astra_camera {
class ParametersBackend {
public:
  // 定义回调类型：接收参数列表，返回设置结果
  using OnParametersSetCallbackType = std::function<rcl_interfaces::msg::SetParametersResult(
    const std::vector<rclcpp::Parameter>& parameters
  )>;

  // 构造函数（保持不变）
  explicit ParametersBackend(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params)
    : node_params_(node_params) {}

  // 声明回调注册函数（参数类型为上面定义的回调类型）
  void addOnSetParametersCallback(OnParametersSetCallbackType callback);

private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_;
};
}  // namespace astra_camera