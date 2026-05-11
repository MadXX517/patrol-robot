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

#include "astra_camera/ros_param_backend.h"

namespace astra_camera {
void ParametersBackend::addOnSetParametersCallback(OnParametersSetCallbackType callback) {
  if (node_params_) {
    // 调用ROS 2的参数回调注册API（Jazzy中为蛇形命名）
    node_params_->add_on_set_parameters_callback(callback);
  }
}
}  // namespace astra_camera
