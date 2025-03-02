// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "service_utils.hpp"

#include <string>

namespace service_utils
{

// 创建一个表示服务未准备好的异常
ServiceException ServiceUnready(const std::string & message)
{
  // 返回一个 ServiceException 对象，状态为 SERVICE_UNREADY，附带错误信息
  return ServiceException(ResponseStatus::SERVICE_UNREADY, message, false);
}

// 创建一个表示坐标转换错误的异常
ServiceException TransformError(const std::string & message)
{
  // 返回一个 ServiceException 对象，状态为 TRANSFORM_ERROR，附带错误信息
  return ServiceException(ResponseStatus::TRANSFORM_ERROR, message, false);
};

}  // namespace service_utils
