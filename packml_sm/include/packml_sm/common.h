/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Plus One Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef COMMON_H
#define COMMON_H

#include <iostream>

namespace packml_sm
{
// This magic function allows iostream (i.e. ROS_##_STREAM) macros to print out
// enumerations
// see: http://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template <typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
  return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

enum class StatesEnum
{
  UNDEFINED = 0,
  CLEARING = 1,
  STOPPED = 2,
  STARTING = 3,
  IDLE = 4,
  SUSPENDED = 5,
  EXECUTE = 6,
  STOPPING = 7,
  ABORTING = 8,
  ABORTED = 9,
  HOLDING = 10,
  HELD = 11,
  UNHOLDING = 12,
  SUSPENDING = 13,
  UNSUSPENDING = 14,
  RESETTING = 15,
  COMPLETING = 16,
  COMPLETE = 17,


  // Super states that encapsulate multiple substates with a common transition
  // Not explicitly used in the standard but helpful for consutrcting the state
  // machine.
  OFF = 200,
  ABORTABLE = 201,
  STOPPABLE = 202
};

enum class ModeEnum
{
  UNDEFINED = 0,
  AUTOMATIC = 1,
  SEMI_AUTOMATIC = 2,
  MANUAL = 3,
  IDLE = 4,
  SETUP = 11
};

enum class CmdEnum : int
{
  NO_COMMAND=0,
  RESET=1,
  START=2,
  STOP=3,
  HOLD=4,
  UNHOLD=5,
  SUSPEND=6,
  UNSUSPEND=7,
  ABORT=8,
  CLEAR=9
};

enum class EventsEnum : int
{
  UNDEFINED=0,
  STATE_COMPLETE=1,
  HOLD=2,
  UNHOLD=3,
  SUSPEND=4,
  UNSUSPEND=5,
  RESET=6,
  CLEAR=7
};

enum class MetricIDEnum : int32_t
{
  CYCLE_INC_ID = 0,
  SUCCESS_INC_ID = 1,
  FAILURE_INC_ID = 2,
  MIN_QUALITY_ID = 1000,
  MAX_QUALITY_ID = 1999,
  MIN_ERROR_ID = 2000,
  MAX_ERROR_ID = 2999
};

} // End namespace packml_sm

#endif  // COMMON_H
