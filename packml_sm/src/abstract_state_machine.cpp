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

#include "packml_sm/abstract_state_machine.h"
#include "packml_sm/common.h"
#include "packml_sm/dlog.h"

#include <thread>

namespace packml_sm
{
AbstractStateMachine::AbstractStateMachine() : start_time_(std::chrono::steady_clock::now()),
incremental_start_time_(std::chrono::steady_clock::now())
{
}

bool AbstractStateMachine::start()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::IDLE:
      _start();
      return true;
    default:
      DLog::LogWarning("Ignoring START command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::clear()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTED:
      _clear();
      return true;
    default:
      DLog::LogWarning("Ignoring CLEAR command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::reset()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::COMPLETE:
    case StatesEnum::STOPPED:
      _reset();
      return true;
    default:
      DLog::LogWarning("Ignoring RESET command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::hold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _hold();
      return true;
    default:
      DLog::LogWarning("Ignoring HOLD command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::unhold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::HELD:
      _unhold();
      return true;
    default:
      DLog::LogWarning("Ignoring HELD command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::suspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _suspend();
      return true;
    default:
      DLog::LogWarning("Ignoring SUSPEND command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::unsuspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::SUSPENDED:
      _unsuspend();
      return true;
    default:
      DLog::LogWarning("Ignoring UNSUSPEND command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::stop()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::STOPPABLE:
    case StatesEnum::STARTING:
    case StatesEnum::IDLE:
    case StatesEnum::SUSPENDED:
    case StatesEnum::EXECUTE:
    case StatesEnum::HOLDING:
    case StatesEnum::HELD:
    case StatesEnum::SUSPENDING:
    case StatesEnum::UNSUSPENDING:
    case StatesEnum::UNHOLDING:
    case StatesEnum::COMPLETING:
    case StatesEnum::COMPLETE:
      _stop();
      return true;
    default:
      DLog::LogWarning("Ignoring STOP command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::abort()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTABLE:
    case StatesEnum::STOPPED:
    case StatesEnum::STARTING:
    case StatesEnum::IDLE:
    case StatesEnum::SUSPENDED:
    case StatesEnum::EXECUTE:
    case StatesEnum::HOLDING:
    case StatesEnum::HELD:
    case StatesEnum::SUSPENDING:
    case StatesEnum::UNSUSPENDING:
    case StatesEnum::UNHOLDING:
    case StatesEnum::COMPLETING:
    case StatesEnum::COMPLETE:
    case StatesEnum::CLEARING:
    case StatesEnum::STOPPING:
      _abort();
      return true;
    default:
      DLog::LogWarning("Ignoring ABORT command in current state: %d", getCurrentState());
      return false;
  }
}

void AbstractStateMachine::getCurrentStatSnapshot(PackmlStatsSnapshot& snapshot_out)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  auto scheduled_time = calculateTotalTime();
  float throughput = 0.0f;
  if (scheduled_time > std::numeric_limits<double>::epsilon())
  {
    throughput = success_count_ / scheduled_time;
  }

  auto held_time = getHeldTime();
  auto stopped_time = getStoppedTime();
  auto suspend_time = getSuspendedTime();
  auto aborted_time = getAbortedTime();

  auto operating_time = scheduled_time;
  operating_time -= stopped_time;
  operating_time -= suspend_time;
  operating_time -= aborted_time;

  float availability = 0.0f;
  if (scheduled_time > std::numeric_limits<double>::epsilon())
  {
    availability = operating_time / scheduled_time;
  }

  float performance = 0.0f;
  if (operating_time > std::numeric_limits<double>::epsilon())
  {
    performance = static_cast<float>(success_count_) * ideal_cycle_time_ / operating_time;
  }

  float quality = 0.0f;
  auto total_count = success_count_ + failure_count_;
  if (total_count > 0)
  {
    quality = static_cast<float>(success_count_) / static_cast<float>(total_count);
  }

  snapshot_out.duration = scheduled_time;
  snapshot_out.idle_duration = getIdleTime();
  snapshot_out.exe_duration = getExecuteTime();
  snapshot_out.held_duration = held_time;
  snapshot_out.susp_duration = suspend_time;
  snapshot_out.cmplt_duration = getCompleteTime();
  snapshot_out.stop_duration = stopped_time;
  snapshot_out.abort_duration = aborted_time;
  snapshot_out.success_count = success_count_;
  snapshot_out.fail_count = failure_count_;
  snapshot_out.throughput = throughput;
  snapshot_out.availability = availability;
  snapshot_out.performance = performance;
  snapshot_out.quality = quality;
  snapshot_out.overall_equipment_effectiveness = quality * performance * availability;
  snapshot_out.itemized_error_map = itemized_error_map_;
  snapshot_out.itemized_quality_map = itemized_quality_map_;
}

void AbstractStateMachine::getCurrentIncrementalStatSnapshot(PackmlStatsSnapshot &snapshot_out)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  auto scheduled_time = calculateTotalTime(true);
  float throughput = 0.0f;
  if (scheduled_time > std::numeric_limits<double>::epsilon())
  {
    throughput = incremental_success_count_ / scheduled_time;
  }

  auto held_time = getHeldTime(true);
  auto stopped_time = getStoppedTime(true);
  auto suspend_time = getSuspendedTime(true);
  auto aborted_time = getAbortedTime(true);

  auto operating_time = scheduled_time;
  operating_time -= stopped_time;
  operating_time -= suspend_time;
  operating_time -= aborted_time;

  float availability = 0.0f;
  if (scheduled_time > std::numeric_limits<double>::epsilon())
  {
    availability = operating_time / scheduled_time;
  }

  float performance = 0.0f;
  if (operating_time > std::numeric_limits<double>::epsilon())
  {
    performance = static_cast<float>(incremental_success_count_) * ideal_cycle_time_ / operating_time;
  }

  float quality = 0.0f;
  auto total_count = incremental_success_count_ + incremental_failure_count_;
  if (total_count > 0)
  {
    quality = static_cast<float>(incremental_success_count_) / static_cast<float>(total_count);
  }

  snapshot_out.duration = scheduled_time;
  snapshot_out.idle_duration = getIdleTime(true);
  snapshot_out.exe_duration = getExecuteTime(true);
  snapshot_out.held_duration = held_time;
  snapshot_out.susp_duration = suspend_time;
  snapshot_out.cmplt_duration = getCompleteTime(true);
  snapshot_out.stop_duration = stopped_time;
  snapshot_out.abort_duration = aborted_time;
  snapshot_out.success_count = incremental_success_count_;
  snapshot_out.fail_count = incremental_failure_count_;
  snapshot_out.throughput = throughput;
  snapshot_out.availability = availability;
  snapshot_out.performance = performance;
  snapshot_out.quality = quality;
  snapshot_out.overall_equipment_effectiveness = quality * performance * availability;
  snapshot_out.itemized_error_map = incremental_itemized_error_map_;
  snapshot_out.itemized_quality_map = inremental_itemized_quality_map_;

  resetIncrementalStats();
}

double AbstractStateMachine::getIdleTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::IDLE, is_incremental);
}

double AbstractStateMachine::getStartingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::STARTING, is_incremental);
}

double AbstractStateMachine::getResettingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::RESETTING, is_incremental);
}

double AbstractStateMachine::getExecuteTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::EXECUTE, is_incremental);
}

double AbstractStateMachine::getHeldTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::HELD, is_incremental);
}

double AbstractStateMachine::getHoldingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::HOLDING, is_incremental);
}

double AbstractStateMachine::getUnholdingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::UNHOLDING, is_incremental);
}

double AbstractStateMachine::getSuspendedTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::SUSPENDED, is_incremental);
}

double AbstractStateMachine::getSuspendingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::SUSPENDING, is_incremental);
}

double AbstractStateMachine::getUnsuspendingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::UNSUSPENDING, is_incremental);
}

double AbstractStateMachine::getCompleteTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::COMPLETE, is_incremental);
}

double AbstractStateMachine::getStoppedTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::STOPPED, is_incremental);
}

double AbstractStateMachine::getClearingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::CLEARING, is_incremental);
}

double AbstractStateMachine::getStoppingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::STOPPING, is_incremental);
}

double AbstractStateMachine::getAbortedTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::ABORTED, is_incremental);
}

double AbstractStateMachine::getAbortingTime(bool is_incremental)
{
  return getStateDuration(StatesEnum::ABORTING, is_incremental);
}

double AbstractStateMachine::calculateTotalTime(bool is_incremental)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  std::chrono::duration<double> duration = std::chrono::steady_clock::now() - (is_incremental ? incremental_start_time_ : start_time_);
  auto elapsed_time = duration.count();
  for (auto & iter : (is_incremental ? incremental_duration_map_ : duration_map_) )
  {
    elapsed_time += iter.second;
  }

  return elapsed_time;
}

void AbstractStateMachine::resetStats()
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  start_time_ = std::chrono::steady_clock::now();
  duration_map_.clear();
  failure_count_ = 0;
  success_count_ = 0;

  for (auto& itemized_it : itemized_error_map_)
  {
    itemized_it.second.count = 0;
    itemized_it.second.duration = 0;
  }

  for (auto& itemized_it : itemized_quality_map_)
  {
    itemized_it.second.count = 0;
    itemized_it.second.duration = 0;
  }
}

void AbstractStateMachine::resetIncrementalStats()
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  incremental_start_time_ = std::chrono::steady_clock::now();
  incremental_duration_map_.clear();
  incremental_success_count_ = 0;
  incremental_failure_count_ = 0;

  for (auto& itemized_it : incremental_itemized_error_map_)
  {
    itemized_it.second.count = 0;
    itemized_it.second.duration = 0;
  }

  for (auto& itemized_it : inremental_itemized_quality_map_)
  {
    itemized_it.second.count = 0;
    itemized_it.second.duration = 0;
  }
}

void AbstractStateMachine::loadStats(const PackmlStatsSnapshot &snapshot)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);

  setStateDuration(StatesEnum::IDLE, snapshot.idle_duration);
  setStateDuration(StatesEnum::EXECUTE, snapshot.exe_duration);
  setStateDuration(StatesEnum::HELD, snapshot.held_duration);
  setStateDuration(StatesEnum::SUSPENDED, snapshot.susp_duration);
  setStateDuration(StatesEnum::COMPLETE, snapshot.cmplt_duration);
  setStateDuration(StatesEnum::STOPPED, snapshot.stop_duration);
  setStateDuration(StatesEnum::ABORTED, snapshot.abort_duration);

  success_count_ = snapshot.success_count;
  failure_count_ = snapshot.fail_count;
  itemized_error_map_ = snapshot.itemized_error_map;
  itemized_quality_map_ = snapshot.itemized_quality_map;
}

void AbstractStateMachine::incrementMapStatItem(std::map<int16_t, PackmlStatsItemized>& itemized_map, int16_t id,
                                                float step, float duration)
{
  auto itemized_stat_it = itemized_map.find(id);
  if (itemized_stat_it != itemized_map.end())
  {
    itemized_stat_it->second.count += step;
    itemized_stat_it->second.duration += duration;
  }
  else
  {
    PackmlStatsItemized new_stats;
    new_stats.id = id;
    new_stats.count = step;
    new_stats.duration = duration;
    itemized_map.insert(std::pair<int16_t, PackmlStatsItemized>(id, new_stats));
  }
}

void AbstractStateMachine::incrementErrorStatItem(int16_t id, float step, float duration)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  incrementMapStatItem(itemized_error_map_, id, step, duration);
  incrementMapStatItem(incremental_itemized_error_map_, id, step, duration);
}

void AbstractStateMachine::incrementQualityStatItem(int16_t id, float step, float duration)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  incrementMapStatItem(itemized_quality_map_, id, step, duration);
  incrementMapStatItem(inremental_itemized_quality_map_, id, step, duration);
}

void AbstractStateMachine::incrementSuccessCount()
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  success_count_++;
  incremental_success_count_++;
}

void AbstractStateMachine::incrementFailureCount()
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  failure_count_++;
  incremental_failure_count_++;
}

void AbstractStateMachine::setIdealCycleTime(float ideal_cycle_time)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  ideal_cycle_time_ = ideal_cycle_time;
}

void AbstractStateMachine::invokeStateChangedEvent(const std::string& name, StatesEnum value)
{
  updateClock(value);
  stateChangedEvent.invoke(*this, { name, value });
}

void AbstractStateMachine::updateClock(StatesEnum new_state)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  std::chrono::duration<double> duration = std::chrono::steady_clock::now() - start_time_;
  auto elapsed_time = duration.count();
  if (duration_map_.find(current_state_) != duration_map_.end())
  {
    elapsed_time += duration_map_[current_state_];
  }
  duration_map_[current_state_] = elapsed_time;

  std::chrono::duration<double> incremental_duration = std::chrono::steady_clock::now() - incremental_start_time_;
  auto incremental_elapsed_time = incremental_duration.count();
  if (incremental_duration_map_.find(current_state_) != incremental_duration_map_.end())
  {
    incremental_elapsed_time += incremental_duration_map_[current_state_];
  }
  incremental_duration_map_[current_state_] = incremental_elapsed_time;

  current_state_ = new_state;
  start_time_ = std::chrono::steady_clock::now();
  incremental_start_time_ = std::chrono::steady_clock::now();
}

double AbstractStateMachine::getStateDuration(StatesEnum state, bool is_incremental)
{
  std::lock_guard<std::recursive_mutex> lock(stat_mutex_);
  double elapsed_time = 0;
  if (state == current_state_)
  {
    std::chrono::duration<double> duration = std::chrono::steady_clock::now() - (is_incremental ? incremental_start_time_ : start_time_);
    elapsed_time += duration.count();
  }

  auto map = is_incremental ? incremental_duration_map_ : duration_map_;
  if (map.find(state) != map.end())
  {
    elapsed_time += map[state];
  }

  return elapsed_time;
}

void AbstractStateMachine::setStateDuration(StatesEnum state, double duration)
{
  duration_map_[state] = duration;
}
}  // namespace packml_sm
