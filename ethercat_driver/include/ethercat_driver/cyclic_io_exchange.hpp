// Copyright (c) 2026
// SPDX-License-Identifier: Apache-2.0
#ifndef ETHERCAT_DRIVER__CYCLIC_IO_EXCHANGE_HPP_
#define ETHERCAT_DRIVER__CYCLIC_IO_EXCHANGE_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <vector>

namespace ethercat_driver
{
// Exactly one producer and one consumer. Each owns a slot; the third slot is
// handed over by one atomic exchange. The producer can replace unread frames
// but can never touch the consumer's slot. No retries, locks or waits.
template<class Frame>
class SpscLatestFrame
{
public:
  void initialize(const Frame & initial)
  {
    for (auto & frame : frames_) {frame = initial;}
    producer_ = 0;
    consumer_ = 1;
    shared_.store(2, std::memory_order_relaxed);
  }

  Frame & producer_buffer() noexcept {return frames_[producer_];}
  const Frame & consumer_buffer() const noexcept {return frames_[consumer_];}
  void publish() noexcept
  {
    producer_ = shared_.exchange(producer_ | kDirty, std::memory_order_acq_rel) & kIndex;
  }
  bool consume() noexcept
  {
    if (!(shared_.load(std::memory_order_acquire) & kDirty)) {return false;}
    consumer_ = shared_.exchange(consumer_, std::memory_order_acq_rel) & kIndex;
    return true;
  }

private:
  static_assert(ATOMIC_INT_LOCK_FREE == 2, "Cyclic IO requires lock-free unsigned atomics");
  static constexpr unsigned kDirty = 4, kIndex = 3;
  std::array<Frame, 3> frames_{};
  alignas(64) std::atomic<unsigned> shared_{2};
  alignas(64) unsigned producer_{0};
  alignas(64) unsigned consumer_{1};
};

using InterfaceGroups = std::vector<std::vector<double>>;
struct InterfaceFrame
{
  InterfaceGroups joints, sensors, gpios;
};

class CyclicIoExchange
{
public:
  // Non-RT only, before slave setup and before either cyclic thread starts.
  // Shapes and addresses remain fixed until both threads have stopped.
  void initialize(const InterfaceFrame & commands, const InterfaceFrame & states)
  {
    bus_commands_ = commands;
    bus_states_ = states;
    commands_.initialize(commands);
    states_.initialize(states);
  }

  // Controller thread: commit once, after the entire update() has finished.
  void publish_commands(
    const InterfaceGroups & joints, const InterfaceGroups & sensors,
    const InterfaceGroups & gpios) noexcept
  {
    auto & frame = commands_.producer_buffer();
    copy_group(joints, frame.joints);
    copy_group(sensors, frame.sensors);
    copy_group(gpios, frame.gpios);
    commands_.publish();
  }

  // EtherCAT thread: latch once before processing any slave's PDO channels.
  void latch_commands() noexcept
  {
    if (commands_.consume()) {copy_frame(commands_.consumer_buffer(), bus_commands_);}
  }
  InterfaceFrame & bus_commands() noexcept {return bus_commands_;}
  InterfaceFrame & bus_states() noexcept {return bus_states_;}

  // EtherCAT thread: publish only after ALL slaves' feedback has been read.
  void publish_states() noexcept
  {
    copy_frame(bus_states_, states_.producer_buffer());
    states_.publish();
  }

  // Controller thread: stable feedback throughout the subsequent update().
  void read_states(
    InterfaceGroups & joints, InterfaceGroups & sensors, InterfaceGroups & gpios) noexcept
  {
    if (!states_.consume()) {return;}
    const auto & frame = states_.consumer_buffer();
    copy_group(frame.joints, joints);
    copy_group(frame.sensors, sensors);
    copy_group(frame.gpios, gpios);
  }

private:
  static void copy_group(const InterfaceGroups & from, InterfaceGroups & to) noexcept
  {
    for (std::size_t i = 0; i < from.size(); ++i) {
      std::copy(from[i].begin(), from[i].end(), to[i].begin());
    }
  }
  static void copy_frame(const InterfaceFrame & from, InterfaceFrame & to) noexcept
  {
    copy_group(from.joints, to.joints);
    copy_group(from.sensors, to.sensors);
    copy_group(from.gpios, to.gpios);
  }
  InterfaceFrame bus_commands_, bus_states_;
  SpscLatestFrame<InterfaceFrame> commands_, states_;
};
}  // namespace ethercat_driver
#endif  // ETHERCAT_DRIVER__CYCLIC_IO_EXCHANGE_HPP_
