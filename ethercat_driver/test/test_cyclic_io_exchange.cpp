// Copyright (c) 2026
// SPDX-License-Identifier: Apache-2.0
#include <array>
#include <atomic>
#include <cstdlib>
#include <new>
#include <thread>
#include <gtest/gtest.h>
#include "ethercat_driver/cyclic_io_exchange.hpp"

namespace
{
thread_local bool count_allocations = false;
thread_local unsigned allocations = 0;
}

void * operator new(std::size_t size)
{
  if (count_allocations) {++allocations;}
  if (void * result = std::malloc(size ? size : 1)) {return result;}
  throw std::bad_alloc();
}
void * operator new[](std::size_t size) {return ::operator new(size);}
void operator delete(void * value) noexcept {std::free(value);}
void operator delete[](void * value) noexcept {std::free(value);}
#if __cplusplus >= 201402L
void operator delete(void * value, std::size_t) noexcept {std::free(value);}
void operator delete[](void * value, std::size_t) noexcept {std::free(value);}
#endif

namespace ethercat_driver
{
namespace
{
InterfaceFrame frame(double value)
{
  return {{{value, value, value}, {value, value}}, {{value}}, {{value, value}}};
}

TEST(CyclicIoExchange, OnlyCommittedCommandsReachBusAndRemainPinnedForWholeCycle)
{
  auto commands = frame(70.0);
  auto states = frame(0.0);
  CyclicIoExchange exchange;
  exchange.initialize(commands, states);
  commands.joints[0][0] = 0.0;  // Old sampler's temporary target, not committed.
  exchange.latch_commands();
  EXPECT_DOUBLE_EQ(exchange.bus_commands().joints[0][0], 70.0);
  commands.joints[0][0] = 70.0;
  exchange.publish_commands(commands.joints, commands.sensors, commands.gpios);
  exchange.latch_commands();
  EXPECT_DOUBLE_EQ(exchange.bus_commands().joints[0][0], 70.0);
  commands = frame(71.0);
  exchange.publish_commands(commands.joints, commands.sensors, commands.gpios);
  EXPECT_DOUBLE_EQ(exchange.bus_commands().joints[0][0], 70.0);
  EXPECT_DOUBLE_EQ(exchange.bus_commands().gpios[0][1], 70.0);
  exchange.latch_commands();
  EXPECT_DOUBLE_EQ(exchange.bus_commands().joints[0][0], 71.0);
  EXPECT_DOUBLE_EQ(exchange.bus_commands().gpios[0][1], 71.0);
}

TEST(CyclicIoExchange, FeedbackIsPublishedOnlyAfterAllFieldsAreUpdated)
{
  auto commands = frame(0.0);
  auto states = frame(1.0);
  CyclicIoExchange exchange;
  exchange.initialize(commands, states);
  exchange.bus_states().joints[0][0] = 2.0;
  exchange.read_states(states.joints, states.sensors, states.gpios);
  EXPECT_DOUBLE_EQ(states.joints[0][0], 1.0);
  exchange.bus_states() = frame(2.0);
  exchange.publish_states();
  exchange.read_states(states.joints, states.sensors, states.gpios);
  EXPECT_DOUBLE_EQ(states.joints[0][0], 2.0);
  EXPECT_DOUBLE_EQ(states.gpios[0][1], 2.0);
  exchange.bus_states().joints[0][0] = 3.0;
  exchange.read_states(states.joints, states.sensors, states.gpios);
  EXPECT_DOUBLE_EQ(states.joints[0][0], 2.0);
}

TEST(CyclicIoExchange, ProducerCannotReuseFrameStillOwnedByConsumer)
{
  SpscLatestFrame<std::array<unsigned, 256>> buffer;
  buffer.initialize({});
  buffer.producer_buffer().fill(1);
  buffer.publish();
  ASSERT_TRUE(buffer.consume());
  const auto & pinned = buffer.consumer_buffer();
  for (unsigned i = 2; i < 10000; ++i) {
    buffer.producer_buffer().fill(i);
    buffer.publish();
  }
  for (unsigned value : pinned) {EXPECT_EQ(value, 1U);}
  ASSERT_TRUE(buffer.consume());
  for (unsigned value : buffer.consumer_buffer()) {EXPECT_EQ(value, 9999U);}
  EXPECT_FALSE(buffer.consume());
}

TEST(CyclicIoExchange, CyclicOperationsNeverAllocateAfterInitialization)
{
  auto commands = frame(70.0), states = frame(1.0);
  CyclicIoExchange exchange;
  exchange.initialize(commands, states);
  allocations = 0;
  count_allocations = true;
  for (unsigned i = 0; i < 10000; ++i) {
    exchange.publish_commands(commands.joints, commands.sensors, commands.gpios);
    exchange.latch_commands();
    exchange.publish_states();
    exchange.read_states(states.joints, states.sensors, states.gpios);
  }
  count_allocations = false;
  EXPECT_EQ(allocations, 0U);
}

TEST(CyclicIoExchange, ConcurrentReadersNeverSeeMixedGenerationInEitherDirection)
{
  CyclicIoExchange exchange;
  const auto initial = frame(0.0);
  exchange.initialize(initial, initial);
  std::atomic<bool> done{false}, failed{false};
  auto all_equal = [](const InterfaceFrame & value, double expected) {
      for (const auto * group : {&value.joints, &value.sensors, &value.gpios}) {
        for (const auto & fields : *group) {
          for (double field : fields) {if (field != expected) {return false;}}
        }
      }
      return true;
    };
  auto fill = [](InterfaceFrame & value, double generation) {
      for (auto * group : {&value.joints, &value.sensors, &value.gpios}) {
        for (auto & fields : *group) {
          for (double & field : fields) {field = generation;}
        }
      }
    };
  std::thread bus([&]() {
      unsigned generation = 0;
      do {
        exchange.latch_commands();
        const auto & received = exchange.bus_commands();
        if (!all_equal(received, received.joints[0][0])) {failed.store(true);}
        fill(exchange.bus_states(), ++generation);
        exchange.publish_states();
      } while (!done.load(std::memory_order_acquire));
    });
  auto commands = initial, received = initial;
  for (unsigned generation = 1; generation <= 200000; ++generation) {
    fill(commands, generation);
    exchange.publish_commands(commands.joints, commands.sensors, commands.gpios);
    exchange.read_states(received.joints, received.sensors, received.gpios);
    if (!all_equal(received, received.joints[0][0])) {failed.store(true);}
  }
  done.store(true, std::memory_order_release);
  bus.join();
  EXPECT_FALSE(failed.load());
  exchange.latch_commands();
  EXPECT_TRUE(all_equal(exchange.bus_commands(), 200000.0));
}
}  // namespace
}  // namespace ethercat_driver
