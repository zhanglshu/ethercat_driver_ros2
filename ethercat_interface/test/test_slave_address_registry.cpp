// Copyright 2026 ICUBE Laboratory, University of Strasbourg
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

#include <gtest/gtest.h>
#include <stdexcept>

#include "ethercat_interface/slave_address_registry.hpp"

namespace ethercat_interface
{
namespace
{

TEST(SlaveAddressRegistry, DistinguishesAliasesThatShareRelativePositionZero)
{
  SlaveAddressRegistry registry;
  registry.add({1U, 0U});
  registry.add({2U, 0U});
  registry.add({6U, 0U});

  EXPECT_EQ(registry.size(), 3U);
  EXPECT_TRUE(registry.contains({1U, 0U}));
  EXPECT_TRUE(registry.contains({2U, 0U}));
  EXPECT_TRUE(registry.contains({6U, 0U}));
}

TEST(SlaveAddressRegistry, RejectsDuplicateAliasPositionPairs)
{
  SlaveAddressRegistry registry;
  registry.add({3U, 0U});
  EXPECT_THROW(registry.add({3U, 0U}), std::invalid_argument);
}

TEST(SlaveAddressRegistry, RejectsZeroAliasForAliasBasedTopology)
{
  SlaveAddressRegistry registry;
  EXPECT_THROW(registry.add({0U, 0U}), std::invalid_argument);
}

}  // namespace
}  // namespace ethercat_interface
