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

#ifndef ETHERCAT_INTERFACE__SLAVE_ADDRESS_REGISTRY_HPP_
#define ETHERCAT_INTERFACE__SLAVE_ADDRESS_REGISTRY_HPP_

#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>

namespace ethercat_interface
{

struct SlaveAddress
{
  std::uint16_t alias{0U};
  std::uint16_t position{0U};

  bool operator<(const SlaveAddress & other) const noexcept
  {
    return alias < other.alias || (alias == other.alias && position < other.position);
  }
};

class SlaveAddressRegistry
{
public:
  void add(const SlaveAddress address)
  {
    if (address.alias == 0U) {
      throw std::invalid_argument("station alias must be non-zero for alias-based topology");
    }
    if (!addresses_.insert(address).second) {
      throw std::invalid_argument("duplicate EtherCAT alias/position address");
    }
  }

  bool contains(const SlaveAddress address) const noexcept
  {
    return addresses_.find(address) != addresses_.end();
  }

  std::size_t size() const noexcept {return addresses_.size();}

private:
  std::set<SlaveAddress> addresses_;
};

}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__SLAVE_ADDRESS_REGISTRY_HPP_
