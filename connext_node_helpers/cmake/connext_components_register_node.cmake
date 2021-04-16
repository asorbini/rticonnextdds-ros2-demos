# (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

################################################################################
# Similar to `rclcpp_components_register_node()` but generates a main() which
# links Connext DDS directly, and which is linked with `-rdynamic` to enable
# sharing of the DomainParticipantFactory and other entities with the RMW.
################################################################################
function(connext_components_register_node target)
  cmake_parse_arguments(_component
    "" # boolean arguments
    "PLUGIN;EXECUTABLE;RESOURCE_INDEX" # single value arguments
    "" # multi-value arguments
    ${ARGN} # current function arguments
    )

  # use a custom template to generate each node's main, because we must make
  # sure that the DomainParticipantFactory's global static variable is linked
  # into the process. We do this by calling some DDS API that accesses it.
  set(rclcpp_components_NODE_TEMPLATE ${connext_node_helpers_NODE_TEMPLATE})

  rclcpp_components_register_node(${target}
    PLUGIN ${_component_PLUGIN}
    EXECUTABLE ${_component_EXECUTABLE}
    RESOURCE_INDEX ${_component_RESOURCE_INDEX})

  set_target_properties(${_component_EXECUTABLE} PROPERTIES ENABLE_EXPORTS true)
  target_link_libraries(${_component_EXECUTABLE} RTIConnextDDS::cpp2_api)
endfunction()
