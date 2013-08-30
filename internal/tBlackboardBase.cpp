//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/blackboard/internal/tBlackboardBase.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 */
//----------------------------------------------------------------------
#include "plugins/blackboard/internal/tBlackboardBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/algorithm/string.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace blackboard
{
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

core::tPortGroup* tBlackboardBase::default_port_group = (core::tPortGroup*)1;

tBlackboardBase::tBlackboardBase() :
  write_port1(NULL),
  write_port2(NULL)
{}

tBlackboardBase::tBlackboardBase(const tBlackboardBase& replicated_bb, structure::tSenseControlGroup* parent, bool create_read_port_in_co, bool forward_write_port_in_controller, bool forward_write_port_in_sensor) :
  write_port1(NULL),
  write_port2(NULL)
{
  // forward write ports
  std::vector<core::tAbstractPort*> new_ports;
  for (int i = 0; i < 2; i++)
  {
    core::tAbstractPort* port = (i == 0) ? replicated_bb.write_port1 : replicated_bb.write_port2;
    if (port)
    {
      // where do we create ports?
      core::tFrameworkElement* pg = port->GetParent();
      if (pg->NameEquals("Input"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(*port, parent->GetSensorInputs(), port->GetName()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(*port, parent->GetControllerInputs(), port->GetName()));
        }
      }
      else if (pg->NameEquals("Output"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(*port, parent->GetSensorOutputs(), port->GetName()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(*port, parent->GetControllerOutputs(), port->GetName()));
        }
      }
      else if ((boost::starts_with(pg->GetName(), "Sensor") && forward_write_port_in_sensor) || (boost::starts_with(pg->GetName(), "Controller") && forward_write_port_in_controller))
      {
        new_ports.push_back(ReplicateWritePort(*port, parent->GetInterface(pg->GetName()), port->GetName()));
      }
    }
  }
  assert(new_ports.size() <= 2);
  if (new_ports.size() >= 1)
  {
    write_port1 = new_ports[0];
  }
  if (new_ports.size() >= 2)
  {
    write_port2 = new_ports[1];
  }
}

tBlackboardBase::tBlackboardBase(tBlackboardBase && other) :
  write_port1(NULL),
  write_port2(NULL)
{
  std::swap(write_port1, other.write_port1);
  std::swap(write_port2, other.write_port2);
}

tBlackboardBase& tBlackboardBase::operator=(tBlackboardBase && other)
{
  std::swap(write_port1, other.write_port1);
  std::swap(write_port2, other.write_port2);
  return *this;
}

core::tAbstractPort* tBlackboardBase::ReplicateWritePort(core::tAbstractPort& write_port, core::tFrameworkElement& port_group, const std::string& name)
{
  core::tAbstractPortCreationInfo creation_info;
  creation_info.flags = core::tFrameworkElement::tFlag::ACCEPTS_DATA | core::tFrameworkElement::tFlag::EMITS_DATA;
  if (typeid(port_group) == typeid(core::tPortGroup))
  {
    creation_info.flags |= static_cast<core::tPortGroup&>(port_group).GetDefaultPortFlags();
  }
  creation_info.data_type = write_port.GetDataType();
  creation_info.parent = &port_group;
  creation_info.name = name;
  rpc_ports::internal::tRPCPort* new_port = new rpc_ports::internal::tRPCPort(creation_info, NULL);
  write_port.ConnectTo(*new_port);
  return new_port;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
