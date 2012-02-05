/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2007-2010 Max Reichardt,
 *   Robotics Research Lab, University of Kaiserslautern
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include "plugins/blackboard/tBlackboardClient.h"
#include "plugins/blackboard/tBlackboard.h"

namespace finroc
{
namespace blackboard
{
namespace internal
{
core::tPortGroup* tBlackboardBase::default_port_group = (core::tPortGroup*)1;

tBlackboardBase::tBlackboardBase(const tBlackboardBase& replicated_bb, core::structure::tGroup* parent, bool create_read_port_in_co, bool forward_write_port_in_controller, bool forward_write_port_in_sensor) :
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
          new_ports.push_back(ReplicateWritePort(port, &parent->GetSensorInputs(), port->GetName()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetControllerInputs(), port->GetName()));
        }
      }
      else if (pg->NameEquals("Output"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetSensorOutputs(), port->GetName()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetControllerOutputs(), port->GetName()));
        }
      }
      else if ((pg->GetName().StartsWith("Sensor") && forward_write_port_in_sensor) || (pg->GetName().StartsWith("Controller") && forward_write_port_in_controller))
      {
        new_ports.push_back(ReplicateWritePort(port, parent->GetChild(pg->GetCName()), port->GetName()));
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

core::tAbstractPort* tBlackboardBase::ReplicateWritePort(core::tAbstractPort* write_port, core::tFrameworkElement* pg, const util::tString& name)
{
  core::tInterfacePort* new_port = new core::tInterfacePort(name, pg, write_port->GetDataType(), core::tInterfacePort::eRouting);
  write_port->ConnectToTarget(new_port);
  return new_port;
}


} // namespace internal
} // namespace finroc
} // namespace blackboard

