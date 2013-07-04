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
/*!\file    plugins/blackboard/tBlackboardClient.hpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/structure/tSenseControlModule.h"
#include <boost/algorithm/string.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboardWriteAccess.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace blackboard
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

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const std::string& name, core::tFrameworkElement* parent, bool push_updates, tAutoConnectMode auto_connect_mode, bool create_read_port) :
  read_port(PossiblyCreateReadPort(create_read_port, push_updates)),
  write_port("write", internal::tBlackboardServer<T>::GetRPCInterfaceType()),
  backend(new internal::tBlackboardClientBackend(name, parent, write_port, read_port)),
  outside_write_port1(),
  outside_write_port2(),
  outside_read_port()
{
  backend->SetAutoConnectMode(auto_connect_mode);
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(internal::tBlackboardServer<T>& server, core::tFrameworkElement* parent, const std::string& non_default_name, bool push_updates, bool create_read_port) :
  read_port(PossiblyCreateReadPort(create_read_port, push_updates)),
  write_port("write", internal::tBlackboardServer<T>::GetRPCInterfaceType()),
  backend(new internal::tBlackboardClientBackend(non_default_name.length() > 0 ? non_default_name : (server.GetName() + " Client"), parent, write_port, read_port)),
  outside_write_port1(),
  outside_write_port2(),
  outside_read_port()
{
  if (create_read_port)
  {
    server.GetReadPort().ConnectTo(read_port);
  }
  server.GetWritePort().ConnectTo(write_port);
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const std::string& name, structure::tModuleBase* parent, bool push_updates, tReadPorts create_read_port, core::tPortGroup* create_write_port_in, core::tPortGroup* create_write_port_in2) :
  read_port(PossiblyCreateReadPort(create_read_port != tReadPorts::NONE, push_updates)),
  write_port("write", internal::tBlackboardServer<T>::GetRPCInterfaceType()),
  backend(new internal::tBlackboardClientBackend(name, parent->GetChild("Blackboards") ? parent->GetChild("Blackboards") : new core::tFrameworkElement(parent, "Blackboards"), write_port, read_port)),
  outside_write_port1(),
  outside_write_port2(),
  outside_read_port()
{
  structure::tModule* module = dynamic_cast<structure::tModule*>(parent);
  structure::tSenseControlModule* sense_control_module = dynamic_cast<structure::tSenseControlModule*>(parent);
  assert((module == NULL || sense_control_module == NULL) && (module || sense_control_module));

  // Possibly create read ports in module
  if (create_read_port == tReadPorts::EXTERNAL)
  {
    if (sense_control_module)
    {
      outside_read_port = structure::tSenseControlModule::tSensorInput<tBuffer>(name, parent, core::tFrameworkElement::tFlag::EMITS_DATA);
    }
    else
    {
      outside_read_port = structure::tModule::tInput<tBuffer>(name, parent, core::tFrameworkElement::tFlag::EMITS_DATA);
    }
    read_port.ConnectTo(outside_read_port);
  }

  // create write/full-access ports
  core::tFrameworkElement& port_group = module ? module->GetOutputs() : sense_control_module->GetControllerOutputs();
  outside_write_port1 = ReplicateWritePort(write_port, create_write_port_in != NULL ? create_write_port_in : &port_group, name);
  if (create_write_port_in2 != NULL)
  {
    outside_write_port2 = ReplicateWritePort(write_port, create_write_port_in2, name);
  }
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const tBlackboardClient& replicated_bb, structure::tGroup* parent, bool create_read_port_in_ci, bool forward_write_port_in_controller, bool forward_write_port_in_sensor) :
  read_port(),
  write_port(),
  backend(NULL),
  outside_write_port1(),
  outside_write_port2(),
  outside_read_port()
{
  // forward read port
  if (replicated_bb.GetOutsideReadPort().GetWrapped())
  {
    // where do we create port?
    core::tFrameworkElement* port_group = replicated_bb.GetOutsideReadPort().GetParent();
    if (port_group->NameEquals("Sensor Input"))
    {
      create_read_port_in_ci = false;
    }
    else if (port_group->NameEquals("Controller Input"))
    {
      create_read_port_in_ci = true;
    }

    // create port
    if (create_read_port_in_ci)
    {
      outside_read_port = structure::tGroup::tControllerInput<tBuffer>(replicated_bb.GetOutsideReadPort().GetName(), parent);
    }
    else
    {
      outside_read_port = structure::tGroup::tSensorInput<tBuffer>(replicated_bb.GetOutsideReadPort().GetName(), parent);
    }
    replicated_bb.GetOutsideReadPort().ConnectTo(outside_read_port);
  }

  // forward write ports
  std::vector<rpc_ports::tProxyPort<tServer, false>> new_ports;
  for (int i = 0; i < 2; i++)
  {
    rpc_ports::tProxyPort<tServer, false> port = (i == 0) ? replicated_bb.GetOutsideWritePort() : replicated_bb.GetOutsideWritePort2();
    if (port.GetWrapped())
    {
      // where do we create ports?
      core::tFrameworkElement* port_group = port.GetParent();
      if (port_group->NameEquals("Input"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetSensorInputs(), port.GetName()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetControllerInputs(), port.GetName()));
        }
      }
      else if (port_group->NameEquals("Output"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetSensorOutputs(), port.GetName()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetControllerOutputs(), port.GetName()));
        }
      }
      else if ((boost::starts_with(port_group->GetName(), "Sensor") && forward_write_port_in_sensor) || (boost::starts_with(port_group->GetName(), "Controller") && forward_write_port_in_controller))
      {
        new_ports.push_back(ReplicateWritePort(port, &parent->GetInterface(port_group->GetCName()), port.GetName()));
      }
    }
  }
  assert(new_ports.size() <= 2);
  if (new_ports.size() >= 1)
  {
    outside_write_port1 = new_ports[0];
  }
  if (new_ports.size() >= 2)
  {
    outside_write_port2 = new_ports[1];
  }
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(tBlackboardClient && o) :
  read_port(),
  write_port(),
  backend(NULL),
  outside_write_port1(),
  outside_write_port2(),
  outside_read_port()
{
  std::swap(read_port, o.read_port);
  std::swap(write_port, o.write_port);
  std::swap(backend, o.backend);
  std::swap(outside_write_port1, o.outside_write_port1);
  std::swap(outside_write_port2, o.outside_write_port2);
  std::swap(outside_read_port, o.outside_read_port);
}

template<typename T>
tBlackboardClient<T>& tBlackboardClient<T>::operator=(tBlackboardClient && o)
{
  std::swap(read_port, o.read_port);
  std::swap(write_port, o.write_port);
  std::swap(backend, o.backend);
  std::swap(outside_write_port1, o.outside_write_port1);
  std::swap(outside_write_port2, o.outside_write_port2);
  std::swap(outside_read_port, o.outside_read_port);
  return *this;
}


template<typename T>
void tBlackboardClient<T>::CheckConnect(core::tPortWrapperBase p1, core::tPortWrapperBase p2)
{
  if (p1.GetWrapped() && p2.GetWrapped())
  {
    core::tFrameworkElement* parent1 = p1.GetParent();
    core::tFrameworkElement* parent2 = p2.GetParent();
    if ((boost::ends_with(parent1->GetName(), "Output") && boost::ends_with(parent2->GetName(), "Input")) ||
        (boost::ends_with(parent1->GetName(), "Input") && boost::ends_with(parent2->GetName(), "Output")))
    {
      if (!((boost::starts_with(parent1->GetName(), "Sensor") && boost::starts_with(parent2->GetName(), "Controller")) ||
            (boost::starts_with(parent1->GetName(), "Controller") && boost::starts_with(parent2->GetName(), "Sensor"))))
      {
        p1.ConnectTo(p2);
      }
    }
  }
}

template<typename T>
void tBlackboardClient<T>::CheckClientConnect(core::tPortWrapperBase p1, core::tPortWrapperBase p2)
{
  if (p1.GetWrapped() && p2.GetWrapped())
  {
    core::tFrameworkElement* parent1 = p1.GetParent();
    core::tFrameworkElement* parent2 = p2.GetParent();
    if ((boost::ends_with(parent1->GetName(), "Input") && boost::ends_with(parent2->GetName(), "Input")) ||
        (boost::ends_with(parent1->GetName(), "Output") && boost::ends_with(parent2->GetName(), "Output")))
    {
      if (!((boost::starts_with(parent1->GetName(), "Sensor") && boost::starts_with(parent2->GetName(), "Controller")) ||
            (boost::starts_with(parent1->GetName(), "Controller") && boost::starts_with(parent2->GetName(), "Sensor"))))
      {
        p1.ConnectTo(p2);
      }
    }
  }
}

template<typename T>
void tBlackboardClient<T>::ConnectTo(const tBlackboard<T>& blackboard)
{
  if (blackboard.GetOutsideReadPort().GetWrapped() && GetOutsideReadPort().GetWrapped())
  {
    CheckConnect(blackboard.GetOutsideReadPort(), GetOutsideReadPort());
  }
  CheckConnect(core::tPortWrapperBase(blackboard.GetOutsideWritePort()), GetOutsideWritePort());
  CheckConnect(core::tPortWrapperBase(blackboard.GetOutsideWritePort2()), GetOutsideWritePort());
  CheckConnect(core::tPortWrapperBase(blackboard.GetOutsideWritePort()), GetOutsideWritePort2());
  CheckConnect(core::tPortWrapperBase(blackboard.GetOutsideWritePort2()), GetOutsideWritePort2());
}

template<typename T>
void tBlackboardClient<T>::ConnectTo(const tBlackboardClient<T>& client)
{
  if (client.GetOutsideReadPort().GetWrapped() && GetOutsideReadPort().GetWrapped())
  {
    CheckClientConnect(client.GetOutsideReadPort(), GetOutsideReadPort());
  }
  CheckClientConnect(client.GetOutsideWritePort(), GetOutsideWritePort());
  CheckClientConnect(client.GetOutsideWritePort2(), GetOutsideWritePort());
  CheckClientConnect(client.GetOutsideWritePort(), GetOutsideWritePort2());
  CheckClientConnect(client.GetOutsideWritePort2(), GetOutsideWritePort2());
}

template<typename T>
bool tBlackboardClient<T>::HasChanged() const
{
  if (!read_port.GetWrapped())
  {
    FINROC_LOG_PRINT(DEBUG_WARNING, "This method only works if read port is created.");
    return false;
  }
  if (!read_port.GetFlag(core::tFrameworkElement::tFlag::PUSH_STRATEGY))
  {
    FINROC_LOG_PRINT(DEBUG_WARNING, "This method only works properly, when push strategy is used.");
  }
  return read_port.HasChanged();
}

template<typename T>
rpc_ports::tFuture<typename tBlackboardClient<T>::tConstBufferPointer> tBlackboardClient<T>::ReadLock(const rrlib::time::tDuration& timeout)
{
  // Possibly obtain value from read_port
  if (read_port.GetWrapped() && read_port.GetFlag(core::tFrameworkElement::tFlag::PUSH_STRATEGY))
  {
    rpc_ports::tPromise<tConstBufferPointer> promise;
    promise.SetValue(read_port.GetPointer());
    return promise.GetFuture();
  }

  return write_port.NativeFutureCall(&tServer::ReadLock, timeout);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
