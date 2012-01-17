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
#include "core/port/tPortCreationInfo.h"
#include "core/port/tPortFlags.h"
#include "core/port/rpc/tMethodCallException.h"
#include "plugins/blackboard/tBlackboardTypeInfo.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"
#include "plugins/blackboard/tBlackboardPlugin.h"
#include "core/port/rpc/method/tPort2Method.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "core/port/rpc/method/tPort1Method.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
tBlackboardClient<T>::tBlackboardClient(const util::tString& description, core::tFrameworkElement* parent, bool push_updates, bool auto_connect, int auto_connect_category, bool read_port, bool write_port, rrlib::serialization::tDataTypeBase type) :
    wrapped(new tRawBlackboardClient(core::tPortCreationInfoBase(description, parent, InitBlackboardType(type), (write_port ? core::tPortFlags::cEMITS_DATA : 0) | (read_port ? core::tPortFlags::cACCEPTS_DATA : 0) | (push_updates ? core::tPortFlags::cPUSH_STRATEGY : 0)), static_cast<T*>(NULL), auto_connect, auto_connect_category)),
    locked(),
    read_locked(),
    write_port1(NULL),
    write_port2(NULL),
    read_port()
{
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const tAbstractBlackboardServerRaw* server, core::tFrameworkElement* parent, const util::tString& non_default_name, bool push_updates, bool read_port, bool write_port) :
    wrapped(new tRawBlackboardClient(core::tPortCreationInfoBase(non_default_name.Length() > 0 ? non_default_name : (server->GetDescription() + " Client").ToString(), parent, InitBlackboardType(rrlib::serialization::tDataType<T>()), (write_port ? core::tPortFlags::cEMITS_DATA : 0) | (read_port ? core::tPortFlags::cACCEPTS_DATA : 0) | (push_updates ? core::tPortFlags::cPUSH_STRATEGY : 0)), static_cast<T*>(NULL), false)),
    locked(),
    read_locked(),
    write_port1(NULL),
    write_port2(NULL),
    read_port()
{
  assert(server != NULL);
  if (read_port)
  {
    server->read_port_raw->ConnectToTarget(wrapped->read_port);
  }
  if (write_port)
  {
    server->write_port_raw->ConnectToTarget(wrapped->write_port->GetWrapped());
  }
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const util::tString& description, core::structure::tModuleBase* parent, bool push_updates, int create_read_port, core::tPortGroup* create_write_port_in, core::tPortGroup* create_write_port_in2) :
    wrapped(new tRawBlackboardClient(core::tPortCreationInfoBase(description, parent->GetChild("Blackboards") ? parent->GetChild("Blackboards") : new core::tFrameworkElement(parent, "Blackboards"), InitBlackboardType(rrlib::serialization::tDataType<T>()), core::tPortFlags::cEMITS_DATA | (create_read_port ? core::tPortFlags::cACCEPTS_DATA : 0) | (push_updates ? core::tPortFlags::cPUSH_STRATEGY : 0)), static_cast<T*>(NULL), false)),
    locked(),
    read_locked(),
    write_port1(NULL),
    write_port2(NULL),
    read_port()
{
  bool plain_module = (parent->GetChild("Sensor Input") == NULL);

  // Possibly create read ports in module
  if (create_read_port >= 2)
  {
    if (!plain_module)
    {
      read_port.reset(new core::structure::tSenseControlModule::tSensorInput<std::vector<T>>(description, parent, core::tPortFlags::cINPUT_PROXY));
    }
    else
    {
      read_port.reset(new core::structure::tModule::tInput<std::vector<T>>(description, parent, core::tPortFlags::cINPUT_PROXY));
    }
    wrapped->GetReadPort()->ConnectToSource(read_port->GetWrapped());
  }

  // create write/full-access ports
  core::tFrameworkElement* pg = plain_module ? parent->GetChild("Output") : parent->GetChild("Controller Output");
  write_port1 = ReplicateWritePort(wrapped->GetWritePort()->GetWrapped(), create_write_port_in != NULL ? create_write_port_in : pg, description);
  if (create_write_port_in2 != NULL)
  {
    write_port2 = ReplicateWritePort(wrapped->GetWritePort()->GetWrapped(), create_write_port_in2, description);
  }
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const tBlackboardClient& replicated_bb, core::structure::tGroup* parent, bool create_read_port_in_ci, bool forward_write_port_in_controller, bool forward_write_port_in_sensor) :
    wrapped(),
    locked(),
    read_locked(),
    write_port1(NULL),
    write_port2(NULL),
    read_port()
{
  // forward read port
  if (replicated_bb.GetOutsideReadPort())
  {
    // where do we create port?
    core::tFrameworkElement* pg = replicated_bb.GetOutsideReadPort()->GetParent();
    if (pg->DescriptionEquals("Sensor Input"))
    {
      create_read_port_in_ci = false;
    }
    else if (pg->DescriptionEquals("Controller Input"))
    {
      create_read_port_in_ci = true;
    }
    else
    {
      assert(pg->DescriptionEquals("Output") && "Have descriptions changed?");
    }

    // create port
    if (create_read_port_in_ci)
    {
      read_port.reset(new core::structure::tGroup::tControllerOutput<std::vector<T>>(replicated_bb.GetOutsideReadPort()->GetDescription()));
    }
    else
    {
      read_port.reset(new core::structure::tGroup::tSensorOutput<std::vector<T>>(replicated_bb.GetOutsideReadPort()->GetDescription()));
    }
    replicated_bb.read_port->ConnectToSource(*read_port);
  }

  // forward write ports
  std::vector<core::tAbstractPort*> new_ports;
  for (int i = 0; i < 2; i++)
  {
    core::tAbstractPort* port = (i == 0) ? replicated_bb.GetOutsideWritePort() : replicated_bb.GetOutsideWritePort2();
    if (port)
    {
      // where do we create ports?
      core::tFrameworkElement* pg = port->GetParent();
      if (pg->DescriptionEquals("Input"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetSensorInputs(), port->GetDescription()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetControllerInputs(), port->GetDescription()));
        }
      }
      else if (pg->DescriptionEquals("Output"))
      {
        if (forward_write_port_in_sensor)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetSensorOutputs(), port->GetDescription()));
        }
        if (forward_write_port_in_controller)
        {
          new_ports.push_back(ReplicateWritePort(port, &parent->GetControllerOutputs(), port->GetDescription()));
        }
      }
      else if ((pg->GetDescription().StartsWith("Sensor") && forward_write_port_in_sensor) || (pg->GetDescription().StartsWith("Controller") && forward_write_port_in_controller))
      {
        new_ports.push_back(ReplicateWritePort(port, parent->GetChild(pg->GetCDescription()), port->GetDescription()));
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

template<typename T>
void tBlackboardClient<T>::CheckConnect(core::tAbstractPort* p1, core::tAbstractPort* p2)
{
  if (p1 != NULL && p2 != NULL)
  {
    core::tFrameworkElement* parent1 = p1->GetParent();
    core::tFrameworkElement* parent2 = p2->GetParent();
    if ((parent1->GetDescription().EndsWith("Output") && parent2->GetDescription().EndsWith("Input")) ||
        (parent1->GetDescription().EndsWith("Input") && parent2->GetDescription().EndsWith("Output")))
    {
      if (!((parent1->GetDescription().StartsWith("Sensor") && parent2->GetDescription().StartsWith("Controller")) ||
            (parent1->GetDescription().StartsWith("Controller") && parent2->GetDescription().StartsWith("Sensor"))))
      {
        p1->ConnectToTarget(p2);
      }
    }
  }
}

template<typename T>
bool tBlackboardClient<T>::CommitAsynchChange(tChangeTransactionVar& change_buf, int index, int offset)
{
  try
  {
    assert(!change_buf.GetManager()->IsUnused() && "Obtain buffer from getUnusedChangeBuffer()");
    tAbstractBlackboardServer<T>::cASYNCH_CHANGE.Call(*wrapped->GetWritePort(), static_cast<tConstChangeTransactionVar&>(change_buf), index, offset, true);

    return true;
  }
  catch (const core::tMethodCallException& e)
  {
    return false;
  }
}

template<typename T>
void tBlackboardClient<T>::ConnectTo(const tBlackboard<T>& blackboard)
{
  if (blackboard.GetOutsideReadPort() && GetOutsideReadPort())
  {
    CheckConnect(blackboard.GetOutsideReadPort()->GetWrapped(), GetOutsideReadPort()->GetWrapped());
  }
  CheckConnect(blackboard.GetOutsideWritePort(), GetOutsideWritePort());
  CheckConnect(blackboard.GetOutsideWritePort2(), GetOutsideWritePort());
  CheckConnect(blackboard.GetOutsideWritePort(), GetOutsideWritePort2());
  CheckConnect(blackboard.GetOutsideWritePort2(), GetOutsideWritePort2());
}

template<typename T>
bool tBlackboardClient<T>::HasChanged() const
{
  assert((wrapped->GetReadPort() != NULL));
  if (!wrapped->GetReadPort()->GetFlag(core::tPortFlags::cPUSH_STRATEGY))
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_WARNING, "This method only works properly, when push strategy is used.");
  }
  return wrapped->GetReadPort()->HasChanged();
}

template<typename T>
rrlib::serialization::tDataTypeBase tBlackboardClient<T>::InitBlackboardType(rrlib::serialization::tDataTypeBase dt)
{
  tBlackboardTypeInfo* bti = tAbstractBlackboardServerRaw::GetBlackboardTypeInfo(dt);
  if (bti == NULL || bti->blackboard_type == NULL)
  {
    tBlackboardPlugin::RegisterBlackboardType<T>(dt);
  }
  return dt;
}

template<typename T>
void tBlackboardClient<T>::Publish(tBBVectorVar& buffer)
{
  assert((wrapped->lock_type == tRawBlackboardClient::eNONE));

  assert(!buffer.GetManager()->IsUnused() && "Obtain buffer from getUnusedBuffer()");

  /*if (buffer.getManager().isUnused()) {
      buffer.getManager().getCurrentRefCounter().setLocks((byte)1);
  }*/
  try
  {
    tAbstractBlackboardServer<T>::cDIRECT_COMMIT.Call(*wrapped->GetWritePort(), buffer, true);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "warning: Blackboard direct commit failed");
  }
}

template<typename T>
const typename tAbstractBlackboardServer<T>::tBBVector* tBlackboardClient<T>::ReadLock(bool force_read_copy_to_avoid_blocking, int timeout)
{
  assert(((locked == NULL && wrapped->lock_type == tRawBlackboardClient::eNONE)) && "Unlock first");
  if (timeout <= 0)
  {
    timeout = 60000;  // wait one minute for method to complete if no time is specified
  }

  // determine whether blackboard server is single buffered
  wrapped->CheckSingleBuffered();
  if (wrapped->server_buffers == tRawBlackboardClient::eUNKNOWN)    // we currently have no partner (?)
  {
    return NULL;
  }

  bool via_port = wrapped->GetReadPort() && ((wrapped->server_buffers == tRawBlackboardClient::eMULTI) || wrapped->GetReadPort()->PushStrategy() || force_read_copy_to_avoid_blocking || wrapped->GetWritePort()->HasRemoteServer());
  if (via_port)
  {
    wrapped->lock_type = tRawBlackboardClient::eREAD;
    wrapped->cur_lock_iD = -1;
    read_locked = Read(timeout);

    return read_locked.get();
  }
  else
  {
    assert((locked == NULL && wrapped->lock_type == tRawBlackboardClient::eNONE));
    assert((wrapped->cur_lock_iD == -1));
    assert((wrapped->IsReady()));
    try
    {
      tConstBBVectorVar ret = tAbstractBlackboardServer<T>::cREAD_LOCK.Call(*wrapped->GetWritePort(), timeout, 0, static_cast<int>((timeout + tRawBlackboardClient::cNET_TIMEOUT)));

      if (ret != NULL)
      {
        wrapped->lock_type = tRawBlackboardClient::eREAD;

        wrapped->cur_lock_iD = ret.GetManager()->lock_iD;
        read_locked = std::move(ret);

        // acknowledge lock
        wrapped->SendKeepAlive();

        return read_locked.get();
      }
      else
      {
        wrapped->cur_lock_iD = -1;
        return NULL;
      }
    }
    catch (const core::tMethodCallException& e)
    {
      wrapped->cur_lock_iD = -1;
      return NULL;
    }
  }
}

template<typename T>
void tBlackboardClient<T>::ResetChanged()
{
  assert((wrapped->GetReadPort() != NULL));
  if (!wrapped->GetReadPort()->GetFlag(core::tPortFlags::cPUSH_STRATEGY))
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_WARNING, "This method only works properly, when push strategy is used.");
  }
  wrapped->GetReadPort()->ResetChanged();
}

template<typename T>
void tBlackboardClient<T>::Unlock()
{
  if (wrapped->lock_type == tRawBlackboardClient::eNONE)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "BlackboardClient warning: nothing to unlock");
    ResetVariables();
    return;  // nothing to unlock
  }
  if (wrapped->lock_type == tRawBlackboardClient::eREAD)
  {
    // we only have a read copy
    assert((read_locked != NULL));
    if (wrapped->cur_lock_iD >= 0)
    {
      try
      {
        tAbstractBlackboardServer<T>::cREAD_UNLOCK.Call(*wrapped->GetWritePort(), wrapped->cur_lock_iD, true);
      }
      catch (const core::tMethodCallException& e)
      {
        FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "warning: Unlocking blackboard (read) failed", e);
      }
    }

    ResetVariables();
    return;
  }

  assert((wrapped->lock_type == tRawBlackboardClient::eWRITE));
  assert((wrapped->cur_lock_iD >= 0));

  try
  {
    tAbstractBlackboardServer<T>::cUNLOCK.Call(*wrapped->GetWritePort(), locked, true);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "warning: Unlocking blackboard failed");
    //e.printStackTrace();
  }
  ResetVariables();
}

template<typename T>
typename tAbstractBlackboardServer<T>::tBBVector* tBlackboardClient<T>::WriteLock(int timeout)
{
  if (timeout <= 0)
  {
    timeout = 60000;  // wait one minute for method to complete if no time is specified
  }

  assert((locked == NULL && wrapped->lock_type == tRawBlackboardClient::eNONE));
  assert((wrapped->cur_lock_iD == -1));
  assert((wrapped->IsReady()));
  try
  {
    tBBVectorVar ret = tAbstractBlackboardServer<T>::cLOCK.Call(*wrapped->GetWritePort(), timeout, static_cast<int>((timeout + tRawBlackboardClient::cNET_TIMEOUT)));

    if (ret != NULL)
    {
      wrapped->lock_type = tRawBlackboardClient::eWRITE;

      wrapped->cur_lock_iD = ret.GetManager()->lock_iD;
      locked = std::move(ret);

      // acknowledge lock
      wrapped->SendKeepAlive();
    }
    else
    {
      wrapped->cur_lock_iD = -1;
    }

    return locked.Get();

  }
  catch (const core::tMethodCallException& e)
  {
    wrapped->cur_lock_iD = -1;
    return NULL;
  }
}

} // namespace finroc
} // namespace blackboard

