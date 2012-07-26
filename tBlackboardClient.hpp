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
#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "plugins/blackboard/tBlackboardPlugin.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
tBlackboardClient<T>::tBlackboardClient(const util::tString& name, core::tFrameworkElement* parent, bool push_updates, bool auto_connect, int auto_connect_category, bool read_port, bool write_port, rrlib::rtti::tDataTypeBase type) :
  wrapped(new tRawBlackboardClient(core::tPortCreationInfoBase(name, parent, InitBlackboardType(type), (write_port ? core::tPortFlags::cEMITS_DATA : 0) | (read_port ? core::tPortFlags::cACCEPTS_DATA : 0) | (push_updates ? core::tPortFlags::cPUSH_STRATEGY : 0)), static_cast<T*>(NULL), auto_connect, auto_connect_category)),
  locked(),
  read_locked(),
  write_port1(NULL),
  write_port2(NULL),
  read_port()
{
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const tAbstractBlackboardServerRaw* server, core::tFrameworkElement* parent, const util::tString& non_default_name, bool push_updates, bool read_port, bool write_port) :
  wrapped(new tRawBlackboardClient(core::tPortCreationInfoBase(non_default_name.length() > 0 ? non_default_name : (server->GetName() + " Client"), parent, InitBlackboardType(rrlib::rtti::tDataType<T>()), (write_port ? core::tPortFlags::cEMITS_DATA : 0) | (read_port ? core::tPortFlags::cACCEPTS_DATA : 0) | (push_updates ? core::tPortFlags::cPUSH_STRATEGY : 0)), static_cast<T*>(NULL), false)),
  locked(),
  read_locked(),
  write_port1(NULL),
  write_port2(NULL),
  read_port()
{
  assert(server != NULL);
  if (read_port)
  {
    server->read_port_raw->ConnectTo(*wrapped->read_port);
  }
  if (write_port)
  {
    server->write_port_raw->ConnectTo(*wrapped->write_port->GetWrapped());
  }
}

template<typename T>
tBlackboardClient<T>::tBlackboardClient(const util::tString& name, core::structure::tModuleBase* parent, bool push_updates, int create_read_port, core::tPortGroup* create_write_port_in, core::tPortGroup* create_write_port_in2) :
  wrapped(new tRawBlackboardClient(core::tPortCreationInfoBase(name, parent->GetChild("Blackboards") ? parent->GetChild("Blackboards") : new core::tFrameworkElement(parent, "Blackboards"), InitBlackboardType(rrlib::rtti::tDataType<T>()), core::tPortFlags::cEMITS_DATA | (create_read_port ? core::tPortFlags::cACCEPTS_DATA : 0) | (push_updates ? core::tPortFlags::cPUSH_STRATEGY : 0)), static_cast<T*>(NULL), false)),
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
      read_port.reset(new core::structure::tSenseControlModule::tSensorInput<std::vector<T>>(name, parent, core::tPortFlags::cINPUT_PROXY));
    }
    else
    {
      read_port.reset(new core::structure::tModule::tInput<std::vector<T>>(name, parent, core::tPortFlags::cINPUT_PROXY));
    }
    wrapped->GetReadPort()->ConnectTo(*read_port->GetWrapped());
  }

  // create write/full-access ports
  core::tFrameworkElement* pg = plain_module ? parent->GetChild("Output") : parent->GetChild("Controller Output");
  write_port1 = ReplicateWritePort(wrapped->GetWritePort()->GetWrapped(), create_write_port_in != NULL ? create_write_port_in : pg, name);
  if (create_write_port_in2 != NULL)
  {
    write_port2 = ReplicateWritePort(wrapped->GetWritePort()->GetWrapped(), create_write_port_in2, name);
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
    if (pg->NameEquals("Sensor Input"))
    {
      create_read_port_in_ci = false;
    }
    else if (pg->NameEquals("Controller Input"))
    {
      create_read_port_in_ci = true;
    }

    // create port
    if (create_read_port_in_ci)
    {
      read_port.reset(new core::structure::tGroup::tControllerInput<std::vector<T>>(replicated_bb.GetOutsideReadPort()->GetName(), parent));
    }
    else
    {
      read_port.reset(new core::structure::tGroup::tSensorInput<std::vector<T>>(replicated_bb.GetOutsideReadPort()->GetName(), parent));
    }
    replicated_bb.read_port->ConnectTo(*read_port);
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
      else if ((boost::starts_with(pg->GetName(), "Sensor") && forward_write_port_in_sensor) || (boost::starts_with(pg->GetName(), "Controller") && forward_write_port_in_controller))
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

template<typename T>
tBlackboardClient<T>::tBlackboardClient(tBlackboardClient && o) :
  wrapped(NULL),
  locked(),
  read_locked(),
  write_port1(NULL),
  write_port2(NULL),
  read_port()
{
  std::swap(wrapped, o.wrapped);
  std::swap(locked, o.locked);
  std::swap(read_locked, o.read_locked);
  std::swap(read_port, o.read_port);
  std::swap(write_port1, o.write_port1);
  std::swap(write_port2, o.write_port2);
}

template<typename T>
void tBlackboardClient<T>::CheckConnect(core::tAbstractPort* p1, core::tAbstractPort* p2)
{
  if (p1 != NULL && p2 != NULL)
  {
    core::tFrameworkElement* parent1 = p1->GetParent();
    core::tFrameworkElement* parent2 = p2->GetParent();
    if ((boost::ends_with(parent1->GetName(), "Output") && boost::ends_with(parent2->GetName(), "Input")) ||
        (boost::ends_with(parent1->GetName(), "Input") && boost::ends_with(parent2->GetName(), "Output")))
    {
      if (!((boost::starts_with(parent1->GetName(), "Sensor") && boost::starts_with(parent2->GetName(), "Controller")) ||
            (boost::starts_with(parent1->GetName(), "Controller") && boost::starts_with(parent2->GetName(), "Sensor"))))
      {
        p1->ConnectTo(*p2);
      }
    }
  }
}

template<typename T>
void tBlackboardClient<T>::CheckClientConnect(core::tAbstractPort* p1, core::tAbstractPort* p2)
{
  if (p1 != NULL && p2 != NULL)
  {
    core::tFrameworkElement* parent1 = p1->GetParent();
    core::tFrameworkElement* parent2 = p2->GetParent();
    if ((boost::ends_with(parent1->GetName(), "Input") && boost::ends_with(parent2->GetName(), "Input")) ||
        (boost::ends_with(parent1->GetName(), "Output") && boost::ends_with(parent2->GetName(), "Output")))
    {
      if (!((boost::starts_with(parent1->GetName(), "Sensor") && boost::starts_with(parent2->GetName(), "Controller")) ||
            (boost::starts_with(parent1->GetName(), "Controller") && boost::starts_with(parent2->GetName(), "Sensor"))))
      {
        p1->ConnectTo(*p2);
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
    tAbstractBlackboardServer<T>::cASYNCH_CHANGE.Call(*wrapped->GetWritePort(), true, static_cast<tConstChangeTransactionVar&>(change_buf), index, offset);

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
void tBlackboardClient<T>::ConnectTo(const tBlackboardClient<T>& client)
{
  if (client.GetOutsideReadPort() && GetOutsideReadPort())
  {
    CheckClientConnect(client.GetOutsideReadPort()->GetWrapped(), GetOutsideReadPort()->GetWrapped());
  }
  CheckClientConnect(client.GetOutsideWritePort(), GetOutsideWritePort());
  CheckClientConnect(client.GetOutsideWritePort2(), GetOutsideWritePort());
  CheckClientConnect(client.GetOutsideWritePort(), GetOutsideWritePort2());
  CheckClientConnect(client.GetOutsideWritePort2(), GetOutsideWritePort2());
}

template<typename T>
bool tBlackboardClient<T>::HasChanged() const
{
  assert((wrapped->GetReadPort() != NULL));
  if (!wrapped->GetReadPort()->GetFlag(core::tPortFlags::cPUSH_STRATEGY))
  {
    FINROC_LOG_PRINT(DEBUG_WARNING, "This method only works properly, when push strategy is used.");
  }
  return wrapped->GetReadPort()->HasChanged();
}

template<typename T>
rrlib::rtti::tDataTypeBase tBlackboardClient<T>::InitBlackboardType(rrlib::rtti::tDataTypeBase dt)
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
    tAbstractBlackboardServer<T>::cDIRECT_COMMIT.Call(*wrapped->GetWritePort(), true, buffer);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_PRINT(WARNING, "warning: Blackboard direct commit failed");
  }
}

template<typename T>
const typename tAbstractBlackboardServer<T>::tBBVector* tBlackboardClient<T>::ReadLock(bool force_read_copy_to_avoid_blocking, const rrlib::time::tDuration& timeout)
{
  assert(((locked == NULL && wrapped->lock_type == tRawBlackboardClient::eNONE)) && "Unlock first");

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
    wrapped->cur_lock_id = -1;
    read_locked = Read(timeout);

    return read_locked.get();
  }
  else
  {
    assert((locked == NULL && wrapped->lock_type == tRawBlackboardClient::eNONE));
    assert((wrapped->cur_lock_id == -1));
    assert((wrapped->IsReady()));
    try
    {
      tConstBBVectorVar ret = tAbstractBlackboardServer<T>::cREAD_LOCK.Call(*wrapped->GetWritePort(), timeout + tRawBlackboardClient::cNET_TIMEOUT, (int)std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count(), 0);

      if (ret != NULL)
      {
        wrapped->lock_type = tRawBlackboardClient::eREAD;

        wrapped->cur_lock_id = ret.GetManager()->lock_id;
        read_locked = std::move(ret);

        // acknowledge lock
        wrapped->SendKeepAlive();

        return read_locked.get();
      }
      else
      {
        wrapped->cur_lock_id = -1;
        return NULL;
      }
    }
    catch (const core::tMethodCallException& e)
    {
      wrapped->cur_lock_id = -1;
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
    FINROC_LOG_PRINT(DEBUG_WARNING, "This method only works properly, when push strategy is used.");
  }
  wrapped->GetReadPort()->ResetChanged();
}

template<typename T>
void tBlackboardClient<T>::Unlock()
{
  if (wrapped->lock_type == tRawBlackboardClient::eNONE)
  {
    FINROC_LOG_PRINT(WARNING, "BlackboardClient warning: nothing to unlock");
    ResetVariables();
    return;  // nothing to unlock
  }
  if (wrapped->lock_type == tRawBlackboardClient::eREAD)
  {
    // we only have a read copy
    assert((read_locked != NULL));
    if (wrapped->cur_lock_id >= 0)
    {
      try
      {
        tAbstractBlackboardServer<T>::cREAD_UNLOCK.Call(*wrapped->GetWritePort(), true, wrapped->cur_lock_id);
      }
      catch (const core::tMethodCallException& e)
      {
        FINROC_LOG_PRINT(WARNING, "warning: Unlocking blackboard (read) failed", e);
      }
    }

    ResetVariables();
    return;
  }

  assert((wrapped->lock_type == tRawBlackboardClient::eWRITE));
  assert((wrapped->cur_lock_id >= 0));

  try
  {
    tAbstractBlackboardServer<T>::cUNLOCK.Call(*wrapped->GetWritePort(), true, locked);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_PRINT(WARNING, "warning: Unlocking blackboard failed");
    //e.printStackTrace();
  }
  ResetVariables();
}

template<typename T>
typename tAbstractBlackboardServer<T>::tBBVector* tBlackboardClient<T>::WriteLock(const rrlib::time::tDuration& timeout)
{
  assert((locked == NULL && wrapped->lock_type == tRawBlackboardClient::eNONE));
  assert((wrapped->cur_lock_id == -1));
  assert((wrapped->IsReady()));
  try
  {
    tBBVectorVar ret = tAbstractBlackboardServer<T>::cLOCK.Call(*wrapped->GetWritePort(), timeout + tRawBlackboardClient::cNET_TIMEOUT, (int)std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());

    if (ret != NULL)
    {
      wrapped->lock_type = tRawBlackboardClient::eWRITE;

      wrapped->cur_lock_id = ret.GetManager()->lock_id;
      locked = std::move(ret);

      // acknowledge lock
      wrapped->SendKeepAlive();
    }
    else
    {
      wrapped->cur_lock_id = -1;
    }

    return locked.Get();

  }
  catch (const core::tMethodCallException& e)
  {
    wrapped->cur_lock_id = -1;
    return NULL;
  }
}

} // namespace finroc
} // namespace blackboard

