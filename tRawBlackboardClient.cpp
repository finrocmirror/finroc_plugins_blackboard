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
#include "core/portdatabase/tDataType.h"
#include "plugins/blackboard/tRawBlackboardClient.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "core/port/tPortFlags.h"
#include "core/port/rpc/tInterfacePort.h"
#include "core/port/rpc/method/tPort0Method.h"
#include "core/port/rpc/tMethodCallException.h"
#include "core/port/std/tPortDataManager.h"
#include "core/port/rpc/method/tVoid2Method.h"
#include "plugins/blackboard/tBlackboardManager.h"
#include "core/port/rpc/method/tVoid1Method.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "core/port/rpc/method/tPort2Method.h"
#include "core/port/rpc/method/tPort3Method.h"
#include "core/port/rpc/method/tPort1Method.h"
#include "core/port/tAbstractPort.h"

namespace finroc
{
namespace blackboard
{
tRawBlackboardClient::tRawBlackboardClient(core::tPortCreationInfo pci, bool auto_connect_, int auto_connect_category_) :
    core::tFrameworkElement(pci.parent, pci.description),
    server_buffers(tRawBlackboardClient::eUNKNOWN),
    write_port(pci.GetFlag(core::tPortFlags::cEMITS_DATA) ? new tWritePort(this, pci.data_type->GetRelatedType()) : NULL),
    read_port(pci.GetFlag(core::tPortFlags::cACCEPTS_DATA) ? new tReadPort(this, core::tPortCreationInfo("read", this, pci.data_type, core::tPortFlags::cACCEPTS_DATA | (pci.flags & core::tPortFlags::cPUSH_STRATEGY))) : NULL),
    locked(NULL),
    read_locked(NULL),
    lock_type(tRawBlackboardClient::eNONE),
    cur_lock_iD(-1),
    auto_connect(auto_connect_),
    auto_connect_category(auto_connect_category_),
    cNET_TIMEOUT(1000)
{
  tAbstractBlackboardServer::CheckType(pci.data_type);
}

bool tRawBlackboardClient::CheckConnect(tAbstractBlackboardServer* server)
{
  assert((auto_connect));
  if (server->read_port == NULL || server->write_port == NULL)
  {
    return false;
  }
  if (IsConnected())
  {
    return true;  // already connected
  }
  if (auto_connect_category >= 0 && auto_connect_category != server->category_index)
  {
    return false;  // wrong category
  }
  if (read_port != NULL && server->read_port->GetDataType() != read_port->GetDataType())
  {
    return false;  // data types don't fit
  }
  if (write_port != NULL && server->write_port->GetDataType() != write_port->GetDataType())
  {
    return false;  // data types don't fit
  }
  if (!GetDescription().Equals(server->GetDescription()))
  {
    return false;  // descriptions don't match
  }

  // checks passed => connect
  if (read_port != NULL)
  {
    read_port->ConnectToSource(server->read_port);
  }
  if (write_port != NULL)
  {
    write_port->ConnectToSource(server->write_port);
  }
  return true;
}

void tRawBlackboardClient::CheckSingleBuffered()
{
  if (server_buffers != tRawBlackboardClient::eUNKNOWN)
  {
    return;
  }
  try
  {
    int8 result = tAbstractBlackboardServer::cIS_SINGLE_BUFFERED.Call(*write_port, cNET_TIMEOUT);
    server_buffers = (result == 0) ? tRawBlackboardClient::eMULTI : tRawBlackboardClient::eSINGLE;
  }
  catch (const core::tMethodCallException& e)
  {
    server_buffers = tRawBlackboardClient::eUNKNOWN;
  }
}

void tRawBlackboardClient::CommitAsynchChange(int offset, const tBlackboardBuffer* change_buf)
{
  if (change_buf->GetManager()->IsUnused())
  {
    change_buf->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
  }
  tAbstractBlackboardServer::cASYNCH_CHANGE.Call(*write_port, offset, change_buf, true);
}

void tRawBlackboardClient::PostChildInit()
{
  ::finroc::core::tFrameworkElement::PostChildInit();
  assert(((tBlackboardManager::GetInstance() != NULL)) && "truly strange");
  tBlackboardManager::GetInstance()->AddClient(this, auto_connect);
}

void tRawBlackboardClient::PrepareDelete()
{
  tBlackboardManager* instance = tBlackboardManager::GetInstance();
  if (instance != NULL)    // we don't need to remove it, if blackboard manager has already been deleted
  {
    instance->RemoveClient(this);
  }
  ::finroc::core::tFrameworkElement::PrepareDelete();
}

void tRawBlackboardClient::Publish(tBlackboardBuffer* buffer)
{
  assert((lock_type == tRawBlackboardClient::eNONE));
  if (buffer->GetManager()->IsUnused())
  {
    buffer->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
  }
  try
  {
    tAbstractBlackboardServer::cDIRECT_COMMIT.Call(*write_port, buffer, true);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Blackboard direct commit failed");
  }
}

const tBlackboardBuffer* tRawBlackboardClient::ReadLock(bool force_read_copy_to_avoid_blocking, int64 timeout)
{
  assert(((locked == NULL && lock_type == tRawBlackboardClient::eNONE)) && "Unlock first");
  if (timeout <= 0)
  {
    timeout = 60000;  // wait one minute for method to complete if no time is specified
  }

  // determine whether blackboard server is single buffered
  CheckSingleBuffered();
  if (server_buffers == tRawBlackboardClient::eUNKNOWN)    // we currently have no partner (?)
  {
    return NULL;
  }

  bool via_port = (server_buffers == tRawBlackboardClient::eMULTI) || read_port->PushStrategy() || force_read_copy_to_avoid_blocking || write_port->HasRemoteServer();
  if (via_port)
  {
    lock_type = tRawBlackboardClient::eREAD;
    cur_lock_iD = -1;
    return (read_locked = Read(timeout));
  }
  else
  {
    assert((locked == NULL && lock_type == tRawBlackboardClient::eNONE));
    assert((cur_lock_iD == -1));
    assert((IsReady()));
    try
    {
      const tBlackboardBuffer* ret = tAbstractBlackboardServer::cREAD_LOCK.Call(*write_port, timeout, 0, static_cast<int>((timeout + cNET_TIMEOUT)));
      if (ret != NULL)
      {
        this->lock_type = tRawBlackboardClient::eREAD;
        cur_lock_iD = ret->lock_iD;
        read_locked = ret;

        // acknowledge lock
        SendKeepAlive();
      }
      else
      {
        cur_lock_iD = -1;
      }
      return ret;
    }
    catch (const core::tMethodCallException& e)
    {
      cur_lock_iD = -1;
      return NULL;
    }
  }
}

tBlackboardBuffer* tRawBlackboardClient::ReadPart(int offset, int length, int timeout)
{
  if (timeout <= 0)
  {
    timeout = 60000;  // wait one minute for method to complete if no time is specified
  }
  try
  {
    return tAbstractBlackboardServer::cREAD_PART.Call(*write_port, offset, length, timeout, timeout + cNET_TIMEOUT);
  }
  catch (const core::tMethodCallException& e)
  {
    return NULL;
  }
}

void tRawBlackboardClient::SendKeepAlive()
{
  int cur_lock_iD = this->cur_lock_iD;
  if (cur_lock_iD >= 0)
  {
    try
    {
      tAbstractBlackboardServer::cKEEP_ALIVE.Call(*write_port, cur_lock_iD, false);
    }
    catch (const core::tMethodCallException& e)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Sending Keep-Alive failed");
    }
  }
}

void tRawBlackboardClient::Unlock()
{
  if (lock_type == tRawBlackboardClient::eNONE)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "BlackboardClient warning: nothing to unlock");
    cur_lock_iD = -1;
    lock_type = tRawBlackboardClient::eNONE;
    locked = NULL;
    read_locked = NULL;
    return;  // nothing to unlock
  }
  if (lock_type == tRawBlackboardClient::eREAD)
  {
    // we only have a read copy
    assert((read_locked != NULL));
    if (cur_lock_iD >= 0)
    {
      try
      {
        tAbstractBlackboardServer::cREAD_UNLOCK.Call(*write_port, cur_lock_iD, true);
      }
      catch (const core::tMethodCallException& e)
      {
        FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Unlocking blackboard (read) failed", e);
      }
    }
    read_locked->GetManager()->ReleaseLock();
    locked = NULL;
    read_locked = NULL;
    cur_lock_iD = -1;
    lock_type = tRawBlackboardClient::eNONE;
    return;
  }

  assert((lock_type == tRawBlackboardClient::eWRITE));
  assert((cur_lock_iD >= 0));

  try
  {
    tAbstractBlackboardServer::cUNLOCK.Call(*write_port, locked, true);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Unlocking blackboard failed");
    //e.printStackTrace();
  }
  locked = NULL;
  read_locked = NULL;
  cur_lock_iD = -1;
  lock_type = tRawBlackboardClient::eNONE;
}

tBlackboardBuffer* tRawBlackboardClient::WriteLock(int64 timeout)
{
  if (timeout <= 0)
  {
    timeout = 60000;  // wait one minute for method to complete if no time is specified
  }

  assert((locked == NULL && lock_type == tRawBlackboardClient::eNONE));
  assert((cur_lock_iD == -1));
  assert((IsReady()));
  try
  {
    tBlackboardBuffer* ret = tAbstractBlackboardServer::cLOCK.Call(*write_port, timeout, static_cast<int>((timeout + cNET_TIMEOUT)));
    if (ret != NULL)
    {
      this->lock_type = tRawBlackboardClient::eWRITE;
      cur_lock_iD = ret->lock_iD;
      locked = ret;

      // acknowledge lock
      SendKeepAlive();
    }
    else
    {
      cur_lock_iD = -1;
    }
    return ret;
  }
  catch (const core::tMethodCallException& e)
  {
    cur_lock_iD = -1;
    return NULL;
  }
}

tRawBlackboardClient::tReadPort::tReadPort(tRawBlackboardClient* const outer_class_ptr_, core::tPortCreationInfo pci) :
    core::tPortBase(pci),
    outer_class_ptr(outer_class_ptr_)
{
}

void tRawBlackboardClient::tReadPort::NewConnection(core::tAbstractPort* partner)
{
  ::finroc::core::tAbstractPort::NewConnection(partner);
  if (outer_class_ptr->write_port != NULL)
  {
    ::finroc::core::tFrameworkElement* w = partner->GetParent()->GetChild("write");
    if (w != NULL)
    {
      outer_class_ptr->write_port->ConnectToSource(static_cast< ::finroc::core::tAbstractPort*>(w));
    }
  }
  outer_class_ptr->server_buffers = tRawBlackboardClient::eUNKNOWN;
}

tRawBlackboardClient::tWritePort::tWritePort(tRawBlackboardClient* const outer_class_ptr_, core::tDataType* type) :
    core::tInterfaceClientPort("write", outer_class_ptr_, type),
    outer_class_ptr(outer_class_ptr_)
{
}

void tRawBlackboardClient::tWritePort::NewConnection(core::tAbstractPort* partner)
{
  ::finroc::core::tInterfaceClientPort::NewConnection(partner);
  if (outer_class_ptr->read_port != NULL)
  {
    core::tFrameworkElement* w = partner->GetParent()->GetChild("read");
    if (w != NULL)
    {
      outer_class_ptr->read_port->ConnectToSource(static_cast<core::tAbstractPort*>(w));
    }
  }
  outer_class_ptr->server_buffers = tRawBlackboardClient::eUNKNOWN;
}

} // namespace finroc
} // namespace blackboard

