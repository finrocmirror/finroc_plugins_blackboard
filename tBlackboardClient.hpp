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
#include "core/port/rpc/tMethodCallException.h"
#include "core/port/rpc/method/tPort2Method.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "core/port/rpc/method/tPort1Method.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
tBlackboardClient<T>::tBlackboardClient(core::tPortCreationInfo pci, bool auto_connect, int auto_connect_category) :
    wrapped(new tRawBlackboardClient(pci, static_cast<T*>(NULL), auto_connect, auto_connect_category)),
    locked(),
    read_locked()
{
}

template<typename T>
void tBlackboardClient<T>::Publish(tBBVectorVar& buffer)
{
  assert((wrapped->lock_type == tRawBlackboardClient::eNONE));

  assert(!core::tPortDataManager::GetManager(buffer)->IsUnused() && "Obtain buffer from getUnusedBuffer()");

  /*if (buffer.getManager().isUnused()) {
      buffer.getManager().getCurrentRefCounter().setLocks((byte)1);
  }*/
  try
  {
    tAbstractBlackboardServer<T>::cDIRECT_COMMIT.Call(*wrapped->GetWritePort(), buffer, true);
  }
  catch (const core::tMethodCallException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Blackboard direct commit failed");
  }
}

template<typename T>
typename tAbstractBlackboardServer<T>::tConstBBVectorVar tBlackboardClient<T>::ReadLock(bool force_read_copy_to_avoid_blocking, int timeout)
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

    return tConstBBVectorVar();
  }

  bool via_port = (wrapped->server_buffers == tRawBlackboardClient::eMULTI) || wrapped->GetReadPort()->PushStrategy() || force_read_copy_to_avoid_blocking || wrapped->GetWritePort()->HasRemoteServer();
  if (via_port)
  {
    wrapped->lock_type = tRawBlackboardClient::eREAD;
    wrapped->cur_lock_iD = -1;
    return (read_locked = Read(timeout));
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

        wrapped->cur_lock_iD = core::tPortDataManager::GetManager(ret)->lock_iD;
        read_locked = ret;

        // acknowledge lock
        wrapped->SendKeepAlive();
      }
      else
      {
        wrapped->cur_lock_iD = -1;
      }
      return ret;
    }
    catch (const core::tMethodCallException& e)
    {
      wrapped->cur_lock_iD = -1;

      return tConstBBVectorVar();
    }
  }
}

template<typename T>
void tBlackboardClient<T>::Unlock()
{
  if (wrapped->lock_type == tRawBlackboardClient::eNONE)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "BlackboardClient warning: nothing to unlock");
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
        FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Unlocking blackboard (read) failed", e);
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
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Unlocking blackboard failed");
    //e.printStackTrace();
  }
  ResetVariables();
}

template<typename T>
typename tAbstractBlackboardServer<T>::tBBVectorVar tBlackboardClient<T>::WriteLock(int timeout)
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

      wrapped->cur_lock_iD = core::tPortDataManager::GetManager(ret)->lock_iD;

      locked = ret;

      // acknowledge lock
      wrapped->SendKeepAlive();
    }
    else
    {
      wrapped->cur_lock_iD = -1;
    }
    return ret;
  }
  catch (const core::tMethodCallException& e)
  {
    wrapped->cur_lock_iD = -1;

    return tBBVectorVar();
  }
}

} // namespace finroc
} // namespace blackboard

