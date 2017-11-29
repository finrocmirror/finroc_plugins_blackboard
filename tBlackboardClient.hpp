//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
#include "rrlib/util/string.h"

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


/*
read_port(create_read_port_in ? tReadPort() : tReadPort()),
    write_port(),
    change_set_buffer_pool(new tChangeSetBufferPool(rrlib::rtti::tDataType<tChangeSet>(), 0))
*/

template<typename T>
tBlackboardClient<T>::tBlackboardClient(bool push_updates, tInterface& create_write_port_in, const std::string& write_port_name, tInterface* create_read_port_in, const std::string& read_port_name) :
  read_port(),
  write_port(write_port_name, &create_write_port_in, internal::tBlackboardServer<T>::GetRPCInterfaceType()),
  change_set_buffer_pool(new tChangeSetBufferPool(rrlib::rtti::tDataType<tChangeSet>(), 0))
{
  if (create_read_port_in)
  {
    read_port = tReadPort(read_port_name, create_read_port_in, data_ports::cDEFAULT_INPUT_PORT_FLAGS);
    if (!push_updates)
    {
      read_port.SetPushStrategy(false);
    }
  }
}


template<typename T>
tBlackboardClient<T>::tBlackboardClient(tBlackboardClient && o) :
  read_port(),
  write_port(),
  change_set_buffer_pool()
{
  std::swap(read_port, o.read_port);
  std::swap(write_port, o.write_port);
  std::swap(change_set_buffer_pool, o.change_set_buffer_pool);
}

template<typename T>
tBlackboardClient<T>& tBlackboardClient<T>::operator=(tBlackboardClient && o)
{
  std::swap(read_port, o.read_port);
  std::swap(write_port, o.write_port);
  std::swap(change_set_buffer_pool, o.change_set_buffer_pool);
  return *this;
}


template<typename T>
void tBlackboardClient<T>::ConnectTo(const tBlackboard<T>& blackboard)
{
  if (blackboard.GetReadPort().GetWrapped() && GetReadPort().GetWrapped())
  {
    blackboard.GetReadPort().ConnectTo(GetReadPort());
  }
  GetWritePort().ConnectTo(blackboard.GetWritePort());
}

template<typename T>
void tBlackboardClient<T>::ConnectTo(const tBlackboardClient<T>& client)
{
  if (client.GetReadPort().GetWrapped() && GetReadPort().GetWrapped())
  {
    client.GetReadPort().ConnectTo(GetReadPort());
  }
  GetWritePort().ConnectTo(client.GetWritePort());
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
rpc_ports::tFuture<typename tBlackboardClient<T>::tReadLockedBufferPointer> tBlackboardClient<T>::ReadLock(const rrlib::time::tDuration& timeout)
{
  // Possibly obtain value from read_port
  if (read_port.GetWrapped() && read_port.GetFlag(core::tFrameworkElement::tFlag::PUSH_STRATEGY))
  {
    auto buffer_pointer = read_port.GetPointer();
    if (buffer_pointer->size())   // TODO: Heuristic: if blackboard does not have read port, blackboard has size zero; so if size is zero, lock via RPC lock which is always appropriate  (this heuristic can break, when read port was connected to multi-buffered blackboard - and is connected to single-buffered blackboard later); why is read port used at all? over the network this is significantly more efficient (push instead of pull)
    {
      rpc_ports::tPromise<tReadLockedBufferPointer> promise;
      promise.SetValue(std::move(buffer_pointer));
      return promise.GetFuture();
    }
  }

  return write_port.NativeFutureCall(&tServer::ReadLock, timeout);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
