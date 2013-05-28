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
/*!\file    plugins/blackboard/internal/tBlackboardServer.hpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-19
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tServerPort.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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

template <typename T>
tBlackboardServer<T>::tBlackboardServer(const std::string& name, core::tFrameworkElement* parent, bool multi_buffered, size_t elements, bool shared) :
  tAbstractBlackboardServer(parent, name, shared ? core::tFrameworkElement::tFlags(core::tFrameworkElement::tFlag::SHARED) : core::tFrameworkElement::tFlags()),
  read_port("read", this, core::tFrameworkElement::tFlag::FINSTRUCT_READ_ONLY),
  write_port(rpc_ports::tServerPort<tBlackboardServer<T>>(*this, "write", this, GetRPCInterfaceType())),
  pending_change_tasks(),
  pending_lock_requests(),
  current_buffer(read_port.GetWrapped()->GetCurrentValueRaw()),
  lock_id(0),
  write_lock(tWriteLock::NONE),
  unlock_future(),
  single_buffered(!multi_buffered)
{
  read_port.Init();
  write_port.Init();
  if (elements > 0)
  {
    NewCurrentBuffer(true);
    rrlib::rtti::ResizeVector(current_buffer->GetObject().GetData<tBuffer>(), elements);
    read_port.GetWrapped()->Publish(current_buffer);
    current_buffer = read_port.GetWrapped()->GetCurrentValueRaw();
  }
  assert(!current_buffer->IsUnused());
}

template <typename T>
void tBlackboardServer<T>::AsynchronousChange(tChangeSetPointer change_set)
{
  if (change_set)
  {
    rrlib::thread::tLock lock(this->BlackboardMutex());
    if (write_lock != tWriteLock::NONE)
    {
      this->DeferAsynchronousChange(std::move(change_set));
    }
    else
    {
      // Update internal buffer?
      if (!current_buffer->Unique())
      {
        NewCurrentBuffer(true);
      }

      // Apply change
      this->ApplyAsynchronousChange(current_buffer->GetObject().GetData<tBuffer>(), *change_set);

      // Publish current buffer
      ConsiderPublishing();
    }
  }
}

template <typename T>
void tBlackboardServer<T>::DirectCommit(tBufferPointer new_buffer)
{
  if (new_buffer)
  {
    rrlib::thread::tLock lock(this->BlackboardMutex());

    // Clear any asynch change commands from queue, since they were for old buffer
    this->ClearPendingChangeTasks();

    // Update internal variables
    lock_id++; // any current lock is obsolete, since we have completely new buffer
    write_lock = tWriteLock::NONE;
    unlock_future = tUnlockFuture(); // remove handler
    if (!current_buffer->Unique())
    {
      NewCurrentBuffer(false);
    }
    std::swap(*new_buffer, current_buffer->GetObject().GetData<tBuffer>());

    // Publish current buffer
    ConsiderPublishing();

    // Any pending lock requests?
    this->ProcessPendingLockRequests();
  }
}

template <typename T>
rrlib::rtti::tType tBlackboardServer<T>::GetRPCInterfaceType()
{
  static rpc_ports::tRPCInterfaceType<tBlackboardServer<T>> type("Blackboard<" + rrlib::rtti::tDataType<T>().GetName() + ">",
      &tBlackboardServer<T>::AsynchronousChange, &tBlackboardServer<T>::DirectCommit, &tBlackboardServer<T>::ReadLock, &tBlackboardServer<T>::WriteLock);
  return type;
}

template <typename T>
void tBlackboardServer<T>::HandleException(rpc_ports::tFutureStatus exception_type)
{
  rrlib::thread::tLock lock(this->BlackboardMutex());
  if (exception_type != rpc_ports::tFutureStatus::READY)
  {
    FINROC_LOG_PRINT(DEBUG, "Blackboard unlock due to exception: ", make_builder::GetEnumString(exception_type));
  }

  lock_id++;
  write_lock = tWriteLock::NONE;
  unlock_future = tUnlockFuture(); // remove handler

  // Apply any pending changes
  if (pending_change_tasks.size() > 0)
  {
    // Update internal buffer?
    if (!current_buffer->Unique())
    {
      NewCurrentBuffer(true);
    }

    // Apply changes
    this->ApplyPendingChangeTasks(current_buffer->GetObject().GetData<tBuffer>());

    // publish buffer
    ConsiderPublishing();
  }

  // Any pending lock requests?
  this->ProcessPendingLockRequests();
}

template <typename T>
void tBlackboardServer<T>::HandleResponse(tLockedBufferData<tBuffer> unlock_data)
{
  if (!unlock_data.buffer)
  {
    HandleException(rpc_ports::tFutureStatus::READY);
    //FINROC_LOG_PRINT(WARNING, "Blackboard unlock without providing buffer");
    return;
  }

  rrlib::thread::tLock lock(this->BlackboardMutex());
  if (unlock_data.lock_id != lock_id)
  {
    FINROC_LOG_PRINT(DEBUG, "Skipping outdated unlock");
    return;
  }

  // Apply any pending changes
  this->ApplyPendingChangeTasks(*unlock_data.buffer);

  // Update internal variables
  lock_id++;
  write_lock = tWriteLock::NONE;
  unlock_future = tUnlockFuture(); // remove handler
  if (unlock_data.buffer.get() != &current_buffer->GetObject().GetData<tBuffer>())
  {
    if (!current_buffer->Unique())
    {
      NewCurrentBuffer(false);
    }
    std::swap(*unlock_data.buffer, current_buffer->GetObject().GetData<tBuffer>());
  }

  // publish buffer
  ConsiderPublishing();

  // Any pending lock requests?
  this->ProcessPendingLockRequests();
}

template <typename T>
void tBlackboardServer<T>::ProcessPendingLockRequests()
{
  while (pending_lock_requests.size() > 0)
  {
    tLockRequest& lock_request = pending_lock_requests.front();
    if (rrlib::time::Now() <= lock_request.timeout_time)
    {
      if (lock_request.write_lock)
      {
        WriteLockImplementation(lock_request.write_lock_promise, lock_request.remote_call);
        pending_lock_requests.pop_front();
        return;
      }
      else
      {
        current_buffer->AddLocks(1);
        tConstBufferPointer pointer_clone(data_ports::standard::tStandardPort::tLockingManagerPointer(current_buffer.get()), *read_port.GetWrapped());
        lock_request.read_lock_promise.SetValue(pointer_clone);
      }
    }
    pending_lock_requests.pop_front();
  }
}

template <typename T>
rpc_ports::tFuture<typename tBlackboardServer<T>::tConstBufferPointer> tBlackboardServer<T>::ReadLock(const rrlib::time::tDuration& timeout)
{
  rrlib::thread::tLock lock(this->BlackboardMutex());
  rpc_ports::tPromise<tConstBufferPointer> promise;
  rpc_ports::tFuture<tConstBufferPointer> future = promise.GetFuture();
  if (write_lock != tWriteLock::EXCLUSIVE)
  {
    assert(!current_buffer->IsUnused());
    current_buffer->AddLocks(1);
    tConstBufferPointer pointer_clone(data_ports::standard::tStandardPort::tLockingManagerPointer(current_buffer.get()), *read_port.GetWrapped());
    promise.SetValue(pointer_clone);
  }
  else
  {
    FINROC_LOG_PRINT(DEBUG, "Attempt to read-lock during exclusive write lock. Enabling multi-buffered mode to avoid blocking in such situations in the future.");
    single_buffered = false;
    if (timeout > rrlib::time::tDuration::zero())
    {
      this->pending_lock_requests.emplace_back(std::move(promise), rrlib::time::Now() + timeout);
    }
  }
  return future;
}

template <typename T>
rpc_ports::tFuture<tLockedBuffer<typename tBlackboardServer<T>::tBuffer>> tBlackboardServer<T>::WriteLock(tLockParameters lock_parameters)
{
  rrlib::thread::tLock lock(this->BlackboardMutex());
  rpc_ports::tPromise<tLockedBuffer<tBuffer>> promise;
  rpc_ports::tFuture<tLockedBuffer<tBuffer>> future = promise.GetFuture();
  if (write_lock == tWriteLock::NONE)
  {
    WriteLockImplementation(promise, lock_parameters.IsRemoteCall());
  }
  else
  {
    if (lock_parameters.GetTimeout() > rrlib::time::tDuration::zero())
    {
      this->pending_lock_requests.emplace_back(std::move(promise), rrlib::time::Now() + lock_parameters.GetTimeout(), lock_parameters.IsRemoteCall());
    }
  }
  return future;
}

template <typename T>
void tBlackboardServer<T>::WriteLockImplementation(rpc_ports::tPromise<tLockedBuffer<tBuffer>>& promise, bool remote_call)
{
  assert(!current_buffer->IsUnused());
  if ((!remote_call) && current_buffer->Unique())
  {
    write_lock = tWriteLock::EXCLUSIVE;
    lock_id++;
    current_buffer->AddLocks(1);
    tBufferPointer pointer_clone(data_ports::standard::tStandardPort::tLockingManagerPointer(current_buffer.get()), *read_port.GetWrapped());
    tLockedBuffer<tBuffer> locked_buffer(std::move(pointer_clone), lock_id);
    locked_buffer.SetBufferSource(read_port);
    unlock_future = locked_buffer.GetFuture();
    unlock_future.SetCallback(*this);
    promise.SetValue(locked_buffer);
  }
  else
  {
    write_lock = tWriteLock::ON_COPY;
    lock_id++;
    current_buffer->AddLocks(1);
    tConstBufferPointer pointer_clone(data_ports::standard::tStandardPort::tLockingManagerPointer(current_buffer.get()), *read_port.GetWrapped());
    tLockedBuffer<tBuffer> locked_buffer(std::move(pointer_clone), lock_id);
    locked_buffer.SetBufferSource(read_port);
    unlock_future = locked_buffer.GetFuture();
    unlock_future.SetCallback(*this);
    promise.SetValue(locked_buffer);
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
