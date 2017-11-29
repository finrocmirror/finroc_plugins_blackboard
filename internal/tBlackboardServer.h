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
/*!\file    plugins/blackboard/internal/tBlackboardServer.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-19
 *
 * \brief   Contains tBlackboardServer
 *
 * \b tBlackboardServer
 *
 * Blackboard server implementation.
 * Can run in single-buffered and multi-buffered mode.
 * The latter is currently activated automatically, if concurrent
 * read lock operations are detected.
 *
 * In multi-buffered mode, the blackboard server never blocks on read locks.
 * The buffer is copied on each write, which causes computational overhead, though.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tBlackboardServer_h__
#define __plugins__blackboard__internal__tBlackboardServer_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tPromise.h"
#include "plugins/rpc_ports/tRPCInterface.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tChange.h"
#include "plugins/blackboard/internal/tAbstractBlackboardServer.h"
#include "plugins/blackboard/internal/tLockedBuffer.h"
#include "plugins/blackboard/internal/tLockParameters.h"

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

/*! Type of write lock */
enum class tWriteLock
{
  NONE,     //!< Blackboard has no write lock
  ON_COPY,  //!< Write-locked; buffer changes are performed on buffer copy
  EXCLUSIVE  //!< Write-locked; buffer changes are performed on the one and only blackboard buffer (=> concurrent read-access not possible/safe)
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Blackboard Server
/*!
 * Blackboard server implementation.
 * Can run in single-buffered and multi-buffered mode.
 * The latter is currently activated automatically, if concurrent
 * read lock operations are detected.
 *
 * In multi-buffered mode, the blackboard server never blocks on read locks.
 * The buffer is copied on each write, which causes computational overhead, though.
 *
 * \tparam T Type of blackboard elements
 */
template <typename T>
class tBlackboardServer : public tAbstractBlackboardServer, public rpc_ports::tResponseHandler<tLockedBufferData<std::vector<T>>>, public rpc_ports::tRPCInterface
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef std::vector<T> tBuffer;
  typedef data_ports::tPortDataPointer<tBuffer> tBufferPointer;
  typedef data_ports::tPortDataPointer<const tBuffer> tConstBufferPointer;
  typedef tChange<T> tSingleChange;
  typedef std::vector<tSingleChange> tChangeSet;
  typedef data_ports::tPortDataPointer<tChangeSet> tChangeSetPointer;
  typedef data_ports::tOutputPort<tBuffer> tReadPort;

  class tReadLockedBufferPointer : public tConstBufferPointer
  {
    friend class tBlackboardServer;
    typedef tConstBufferPointer tBase;

    tBlackboardServer* process_pending_on_unlock;

  public:
    template <typename ... TArgs>
    tReadLockedBufferPointer(TArgs && ... args) :
      tBase(std::forward<TArgs>(args)...),
      process_pending_on_unlock(nullptr)
    {
    }

    tReadLockedBufferPointer(tReadLockedBufferPointer && other) :
      tBase(std::forward<tBase>(other)),
      process_pending_on_unlock(nullptr)
    {
      std::swap(process_pending_on_unlock, other.process_pending_on_unlock);
    }

    tReadLockedBufferPointer& operator=(tReadLockedBufferPointer && other)
    {
      tBase::operator=(std::forward<tBase>(other));
      std::swap(process_pending_on_unlock, other.process_pending_on_unlock);
      return *this;
    }

    ~tReadLockedBufferPointer()
    {
      if (process_pending_on_unlock)
      {
        process_pending_on_unlock->HandleReadUnlock(*this);
      }
    }
  };


  /*!
   * \param name Name/Uid of blackboard
   * \param parent Parent of blackboard server (usually component-internal blackboard element)
   * \param buffer_mode Buffer mode - whether to use multiple buffers to avoid blocking (at the cost of copying content)
   * \param elements Initial number of elements
   * \param create_write_port_in Interface to create write port in
   * \param create_read_port_in If not nullptr, creates data port for reading blackboard in specified component interface (possibly relevant for data dependencies -> scheduling order)
   * \param read_port_name Name for read port. If empty, blackboard name will be used.
   */
  tBlackboardServer(const std::string& name, core::tFrameworkElement& parent, tBlackboardBufferMode buffer_mode, size_t elements, tInterface& create_write_port_in, tInterface* create_read_port_in = nullptr, const std::string& read_port_name = "");

  virtual ~tBlackboardServer() {}

  /*!
   * \return RPC interface type of tBlackboardServer<T>
   */
  static rrlib::rtti::tType GetRPCInterfaceType();

  /*!
   * (RPC Call)
   * Apply change transaction to blackboard asynchronously
   *
   * \param change_set Change set to apply
   */
  void AsynchronousChange(tChangeSetPointer change_set);

  /*!
   * (RPC Call)
   * Direct commit of new blackboard buffer
   * (existing buffer is replaced with new one
   *
   * \param new_buffer New Buffer
   */
  void DirectCommit(tBufferPointer new_buffer);

  /*!
   * \return Blackboard's current buffer mode
   */
  tBlackboardBufferMode GetBufferMode() const
  {
    return buffer_mode;
  }

  /*!
   * \return Output port for reading current blackboard data
   */
  data_ports::tOutputPort<tBuffer> GetReadPort()
  {
    return read_port;
  }

  /*!
   * \return Revision of blackboard content (is incremented whenever blackboard content changes - signaling that a new version is available)
   */
  inline uint64_t GetRevisionCounter()
  {
    return tAbstractBlackboardServer::GetRevisionCounter();
  }

  /*!
   * \return Interface port for accessing blackboard
   */
  core::tPortWrapperBase GetWritePort()
  {
    return write_port;
  }

  /*!
   * (RPC Call)
   * Acquire read-only lock
   *
   * \param timeout Timeout for call
   * \return Future on locked buffer
   */
  rpc_ports::tFuture<tReadLockedBufferPointer> ReadLock(const rrlib::time::tDuration& timeout);

  /*!
   * (RPC Call)
   * Acquire read/write lock
   *
   * \param timeout Timeout for call
   * \return Future on locked buffer
   */
  rpc_ports::tFuture<tLockedBuffer<tBuffer>> WriteLock(tLockParameters lock_parameters);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tReadLockedBuffer;

  /*! Wraps read and write lock requests for enqueueing */
  class tLockRequest
  {
  public:

    /*! Was a write lock requested? */
    bool write_lock;

    /*! Promise for write lock, if write lock was requested */
    rpc_ports::tPromise<tLockedBuffer<tBuffer>> write_lock_promise;

    /*! Promise for read lock, if read lock was requested */
    rpc_ports::tPromise<tReadLockedBufferPointer> read_lock_promise;

    /*! Timeout for lock request */
    rrlib::time::tTimestamp timeout_time;

    /*! Is this a remote call? */
    bool remote_call;


    tLockRequest(rpc_ports::tPromise<tLockedBuffer<tBuffer>> && write_lock_promise, rrlib::time::tTimestamp timeout_time, bool remote_call) :
      write_lock(true),
      write_lock_promise(std::move(write_lock_promise)),
      read_lock_promise(),
      timeout_time(timeout_time),
      remote_call(remote_call)
    {}

    tLockRequest(rpc_ports::tPromise<tReadLockedBufferPointer> && read_lock_promise, rrlib::time::tTimestamp timeout_time) :
      write_lock(false),
      write_lock_promise(),
      read_lock_promise(std::move(read_lock_promise)),
      timeout_time(timeout_time),
      remote_call(false) // does not matter
    {}
  };

  /*! Output port for reading current blackboard data */
  data_ports::tOutputPort<tBuffer> read_port;

  /*! Interface port for accessing blackboard */
  core::tPortWrapperBase write_port;

  /*!
   * Queue with pending asynchronous change tasks
   * (They don't lock and don't execute in an extra thread)
   * (May only be accessed in synchronized context)
   */
  std::vector<tChangeSetPointer> pending_change_tasks;

  /*!
   * Queue with pending lock requests
   */
  std::deque<tLockRequest> pending_lock_requests;

  /*!
   * Buffer with current blackboard data
   * (we use raw tLockingManagerPointer instead of tPortDataPointer<T> to be able to duplicate pointer)
   */
  typename data_ports::standard::tStandardPort::tLockingManagerPointer current_buffer;

  /*! ID of current lock - to sort out obsolete unlocks */
  uint64_t lock_id;

  /*! Is blackboard currently locked for writing? */
  tWriteLock write_lock;

  typedef rpc_ports::tFuture<tLockedBufferData<tBuffer>> tUnlockFuture;

  /*! Future for blackboard unlock (when it is locked) */
  tUnlockFuture unlock_future;

  /*! Buffer mode that blackboard server is currently using */
  tBlackboardBufferMode buffer_mode;


  /*!
   * Applies change set to blackboard buffer
   *
   * \param blackboard_buffer Blackboard buffer to apply change set to
   * \param change_set Change set to appl
   */
  void ApplyAsynchronousChange(tBuffer& blackboard_buffer, tChangeSet& change_set)
  {
    for (auto it = change_set.begin(); it != change_set.end(); ++it)
    {
      it->Apply(blackboard_buffer);
    }
  }

  /*!
   * Applies all pending change tasks to provided buffer
   *
   * \param blackboard_buffer Blackboard buffer to apply deferred changes to
   */
  void ApplyPendingChangeTasks(tBuffer& blackboard_buffer)
  {
    for (auto it = pending_change_tasks.begin(); it != pending_change_tasks.end(); ++it)
    {
      ApplyAsynchronousChange(blackboard_buffer, **it);
    }
    pending_change_tasks.clear();
  }

  /*!
   * Clear and discard any pending change tasks
   */
  void ClearPendingChangeTasks()
  {
    pending_change_tasks.clear();
  }

  /*!
   * Check if updated buffer should be published and possibly do do
   */
  void ConsiderPublishing()
  {
    if (buffer_mode != tBlackboardBufferMode::SINGLE_BUFFERED)
      //if ((!single_buffered) || read_port.GetWrapped()->GetStrategy() > 0)  // TODO: Implementation needs to publish on strategy change, too (e.g. change log blackboard)
    {
      assert(!current_buffer->IsUnused());
      current_buffer->AddLocks(1);
      tConstBufferPointer pointer_clone(data_ports::standard::tStandardPort::tLockingManagerPointer(current_buffer.get()), *read_port.GetWrapped());
      read_port.Publish(pointer_clone);
      this->IncrementRevisionCounter();
    }
  }

  /*!
   * Copy a blackboard buffer
   * TODO: provide factory for buffer reuse
   *
   * \param src Source Buffer
   * \param target Target Buffer
   */
  static inline void CopyBlackboardBuffer(const tBuffer& src, tBuffer& target)
  {
    rrlib::rtti::GenericOperations<tBuffer>::DeepCopy(src, target);
  }

  /*!
   * If blackboard is currently locked, defers change set
   * execution until blackboard is unlocked again
   */
  void DeferAsynchronousChange(tChangeSetPointer change_set)
  {
    pending_change_tasks.push_back(std::move(change_set));
  }

  virtual void HandleException(rpc_ports::tFutureStatus exception_type) override;

  void HandleReadUnlock(tReadLockedBufferPointer& unlock);

  virtual void HandleResponse(tLockedBufferData<tBuffer> call_result) override;

  /*!
   * Replaces 'current_buffer' with new buffer that is unique
   *
   * \param copy_current_buffer Copy contents of current buffer to new unique buffer?
   */
  void NewCurrentBuffer(bool copy_current_buffer)
  {
    data_ports::standard::tStandardPort::tUnusedManagerPointer unused_buffer = read_port.GetWrapped()->GetUnusedBufferRaw();
    unused_buffer->SetUnused(false);
    unused_buffer->InitReferenceCounter(1);
    if (copy_current_buffer)
    {
      CopyBlackboardBuffer(current_buffer->GetObject().GetData<tBuffer>(), unused_buffer->GetObject().GetData<tBuffer>());
    }
    current_buffer.reset(unused_buffer.release());
  }

  virtual void OnManagedDelete() override
  {
    {
      rrlib::thread::tLock lock(this->BlackboardMutex());
      lock_id = std::numeric_limits<uint64_t>::max();
      unlock_future = tUnlockFuture();
    }
    tAbstractBlackboardServer::OnManagedDelete();
    read_port.ManagedDelete();
    write_port.ManagedDelete();
  }

  /*!
   * Processes deferred lock requests
   */
  void ProcessPendingLockRequests();

  /*!
   * Common functionality needed in WriteLock and ProcessPendingLockRequests functions
   */
  void WriteLockImplementation(rpc_ports::tPromise<tLockedBuffer<tBuffer>>& promise, bool remote_call);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "plugins/blackboard/internal/tBlackboardServer.hpp"

#endif
