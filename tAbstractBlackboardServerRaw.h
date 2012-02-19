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

#ifndef plugins__blackboard__tAbstractBlackboardServerRaw_h__
#define plugins__blackboard__tAbstractBlackboardServerRaw_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/finroc_core_utils/container/tSimpleList.h"
#include "rrlib/finroc_core_utils/thread/sThreadUtil.h"
#include "rrlib/rtti/tDataTypeBase.h"

#include "core/port/rpc/tInterfacePort.h"
#include "core/tLockOrderLevels.h"
#include "core/tFrameworkElement.h"
#include "core/port/rpc/method/tAbstractMethodCallHandler.h"
#include "core/port/rpc/method/tAbstractMethod.h"

#include "plugins/blackboard/tBlackboardBuffer.h"
#include "plugins/blackboard/tBlackboardTask.h"
#include "plugins/blackboard/tBlackboardTypeInfo.h"
#include "plugins/blackboard/tBlackboardManager.h"

namespace finroc
{
namespace core
{
class tPortBase;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
/*!
 * Abstract base class of all blackboard servers
 */
struct tAbstractBlackboardServerRaw : public core::tFrameworkElement, public core::tAbstractMethodCallHandler
{
protected:

  /*!
   * Queue with pending major commands (e.g. LOCK, READ_PART in SingleBufferedBlackboard)
   * They are executed in another thread
   * may only be accessed in synchronized context */
  util::tSimpleList<tBlackboardTask> pending_major_tasks;

  /*! Uid of thread that is allowed to wake up now - after notifyAll() - thread should reset this to -1 as soon as possible */
  int64 wakeup_thread;

public:

  /*! Lock for blackboard operation (needs to be deeper than runtime - (for initial pushes etc.)) */
  util::tMutexLockOrderWithMonitor bb_lock;

  /*! read port */
  core::tPortBase* read_port_raw;

  /*! write port */
  core::tInterfacePort* write_port_raw;

  /*! Category index of Blackboard category that this server belongs to (see constants in BlackboardManager) */
  int category_index;

  /*! Blackboard category that this server belongs to */
  tBlackboardManager::tBlackboardCategory* my_category;

protected:

  /*!
   * Resize for Blackboards based on BlackboardBuffer (such as class MCA-style ones)
   *
   * \param buffer Blackboard buffer to resize
   * \param capacity Blackboard capacity (see BlackboardBuffer)
   * \param elements Number of element (see BlackboardBuffer)
   * \param elem_size Element size (see BlackboardBuffer)
   */
  inline void ClassicBlackboardResize(tBlackboardBuffer* buffer, int capacity, int elements, int elem_size)
  {
    buffer->Resize(capacity, elements, elem_size, true);
  }

  /*!
   * Overload for non-blackboard-types
   */
  inline void ClassicBlackboardResize(void* o, int capacity, int elements, int elem_size)
  {
  }

  /*!
   * (only call in synchronized context)
   * Clear any asynch change tasks from list
   */
  virtual void ClearAsyncChangeTasks() = 0;

  /*!
   * \return Thread string (debug helper method)
   */
  virtual util::tString CreateThreadString()
  {
    return util::tStringBuilder("Thread ") + util::tThread::CurrentThread()->GetName() + " (" + util::sThreadUtil::GetCurrentThreadId() + ")";
  }

  /*!
   * \return Is blackboard currently locked?
   */
  virtual bool IsLocked() = 0;

  /*!
   * \return Is this a single buffered blackboard server?
   */
  virtual bool IsSingleBuffered() = 0;

  // methods that need to be implemented

  /*!
   * Keep-alive signal received
   *
   * \param lock_id LockId from origin
   */
  virtual void KeepAlive(int lock_id) = 0;

  /*!
   * \return Does blackboard have pending commands that are waiting for execution?
   */
  virtual bool PendingTasks()
  {
    return (pending_major_tasks.Size() > 0) || (wakeup_thread != -1);
  }

  virtual void PostChildInit()
  {
    my_category->Add(this);
  }

  virtual void PrepareDelete();

  /*!
   * Execute any pending tasks
   * (may only be called as last statement in synchronized context - and when there's no lock)
   *
   * \return Were there any pending commands that are (were) now executed?
   */
  virtual bool ProcessPendingCommands(util::tLock& passed_lock);

  //
  //    // finroc::util::tLock* curlock;

  /*!
   * Wait to receive lock on blackboard for specified amount of time
   * (MUST be called in synchronized context)
   *
   * \param timeout Time to wait for lock
   * \return Do we have a lock now? (or did rather timeout expire?)
   */
  bool WaitForLock(util::tLock& passed_lock, int64 timeout);

public:

  /*!
   * \param bb_name Blackboard name
   * \param category Blackboard category (see constants in BlackboardManager)
   */
  tAbstractBlackboardServerRaw(const util::tString& bb_name, int category, core::tFrameworkElement* parent = NULL);

  /*!
   * Check whether this is a valid data type for blackboards
   *
   * \param dt Data type to check
   */
  static void CheckType(rrlib::rtti::tDataTypeBase dt);

  /*!
   * \param dt Data type
   * \return Blackboard type info for data type
   */
  inline static tBlackboardTypeInfo* GetBlackboardTypeInfo(rrlib::rtti::tDataTypeBase dt)
  {
    return dt.GetAnnotation<tBlackboardTypeInfo>();
  }

  // Call handling

  inline int8 HandleCall(const core::tAbstractMethod& method)
  {
    return IsSingleBuffered() ? static_cast<int8>(1) : static_cast<int8>(0);
  }

  /*!
   * Check whether lock has timed out
   */
  virtual void LockCheck() = 0;

};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tAbstractBlackboardServerRaw_h__
