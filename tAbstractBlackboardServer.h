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
#include "finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__BLACKBOARD__TABSTRACTBLACKBOARDSERVER_H
#define PLUGINS__BLACKBOARD__TABSTRACTBLACKBOARDSERVER_H

#include "core/portdatabase/tDataType.h"
#include "core/port/std/tPortBase.h"
#include "core/port/rpc/tInterfacePort.h"
#include "blackboard/tBlackboardManager.h"
#include "finroc_core_utils/container/tSimpleList.h"
#include "blackboard/tBlackboardTask.h"
#include "core/port/rpc/method/tPortInterface.h"
#include "core/port/rpc/method/tPort1Method.h"
#include "blackboard/tBlackboardBuffer.h"
#include "core/port/rpc/method/tPort2Method.h"
#include "core/port/rpc/method/tVoid1Method.h"
#include "core/port/rpc/method/tVoid2Method.h"
#include "core/port/rpc/method/tPort3Method.h"
#include "core/port/rpc/method/tPort0Method.h"
#include "core/tFrameworkElement.h"
#include "core/port/rpc/method/tAbstractMethod.h"
#include "core/port/rpc/method/tAbstractMethodCallHandler.h"

namespace finroc
{
namespace blackboard
{
/*! Blackboard info */
struct tAbstractBlackboardServer : public core::tFrameworkElement, public core::tAbstractMethodCallHandler
{
protected:

  /*!
   * Queue with pending major commands (e.g. LOCK, READ_PART in SingleBufferedBlackboard)
   * They are executed in another thread
   * may only be accessed in synchronized context */
  util::tSimpleList<tBlackboardTask*> pending_major_tasks;

  /*!
   * Queue with pending asynch change commands
   * they don't lock and don't execute in an extra thread
   * may only be accessed in synchronized context
   */
  util::tSimpleList<tBlackboardTask*> pending_asynch_change_tasks;

  /*! Uid of thread that is allowed to wake up now - after notifyAll() - thread should reset this to -1 as soon as possible */
  int64 wakeup_thread;

public:

  // for monitor functionality
  mutable util::tMonitor monitor;

  /*! read port */
  core::tPortBase* read_port;

  /*! write port */
  core::tInterfacePort* write_port;

  /*! Category index of Blackboard category that this server belongs to (see constants in BlackboardManager) */
  int category_index;

  /*! Blackboard category that this server belongs to */
  tBlackboardManager::tBlackboardCategory* my_category;

  // Methods...

  /*! Blackboard interface */
  static core::tPortInterface cMETHODS;

  /*! Write Lock */
  static core::tPort1Method<tAbstractBlackboardServer*, tBlackboardBuffer*, int64> cLOCK;

  /*! Read Lock (only useful for SingleBufferedBlackboardBuffers) */
  static core::tPort2Method<tAbstractBlackboardServer*, const tBlackboardBuffer*, int64, int> cREAD_LOCK;

  /*! Write Unlock */
  static core::tVoid1Method<tAbstractBlackboardServer*, tBlackboardBuffer*> cUNLOCK;

  /*! Read Unlock */
  static core::tVoid1Method<tAbstractBlackboardServer*, int> cREAD_UNLOCK;

  /*! Asynch Change */
  static core::tVoid2Method<tAbstractBlackboardServer*, int, const tBlackboardBuffer*> cASYNCH_CHANGE;

  /*! Read part of blackboard (no extra thread with multi-buffered blackboards) */
  static core::tPort3Method<tAbstractBlackboardServer*, tBlackboardBuffer*, int, int, int> cREAD_PART;

  /*! Directly commit buffer */
  static core::tVoid1Method<tAbstractBlackboardServer*, tBlackboardBuffer*> cDIRECT_COMMIT;

  /*! Is server a single-buffered blackboard server? */
  static core::tPort0Method<tAbstractBlackboardServer*, int8> cIS_SINGLE_BUFFERED;

  /*! Is server a single-buffered blackboard server? */
  static core::tVoid1Method<tAbstractBlackboardServer*, int> cKEEP_ALIVE;

protected:

  /*!
   * Asynchronous change to blackboard
   *
   * \param offset Offset in blackboard
   * \param buf Buffer with contents to write there
   * \param check_lock Check whether buffer is currently locked, before performing asynch change (normal operation)
   */
  virtual void AsynchChange(int i, const tBlackboardBuffer* buf, bool check_lock) = 0;

  /*!
   * (only call in synchronized context)
   * Clear any asynch change tasks from list
   */
  void ClearAsyncChangeTasks();

  /*!
   * (only call in synchronized context)
   * Process change command later
   *
   * \param offset Offset offset to start writing
   * \param buf (Locked) buffer with contents to write
   */
  void DeferAsynchChangeCommand(int offset, const tBlackboardBuffer* buf);

  /*!
   * Direct commit of new blackboard buffer
   *
   * \param buf New Buffer
   */
  virtual void DirectCommit(tBlackboardBuffer* buf) = 0;

  /*!
   * \return Unused blackboard task for pending tasks
   */
  tBlackboardTask* GetUnusedBlackboardTask();

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
  inline bool PendingTasks()
  {
    return (pending_major_tasks.Size() > 0) || (wakeup_thread != -1);
  }

  virtual void PostChildInit()
  {
    my_category->Add(this);
  }

  virtual void PrepareDelete();

  /*!
   * (only call in synchronized context)
   * Process pending asynch change commands (good idea when unlocking blackboard)
   */
  void ProcessPendingAsynchChangeTasks();

  /*!
   * Execute any pending tasks
   * (may only be called as last statement in synchronized context - and when there's no lock)
   */
  void ProcessPendingCommands(util::tLock& passed_lock);

  /*!
   * Perform read lock (only do this on single-buffered blackboards)
   *
   * \param timeout Timeout in ms
   * \return Locked BlackboardBuffer (if lockId is < 0 this is a copy)
   */
  virtual const tBlackboardBuffer* ReadLock(int64 timeout) = 0;

  /*!
   * Read part of blackboard
   *
   * \param offset Offset to start reading
   * \param length Length (in bytes) of area to read
   * \param timeout Timeout for this command
   * \return Memory buffer containing read area
   */
  virtual tBlackboardBuffer* ReadPart(int offset, int length, int timeout) = 0;

  /*!
   * Unlock blackboard (from read lock)
   *
   * \param lock_id LockId from origin
   */
  virtual void ReadUnlock(int lock_id) = 0;

  finroc::util::tLock* curlock;

  /*!
   * Wait to receive lock on blackboard for specified amount of time
   * (MUST be called in synchronized context)
   *
   * \param timeout Time to wait for lock
   * \return Do we have a lock now? (or did rather timeout expire?)
   */
  bool WaitForLock(util::tLock& passed_lock, int64 timeout);

  /*!
   * Perform read lock (only do this on single-buffered blackboards)
   *
   * \param timeout Timeout in ms
   * \return Locked BlackboardBuffer (if lockId is < 0 this is a copy)
   */
  virtual tBlackboardBuffer* WriteLock(int64 timeout) = 0;

  /*!
   * Unlock blackboard (from write lock)
   *
   * \param buf Buffer containing changes (may be the same or another one - the latter is typically the case with remote clients)
   */
  virtual void WriteUnlock(tBlackboardBuffer* buf) = 0;

public:

  /*!
   * \param bb_name Blackboard name
   * \param category Blackboard category (see constants in BlackboardManager)
   */
  tAbstractBlackboardServer(const util::tString& bb_name, int category, core::tFrameworkElement* parent = NULL);

  /*!
   * Check whether this is a valid data type for blackboards
   *
   * \param dt Data type to check
   */
  inline static void CheckType(core::tDataType* dt)
  {
    assert(((dt->GetRelatedType() != NULL && dt->GetRelatedType()->IsMethodType())) && "Please register Blackboard types using Blackboard2Plugin class");
  }

  /*!
   * Copy a blackboard buffer
   *
   * \param src Source Buffer
   * \param target Target Buffer
   */
  inline void CopyBlackboardBuffer(tBlackboardBuffer* src, tBlackboardBuffer* target)
  {
    target->Resize(src->GetBbCapacity(), src->GetElements(), src->GetElementSize(), false);
    target->GetBuffer()->Put(0u, *src->GetBuffer(), 0u, src->GetSize());
  }

  void HandleVoidCall(core::tAbstractMethod* method, tBlackboardBuffer* p1)
  {
    if (method == &cDIRECT_COMMIT)
    {
      DirectCommit(p1);
    }
    else if (method == &cUNLOCK)
    {
      WriteUnlock(p1);
    }
    else
    {
      cLOCK.Cleanup(p1);
      throw core::tMethodCallException(core::tMethodCallException::eUNKNOWN_METHOD);
    }
  }

  void HandleVoidCall(core::tAbstractMethod* method, int p1)
  {
    if (method == &cKEEP_ALIVE)
    {
      KeepAlive(p1);
    }
    else if (method == &cREAD_UNLOCK)
    {
      ReadUnlock(p1);
    }
    else
    {
      cLOCK.Cleanup(p1);
      throw core::tMethodCallException(core::tMethodCallException::eUNKNOWN_METHOD);
    }
  }

  inline tBlackboardBuffer* HandleCall(const core::tAbstractMethod* method, int p1, int p2, int p3)
  {
    assert((method == &(cREAD_PART)));
    return ReadPart(p1, p2, p3);
  }

  inline tBlackboardBuffer* HandleCall(const core::tAbstractMethod* method, int64 p1)
  {
    assert((method == &(cLOCK)));
    return WriteLock(p1);
  }

  inline const tBlackboardBuffer* HandleCall(const core::tAbstractMethod* method, int64 p1, int dummy)
  {
    assert((method == &(cREAD_LOCK)));
    return ReadLock(p1);
  }

  inline int8 HandleCall(const core::tAbstractMethod* method)
  {
    return IsSingleBuffered() ? static_cast<int8>(1) : static_cast<int8>(0);
  }

  inline void HandleVoidCall(const core::tAbstractMethod* method, int p1, const tBlackboardBuffer* p2)
  {
    assert((method == &(cASYNCH_CHANGE)));
    AsynchChange(p1, p2, true);
  }

  /*!
   * Check whether lock has timed out
   */
  virtual void LockCheck() = 0;

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TABSTRACTBLACKBOARDSERVER_H
