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

#ifndef plugins__blackboard__tAbstractBlackboardServer_h__
#define plugins__blackboard__tAbstractBlackboardServer_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "core/port/rpc/method/tPortInterface.h"
#include "core/port/rpc/method/tPort1Method.h"
#include "core/port/rpc/method/tPort2Method.h"
#include "core/port/rpc/method/tVoid1Method.h"
#include "core/port/rpc/method/tVoid3Method.h"
#include "core/port/rpc/method/tPort0Method.h"
#include "rrlib/finroc_core_utils/thread/sThreadUtil.h"
#include "rrlib/finroc_core_utils/container/tSimpleList.h"
#include "plugins/blackboard/tBlackboardTask.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"

#include "core/port/tPortTypeMap.h"

namespace finroc
{
namespace core
{
class tFrameworkElement;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
/*!
 * Abstract base class of all blackboard servers - typed version
 */
template<typename T>
struct tAbstractBlackboardServer : public tAbstractBlackboardServerRaw
{
public:

  typedef typename core::tPortTypeMap<T>::tListType tBBVector;
  typedef typename core::tPortDataPtr<tBBVector> tBBVectorVar;
  typedef typename core::tPortDataPtr<const tBBVector> tConstBBVectorVar;
  typedef typename core::tPortTypeMap<tBBVector>::tGenericChange tChangeTransaction;
  typedef typename core::tPortDataPtr<tChangeTransaction> tChangeTransactionVar;
  typedef typename core::tPortDataPtr<const tChangeTransaction> tConstChangeTransactionVar;

  using tAbstractBlackboardServerRaw::HandleCall;

  class tAsynchChangeTask : public tBlackboardTask
  {
  public:

    // BlackboardBuffer to use for task - if this is set, it will be unlocked with recycle
    tConstChangeTransactionVar buffer;

    // Offset for asynch change command
    int64_t offset;

    // index for asynch change command
    int64_t index;

    tAsynchChangeTask(tConstChangeTransactionVar && buffer_, int64_t offset_, int64_t index_) :
      buffer(),
      offset(offset_),
      index(index_)
    {
      buffer = std::move(buffer);
    }

    tAsynchChangeTask(tAsynchChangeTask && o) :
      buffer(),
      offset(0),
      index(0)
    {
      std::swap(buffer, o.buffer);
      std::swap(offset, o.offset);
      std::swap(index, o.index);
    }

    tAsynchChangeTask& operator=(tAsynchChangeTask && o)
    {
      std::swap(buffer, o.buffer);
      std::swap(offset, o.offset);
      std::swap(index, o.index);
      return *this;
    }
  };

  /*!
   * Queue with pending asynch change commands
   * they don't lock and don't execute in an extra thread
   * may only be accessed in synchronized context
   */
  std::vector<tAsynchChangeTask> pending_asynch_change_tasks;

  /*! Write Lock */
  static typename core::tPort1Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tBBVectorVar, int> cLOCK;

  /*! Read Lock (only useful for SingleBufferedBlackboardBuffers) */
  static typename core::tPort2Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tConstBBVectorVar, int, int> cREAD_LOCK;

  /*! Write Unlock */
  static typename core::tVoid1Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tBBVectorVar> cUNLOCK;

  /*! Read Unlock */
  static core::tVoid1Method<tAbstractBlackboardServer<T>*, int> cREAD_UNLOCK;

  /*! Asynch Change */
  static typename core::tVoid3Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tConstChangeTransactionVar, int, int> cASYNCH_CHANGE;

  //    /** Read part of blackboard (no extra thread with multi-buffered blackboards) */
  //    @PassByValue public static Port3Method<AbstractBlackboardServer, BlackboardBuffer, Integer, Integer, Integer> READ_PART =
  //        new Port3Method<AbstractBlackboardServer, BlackboardBuffer, Integer, Integer, Integer>(METHODS, "Read Part", "Offset", "Length", "Timeout", true);

  /*! Directly commit buffer */
  static typename core::tVoid1Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tBBVectorVar> cDIRECT_COMMIT;

  /*! Is server a single-buffered blackboard server? */
  static core::tPort0Method<tAbstractBlackboardServer<T>*, int8> cIS_SINGLE_BUFFERED;

  /*! Send keep-alive signal for lock */
  static core::tVoid1Method<tAbstractBlackboardServer<T>*, int> cKEEP_ALIVE;

protected:

  /*!
   * Apply asynch change to blackboard
   *
   * \param bb blackboard buffer
   * \param changes Changes to apply
   */
  inline void ApplyAsynchChange(tBBVector& bb, tConstChangeTransactionVar& changes, int index, int offset)
  {
    core::typeutil::ApplyChange(bb, *changes, index, offset);
  }

  /*!
   * Asynchronous change to blackboard
   *
   * \param buf Buffer with contents to write there
   * \param start_idx Start index in blackboard
   * \param offset Custom optional Offset in element type
   * \param check_lock Check whether buffer is currently locked, before performing asynch change (normal operation)
   */
  virtual void AsynchChange(tConstChangeTransactionVar& buf, int start_idx, int offset, bool check_lock) = 0;

  /*!
   * (only call in synchronized context)
   * Clear any asynch change tasks from list
   */
  virtual void ClearAsyncChangeTasks();

  // finroc::util::tLock* curlock;

  /*!
   * \return Thread string (debug helper method)
   */
  virtual util::tString CreateThreadString()
  {
    return util::tStringBuilder("Thread ") + util::tThread::CurrentThread()->GetName() + " (" + util::sThreadUtil::GetCurrentThreadId() + ")";
  }

  /*!
   * (only call in synchronized context)
   * Process change command later
   *
   * \param offset Offset offset to start writing
   * \param buf (Locked) buffer with contents to write
   */
  inline void DeferAsynchChangeCommand(tConstChangeTransactionVar& buf, int index, int offset)
  {
    pending_asynch_change_tasks.push_back(tAsynchChangeTask(std::move(buf), offset, index));
  }

  /*!
   * Direct commit of new blackboard buffer
   *
   * \param buf New Buffer
   */
  virtual void DirectCommit(tBBVectorVar& buf) = 0;

  /*!
   * Helper for constructor
   *
   * \param dt DataType T
   * \return Blackboard method type of write ports
   */
  rrlib::rtti::tDataTypeBase GetBlackboardMethodType(rrlib::rtti::tDataTypeBase dt);

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
    return (this->pending_major_tasks.Size() > 0) || (this->wakeup_thread != -1);
  }

  virtual void PostChildInit();

  /*!
   * (only call in synchronized context)
   * Process pending asynch change commands (good idea when unlocking blackboard)
   */
  void ProcessPendingAsynchChangeTasks();

  /*!
   * Execute any pending tasks
   * (may only be called as last statement in synchronized context - and when there's no lock)
   *
   * \return Were there any pending commands that are (were) now executed?
   */
  virtual bool ProcessPendingCommands(util::tLock& passed_lock);

  /*!
   * Perform read lock (only do this on single-buffered blackboards)
   *
   * \param timeout Timeout in ms
   * \return Locked BlackboardBuffer (if lockId is < 0 this is a copy)
   */
  virtual typename tAbstractBlackboardServer<T>::tConstBBVectorVar ReadLock(int64 timeout) = 0;

  /*!
   * Unlock blackboard (from read lock)
   *
   * \param lock_id LockId from origin
   */
  virtual void ReadUnlock(int lock_id) = 0;

  /*!
   * Resize blackboard
   *
   * \param buf Buffer to resize
   * \param new_capacity new Capacity
   * \param new_elements new current number of elements
   */
  inline void Resize(tBBVector& buf, int new_capacity, int new_elements)
  {
    rrlib::rtti::ResizeVector(buf, new_elements);
  }

  /*!
   * Perform read lock (only do this on single-buffered blackboards)
   *
   * \param timeout Timeout in ms
   * \return Locked BlackboardBuffer (if lockId is < 0 this is a copy)
   */
  virtual typename tAbstractBlackboardServer<T>::tBBVectorVar WriteLock(int64 timeout) = 0;

  /*!
   * Unlock blackboard (from write lock)
   *
   * \param buf Buffer containing changes (may be the same or another one - the latter is typically the case with remote clients)
   */
  virtual void WriteUnlock(tBBVectorVar& buf) = 0;

public:

  /*!
   * \param bb_name Blackboard name
   * \param category Blackboard category (see constants in BlackboardManager)
   */
  tAbstractBlackboardServer(const util::tString& bb_name, int category, core::tFrameworkElement* parent = NULL);

  /*!
   * Copy a blackboard buffer
   * TODO: provide factory for buffer reuse
   *
   * \param src Source Buffer
   * \param target Target Buffer
   */
  inline void CopyBlackboardBuffer(const tBBVector& src, tBBVector& target)
  {
    rrlib::rtti::sStaticTypeInfo<tBBVector>::DeepCopy(src, target, NULL);
  }

  inline static core::tPortInterface& GetBlackboardInterface()
  {
    static core::tPortInterface pi("Blackboard Interface", true);
    return pi;
  }

  void HandleVoidCall(core::tAbstractMethod* method, tBBVectorVar& p1)
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

  //    @Override
  //    public BlackboardBuffer handleCall(AbstractMethod method, Integer p1, Integer p2, Integer p3) throws MethodCallException {
  //        assert(method == READ_PART);
  //        return readPart(p1, p2, p3);
  //    }

  inline tBBVectorVar HandleCall(const core::tAbstractMethod* method, int p1)
  {
    assert((method == &(cLOCK)));
    return WriteLock(p1);
  }

  inline tConstBBVectorVar HandleCall(const core::tAbstractMethod* method, int p1, int dummy)
  {
    assert((method == &(cREAD_LOCK)));
    return ReadLock(p1);
  }

  inline void HandleVoidCall(const core::tAbstractMethod* method, tConstChangeTransactionVar& p2, int index, int offset)
  {
    assert((method == &(cASYNCH_CHANGE)));
    AsynchChange(p2, index, offset, true);
  }

  /*!
   * Check whether lock has timed out
   */
  virtual void LockCheck() = 0;

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tAbstractBlackboardServer.hpp"

#endif // plugins__blackboard__tAbstractBlackboardServer_h__
