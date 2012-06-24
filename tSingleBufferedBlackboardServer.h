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

#ifndef plugins__blackboard__tSingleBufferedBlackboardServer_h__
#define plugins__blackboard__tSingleBufferedBlackboardServer_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/rtti/tDataTypeBase.h"

#include "core/port/std/tPullRequestHandlerRaw.h"
#include "core/port/tPortCreationInfo.h"
#include "core/port/std/tPortBase.h"

#include "plugins/blackboard/tAbstractBlackboardServer.h"

namespace finroc
{
namespace core
{
class tInterfaceServerPort;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard server
 */
template<typename T>
class tSingleBufferedBlackboardServer : public tAbstractBlackboardServer<T>, public core::tPullRequestHandlerRaw
{
public:
  class tBBReadPort; // inner class forward declaration
private:

  typedef typename tAbstractBlackboardServer<T>::tBBVector tBBVector;
  typedef typename tAbstractBlackboardServer<T>::tBBVectorVar tBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tConstBBVectorVar tConstBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransaction tChangeTransaction;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransactionVar tChangeTransactionVar;
  typedef typename tAbstractBlackboardServer<T>::tConstChangeTransactionVar tConstChangeTransactionVar;

  /*! Interface port for write access */
  core::tInterfaceServerPort* write;

  /*!
   * this is the one and only blackboard buffer
   * (can be replaced, when new buffers arrive from network => don't store pointers to it, when not locked)
   * (has exactly one lock)
   */
  tBBVectorVar buffer;

  /*!
   * Is blackboard currently locked?
   * (write locks are stored in this boolean, read locks in reference counter - this class holds one reference counter)
   *
   * positive numbers: # read locks
   * 0: no locks
   * -1: write lock
   */
  int locks;

  /*! Time when last lock was performed */
  rrlib::time::tAtomicTimestamp lock_time;

  /*! Last time a keep-alive-signal was received */
  rrlib::time::tAtomicTimestamp last_keep_alive;

  /*! ID of current lock - against outdated unlocks */
  util::tAtomicInt lock_id_gen;

  int lock_id;

  /*! revision of blackboard (incremented after each unlock) */
  int64 revision;

  /*! Current read copy of blackboard */
  tBBVectorVar read_copy;

  /*! revision of read copy */
  int64 read_copy_revision;

  /*! Is a thread waiting for a blackboard copy? */
  std::atomic<bool> thread_waiting_for_copy;

  /*! Check if lock timed out (only call in synchronized/exclusive access context) */
  void CheckCurrentLock(util::tLock& passed_lock);

  /*!
   * \return Port data manager for buffer
   */
  template <typename Q>
  inline static core::tPortDataManager* GetManager(core::tPortDataPtr<Q>& t)
  {
    return t.GetManager();
  }

  void NewBufferRevision(util::tLock& passed_lock, bool has_changes);

  /*!
   * Helper method for above to avoid nested/double lock
   */
  typename tAbstractBlackboardServer<T>::tConstBBVectorVar ReadLockImpl(util::tLock& passed_lock, const rrlib::time::tDuration& timeout);

  /*!
   * Helper method for above to avoid nested/double lock
   */
  void ReadUnlockImpl(util::tLock& passed_lock, int lock_id_);

  /*!
   * Make a copy for the read port - and hand it to anyone who is interested
   */
  void UpdateReadCopy(util::tLock& passed_lock);

  /*!
   * wait until a read copy has been made
   * (only call in synchronized context)
   *
   * \param min_revision minimal revision we want to receive
   */
  void WaitForReadCopy(util::tLock& passed_lock, int64 min_revision, const rrlib::time::tDuration& timeout);

protected:

  virtual void AsynchChange(tConstChangeTransactionVar& buf, int index, int offset, bool check_lock);

  virtual void DirectCommit(tBBVectorVar& new_buffer);

  virtual bool IsLocked()
  {
    assert(buffer.get() != NULL && buffer.GetManager() != NULL);
    return locks != 0;
  }

  virtual bool IsSingleBuffered()
  {
    return true;
  }

  virtual void KeepAlive(int lock_id_);

  virtual typename tAbstractBlackboardServer<T>::tConstBBVectorVar ReadLock(const rrlib::time::tDuration& timeout);

  virtual void ReadUnlock(int lock_id_);

  virtual typename tAbstractBlackboardServer<T>::tBBVectorVar WriteLock(const rrlib::time::tDuration& timeout);

  virtual void WriteUnlock(tBBVectorVar& buf);

public:

  /*!
   * \param name Name/Uid of blackboard
   * \param capacity Blackboard capacity (see BlackboardBuffer)
   * \param elements Number of element (see BlackboardBuffer)
   * \param elem_size Element size (see BlackboardBuffer)
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   * \param type Data Type of blackboard content
   */
  tSingleBufferedBlackboardServer(const util::tString& name, int capacity, int elements, int elem_size, core::tFrameworkElement* parent = NULL, bool shared = true, rrlib::rtti::tDataTypeBase type = rrlib::rtti::tDataType<T>());

  /*!
   * \param name Name/Uid of blackboard
   * \param elements Initial number of elements
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   * \param type Data Type of blackboard content
   */
  tSingleBufferedBlackboardServer(const util::tString& name, int elements = 0, core::tFrameworkElement* parent = NULL, bool shared = true, rrlib::rtti::tDataTypeBase type = rrlib::rtti::tDataType<T>());

  virtual ~tSingleBufferedBlackboardServer();

  virtual void LockCheck();

  virtual const core::tPortDataManager* PullRequest(core::tPortBase* origin, int8 add_locks, bool intermediate_assign);

  /*! Special read port for blackboard buffer */
  class tBBReadPort : public core::tPortBase
  {
  private:

    // Outer class SingleBufferedBlackboardServer
    tSingleBufferedBlackboardServer* const outer_class_ptr;

  protected:

    virtual void InitialPushTo(core::tAbstractPort* target, bool reverse);

  public:

    tBBReadPort(tSingleBufferedBlackboardServer* const outer_class_ptr_, core::tPortCreationInfoBase pci);

  };

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tSingleBufferedBlackboardServer.hpp"

namespace finroc
{
namespace blackboard
{
extern template class tSingleBufferedBlackboardServer<tBlackboardBuffer>;
extern template class tSingleBufferedBlackboardServer<rrlib::serialization::tMemoryBuffer>;

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tSingleBufferedBlackboardServer_h__
