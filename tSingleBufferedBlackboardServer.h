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
#include "rrlib/finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__BLACKBOARD__TSINGLEBUFFEREDBLACKBOARDSERVER_H
#define PLUGINS__BLACKBOARD__TSINGLEBUFFEREDBLACKBOARDSERVER_H

#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "core/port/std/tPullRequestHandler.h"
#include "core/port/tPortCreationInfo.h"
#include "core/port/std/tPortBase.h"

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
class tBlackboardBuffer;

/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard server
 */
class tSingleBufferedBlackboardServer : public tAbstractBlackboardServer, public core::tPullRequestHandler
{
public:
  class tBBReadPort; // inner class forward declaration
private:

  /*! Unlock timeout in ms - if no keep-alive signal occurs in this period of time */
  static const int64 cUNLOCK_TIMEOUT = 1000;

  /*! Interface port for write access */
  core::tInterfaceServerPort* write;

  /*!
   * this is the one and only blackboard buffer
   * (can be replaced, when new buffers arrive from network => don't store pointers to it, when not locked)
   * (has exactly one lock)
   */
  tBlackboardBuffer* buffer;

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
  volatile int64 lock_time;

  /*! Last time a keep-alive-signal was received */
  volatile int64 last_keep_alive;

  /*! ID of current lock - against outdated unlocks */
  util::tAtomicInt lock_iDGen;

  int lock_id;

  /*! revision of blackboard (incremented after each unlock) */
  int64 revision;

  /*! Current read copy of blackboard */
  tBlackboardBuffer* read_copy;

  /*! revision of read copy */
  int64 read_copy_revision;

  /*! Is a thread waiting for a blackboard copy? */
  volatile bool thread_waiting_for_copy;

  /*! Check if lock timed out (only call in synchronized/exclusive access context) */
  void CheckCurrentLock(util::tLock& passed_lock);

  void NewBufferRevision(util::tLock& passed_lock, bool has_changes);

  /*!
   * Helper method for above to avoid nested/double lock
   */
  tBlackboardBuffer* ReadLockImpl(util::tLock& passed_lock, int64 timeout);

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
  void WaitForReadCopy(util::tLock& passed_lock, int64 min_revision, int64 timeout);

protected:

  virtual void AsynchChange(int offset, const tBlackboardBuffer* buf, bool check_lock);

  virtual void DirectCommit(tBlackboardBuffer* new_buffer);

  virtual bool IsLocked()
  {
    return locks != 0;
  }

  virtual bool IsSingleBuffered()
  {
    return true;
  }

  virtual void KeepAlive(int lock_id_);

  virtual const tBlackboardBuffer* ReadLock(int64 timeout);

  virtual tBlackboardBuffer* ReadPart(int offset, int length, int timeout);

  virtual void ReadUnlock(int lock_id_);

  virtual tBlackboardBuffer* WriteLock(int64 timeout);

  virtual void WriteUnlock(tBlackboardBuffer* buf);

public:

  /*!
   * \param description Name/Uid of blackboard
   * @parent parent of BlackboardServer
   */
  tSingleBufferedBlackboardServer(const util::tString& description, core::tFrameworkElement* parent = NULL);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param capacity Blackboard capacity (see BlackboardBuffer)
   * \param elements Number of element (see BlackboardBuffer)
   * \param elem_size Element size (see BlackboardBuffer)
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tSingleBufferedBlackboardServer(const util::tString& description, core::tDataType* type, int capacity, int elements, int elem_size, core::tFrameworkElement* parent = NULL, bool shared = true);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param mc_type Type of method calls
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tSingleBufferedBlackboardServer(const util::tString& description, core::tDataType* type, core::tFrameworkElement* parent = NULL, bool shared = true);

  virtual ~tSingleBufferedBlackboardServer();

  virtual void GetSizeInfo(size_t& element_size, size_t& elements, size_t& capacity);

  inline int8 HandleCall(const core::tAbstractMethod* method)
  {
    assert((method == &(cIS_SINGLE_BUFFERED)));
    return 1;
  }

  virtual void LockCheck();

  virtual const core::tPortData* PullRequest(core::tPortBase* origin, int8 add_locks);

  /*! Special read port for blackboard buffer */
  class tBBReadPort : public core::tPortBase
  {
  private:

    // Outer class SingleBufferedBlackboardServer
    tSingleBufferedBlackboardServer* const outer_class_ptr;

  protected:

    virtual void InitialPushTo(core::tAbstractPort* target, bool reverse);

  public:

    tBBReadPort(tSingleBufferedBlackboardServer* const outer_class_ptr_, core::tPortCreationInfo pci);

  };

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TSINGLEBUFFEREDBLACKBOARDSERVER_H
