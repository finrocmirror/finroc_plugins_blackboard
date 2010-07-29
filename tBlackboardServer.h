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

#ifndef PLUGINS__BLACKBOARD__TBLACKBOARDSERVER_H
#define PLUGINS__BLACKBOARD__TBLACKBOARDSERVER_H

#include "core/portdatabase/tDataType.h"
#include "core/port/rpc/tInterfaceServerPort.h"
#include "blackboard/tBlackboardBuffer.h"
#include "core/tFrameworkElement.h"
#include "core/port/rpc/tMethodCallException.h"
#include "blackboard/tAbstractBlackboardServer.h"

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard server
 */
class tBlackboardServer : public tAbstractBlackboardServer
{
private:

  /*! Unlock timeout in ms - if no keep-alive signal occurs in this period of time */
  static const int64 cUNLOCK_TIMEOUT = 1000;

  /*! Interface port for write access */
  core::tInterfaceServerPort* write;

  /*! Is blackboard currently locked? - in this case points to duplicated buffer */
  tBlackboardBuffer* locked;

  /*! Time when last lock was performed */
  volatile int64 lock_time;

  /*! Last time a keep-alive-signal was received */
  volatile int64 last_keep_alive;

  /*! ID of current lock - against outdated unlocks */
  util::tAtomicInt lock_iDGen;

  int lock_id;

  /*!
   * Currently published MemBuffer - not extra locked - attached to lock of read port
   * In single buffered mode - this is the one and only buffer
   */
  tBlackboardBuffer* published;

  //  @Override
  //  public void handleMethodCall(MethodCall mc, byte methodId, boolean deferred, long int1, long int2, long int3, double dbl1, double dbl2, double dbl3, TypedObject obj1, TypedObject obj2) {
  //
  //      if (mc.getMethodID() == RawBlackboardClient.IS_SINGLE_BUFFERED) {
  //          mc.setReturn(0);
  //          return;
  //      }
  //
  //      // deferred calls only come synchronized
  //      if (deferred) {
  //          assert(write.access.hasLock());
  //
  //          // apply asynch change optimization...
  //          if (locked != null && methodId == RawBlackboardClient.ASYNCH_CHANGE) { // still locked - before unlock
  //              assert(locked.getManager().isLocked());
  //              applyAsynchChange((int)int1, (BlackboardBuffer)obj1);
  //              return;
  //          }
  //      }
  //
  //      // non-exclusive-access calls
  //      int offset = 0, length = 0;
  //      @Const BlackboardBuffer buffer;
  //      BlackboardBuffer send = null;
  //      long arrival = 0;
  //
  //      switch(methodId) {
  //      case RawBlackboardClient.READ_PART:
  //          // current buffer
  //          buffer = (BlackboardBuffer)readPort.getLockedUnsafeRaw();
  //          assert(buffer.getManager().isLocked());
  //
  //          // call parameters
  //          offset = (int)int1;
  //          length = (int)int2;
  //
  //          // prepare and set return value
  //          send = (BlackboardBuffer)write.getUnusedBuffer(buffer.getType());
  //          send.getBuffer().put(0, buffer.getBuffer(), offset, length);
  //          send.bbCapacity = buffer.bbCapacity;
  //          send.elements = buffer.elements;
  //          send.elementSize = buffer.elementSize;
  //          mc.setReturn(send, false);
  //
  //          buffer.getManager().getCurrentRefCounter().releaseLock();
  //          return;
  //
  //      case RawBlackboardClient.DEPRECATED_DIRECT_BUFFER_ACCESS:
  //          System.out.println("warning: Deprecated unlocked blackboard access");
  //          mc.setReturn(published, true);
  //          return;
  //      }
  //
  //      // exclusive-access calls
  //      // from here, everything has to be synchronized
  //      System.out.println("Thread " + Thread.currentThread().toString() + ": Executing command " + methodString(methodId) + " for Thread " + mc.getThreadUid() + (deferred ? "(deferred)" : ""));
  //      if (!deferred) {
  //          assert(!write.access.hasLock());
  //          System.out.println("Thread " + Thread.currentThread().toString() + ": Acquiring Mutex " + methodString(methodId) + " (Thread " + mc.getThreadUid() + ")");
  //          write.access.lock();
  //          System.out.println("Thread " + Thread.currentThread().toString() + ": Acquired Mutex " + methodString(methodId) + " (Thread " + mc.getThreadUid() + ")");
  //      }
  //
  //      if (methodId == RawBlackboardClient.UNLOCK) { // should be executed before anything else
  //
  //          unlock(mc, (BlackboardBuffer)obj1, (int)int1, deferred);
  //
  //      } else if (deferred || write.handleDeferredCalls()) {
  //
  //          // defer call, because blackboard is currently locked?
  //          if (locked != null) {
  //              checkCurrentLock();
  //              if (locked != null) {
  //                  if (methodId == RawBlackboardClient.LOCK && int1 == 0) { // we do not need to enqueue lock commands with zero timeout
  //                      mc.setReturnNull();
  //                  } else {
  //                      write.deferCall(mc, true);
  //                  }
  //                  if (!deferred) {
  //                      write.access.release();
  //                  }
  //                  return;
  //              }
  //          }
  //
  //          switch(methodId) {
  //
  //          case RawBlackboardClient.LOCK:
  //              System.out.println("Thread " + Thread.currentThread().toString() + ": handleLock");
  //              assert(int2 == 0) : "Client must not attempt read lock on multi-buffered blackboard";
  //              arrival = mc.getArrivalTime();
  //              if (deferred && (arrival + int1 < Time.getCoarse())) { // already timed out... process next commands
  //                  System.out.println("skipping lock command, because it already has timed out");
  //                  mc.setReturnNull();
  //                  break;
  //              } else {
  //                  duplicateAndLock();
  //                  assert(locked != null && locked.getManager().isLocked());
  //                  mc.setReturn(locked, false);
  //              }
  //              break;
  //
  //          case RawBlackboardClient.ASYNCH_CHANGE:
  //              duplicateAndLock();
  //              applyAsynchChange((int)int1, (BlackboardBuffer)obj1);
  //              commitLocked();
  //              assert(locked == null);
  //              processPendingCommands(deferred);
  //              break;
  //
  //          case RawBlackboardClient.DIRECT_COMMIT:
  //              locked = (BlackboardBuffer)obj1;
  //              mc.dontRecycleParam1();
  //              assert(locked != null);
  //              commitLocked();
  //              assert(locked == null);
  //              processPendingCommands(deferred);
  //              break;
  //
  //          }
  //      } else {
  //
  //          // defer call, because there are still commands in queue
  //          write.deferCall(mc, true);
  //      }
  //
  //      if (!deferred) {
  //          System.out.println("Thread " + Thread.currentThread().toString() + ": preRelease " + methodString(methodId) /*+ " (Thread " + mc.getThreadUid() + ")"*/);
  //          write.access.release();
  //      }
  //  }
  //
  //  private void unlock(MethodCall mc, BlackboardBuffer newBuffer, int newLockID, boolean deferred) {
  //      //System.out.println("Thread " + Thread.currentThread().toString() + ": handleUnlock");
  //      if (locked == null) {
  //          System.out.println("somewhat fatal warning: Unlock without lock in blackboard");
  //      }
  //      if (newBuffer == null) { // read unlock
  //          System.out.println("blackboard unlock without providing buffer (legacy read unlock?)");
  //          releaseLockAndProcessPendingCommands(deferred);
  //          return;
  //      }
  //      assert(newBuffer.getManager().isLocked());
  //      assert(newBuffer.lockID == newLockID);
  //      if (newBuffer.lockID != lockID) {
  //          System.out.println("Skipping outdated unlock");
  //          processPendingCommands(deferred);
  //          return;
  //      }
  //      if (newBuffer != locked) {
  //          locked.getManager().getCurrentRefCounter().releaseLock();
  //          mc.dontRecycleParam1();
  //          //newBuffer.getManager().getCurrentRefCounter().addLock();
  //          locked = newBuffer;
  //          System.out.println("Thread " + Thread.currentThread().toString() + ": lock = " + locked.toString());
  //          assert(locked.getCurReference().isLocked());
  //      }
  //      releaseLockAndProcessPendingCommands(deferred);
  //  }
  //

  /*!
   * Check if lock timed out (only call in synchronized/exclusive access context)
   */
  void CheckCurrentLock(util::tLock& passed_lock);

  //
  //  /** process pending commands and commit currently locked buffer (if there is one) - (only call in synchronized context
  //   * access lock will be released)
  //   * \param deferred */
  //  private void releaseLockAndProcessPendingCommands(boolean deferred) {
  //      assert(write.access.hasLock());
  //      if (deferred) { // this method is already active below in call stack
  //          return;
  //      }
  //
  //      // apply any asynch change commands before committing
  //      MethodCall mc = write.deferred.peek();
  //      while(mc != null && mc.getMethodID() == RawBlackboardClient.ASYNCH_CHANGE) {
  //          write.handleDeferredCall();
  //          mc = write.deferred.peek();
  //      }
  //
  //      // unlock & commit
  //      commitLocked();
  //      assert(locked == null || locked.getCurReference().isLocked());
  //
  //      // process pending commands
  //      processPendingCommands(deferred);
  //  }
  //
  //  private void processPendingCommands(boolean deferred) {
  //      if (deferred) {
  //          return;
  //      }
  //      write.handleDeferredCalls();
  //  }
  //
  /*! Release lock and commit changed buffer (needs to be called in synchronized context) */
  void CommitLocked();

  /*! Duplicate and lock current buffer (needs to be called in synchronized context) */
  void DuplicateAndLock();

  /*!
   * This method exists due to current imperfectness in java->c++ converter
   *
   * \param p
   */
  inline void SetPublished(tBlackboardBuffer* p)
  {
    published = p;
  }

protected:

  virtual void AsynchChange(int offset, const tBlackboardBuffer* buf, bool check_lock);

  virtual void DirectCommit(tBlackboardBuffer* new_buffer);

  //
  //  /**
  //   * Apply asynchronous change to blackboard
  //   *
  //   * \param mc Method call buffer with change
  //   */
  //  private void applyAsynchChange(int offset, BlackboardBuffer buffer) {
  //      locked.getBuffer().put(offset, buffer.getBuffer(), 0, buffer.getSize());
  //  }

  virtual bool IsLocked()
  {
    return locked != NULL;
  }

  virtual bool IsSingleBuffered()
  {
    return false;
  }

  virtual void KeepAlive(int lock_id_);

  virtual const tBlackboardBuffer* ReadLock(int64 timeout)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Client must not attempt read lock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::eINVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual tBlackboardBuffer* ReadPart(int offset, int length, int timeout);

  virtual void ReadUnlock(int lock_id_)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Client must not attempt read unlock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::eINVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual tBlackboardBuffer* WriteLock(int64 timeout);

  virtual void WriteUnlock(tBlackboardBuffer* buf);

public:

  /*!
   * \param description Name/Uid of blackboard
   * @parent parent of BlackboardServer
   */
  tBlackboardServer(const util::tString& description, core::tFrameworkElement* parent = NULL);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param capacity Blackboard capacity (see BlackboardBuffer)
   * \param elements Number of element (see BlackboardBuffer)
   * \param elem_size Element size (see BlackboardBuffer)
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tBlackboardServer(const util::tString& description, core::tDataType* type, int capacity, int elements, int elem_size, core::tFrameworkElement* parent = NULL, bool shared = true);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param mc_type Type of method calls
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tBlackboardServer(const util::tString& description, core::tDataType* type, core::tFrameworkElement* parent = NULL, bool shared = true);

  virtual void LockCheck();

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TBLACKBOARDSERVER_H
