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
#include "blackboard/tBlackboardServer.h"
#include "blackboard/tBlackboardManager.h"
#include "core/port/std/tPort.h"
#include "core/port/tPortCreationInfo.h"
#include "core/port/tPortFlags.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "finroc_core_utils/tTime.h"

namespace finroc
{
namespace blackboard
{
const int64 tBlackboardServer::cUNLOCK_TIMEOUT;

tBlackboardServer::tBlackboardServer(const util::tString& description, core::tFrameworkElement* parent) :
    tAbstractBlackboardServer(description, true ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, tBlackboardBuffer::cBUFFER_TYPE->GetRelatedType(), this, true ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    locked(NULL),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    published(NULL)
{
  // this(description,BlackboardBuffer.BUFFER_TYPE,parent,true);
  this->read_port = new core::tPort<tBlackboardBuffer>(core::tPortCreationInfo("read", this, tBlackboardBuffer::cBUFFER_TYPE, core::tPortFlags::cOUTPUT_PORT | (true ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  CheckType(tBlackboardBuffer::cBUFFER_TYPE);
  this->write_port = write;
  SetPublished(static_cast<tBlackboardBuffer*>(this->read_port->GetDefaultBufferRaw()));
}

tBlackboardServer::tBlackboardServer(const util::tString& description, core::tDataType* type, int capacity, int elements, int elem_size, core::tFrameworkElement* parent, bool shared) :
    tAbstractBlackboardServer(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, type->GetRelatedType(), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    locked(NULL),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    published(NULL)
{
  // this(description,type,parent,shared);
  this->read_port = new core::tPort<tBlackboardBuffer>(core::tPortCreationInfo("read", this, type, core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  CheckType(type);
  this->write_port = write;
  SetPublished(static_cast<tBlackboardBuffer*>(this->read_port->GetDefaultBufferRaw()));
  published->Resize(capacity, elements, elem_size, false);
}

tBlackboardServer::tBlackboardServer(const util::tString& description, core::tDataType* type, core::tFrameworkElement* parent, bool shared) :
    tAbstractBlackboardServer(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, type->GetRelatedType(), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    locked(NULL),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    published(NULL)
{
  this->read_port = new core::tPort<tBlackboardBuffer>(core::tPortCreationInfo("read", this, type, core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  CheckType(type);
  this->write_port = write;
  SetPublished(static_cast<tBlackboardBuffer*>(this->read_port->GetDefaultBufferRaw()));
}

void tBlackboardServer::AsynchChange(int offset, const tBlackboardBuffer* buf, bool check_lock)
{
  {
    util::tLock lock2(this->bb_lock);
    if (check_lock && locked != NULL)
    {
      CheckCurrentLock(lock2);
      if (locked != NULL)    // ok, we don't get lock now... defer command to next unlock
      {
        DeferAsynchChangeCommand(offset, buf);
        return;
      }
    }

    assert(((!check_lock) || (!PendingTasks())));

    // duplicate current buffer
    assert((locked == NULL));
    DuplicateAndLock();

    // apply asynch change
    locked->GetBuffer()->Put(offset, *buf->GetBuffer(), 0u, buf->GetSize());
    buf->GetManager()->ReleaseLock();

    // commit changes
    CommitLocked();
    assert((locked == NULL));
    //processPendingCommands();
  }
}

void tBlackboardServer::CheckCurrentLock(util::tLock& passed_lock)
{
  if (locked != NULL && util::tTime::GetCoarse() > last_keep_alive + cUNLOCK_TIMEOUT)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Blackboard server: Lock timed out... unlocking");
    locked->GetCurReference()->GetRefCounter()->ReleaseLock();
    lock_id = lock_iDGen.IncrementAndGet();
    locked = NULL;
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, util::tStringBuilder("Thread "), util::tThread::CurrentThread()->ToString(), ": lock = null");
    bool p = ProcessPendingCommands(passed_lock);
    if ((!p) && (!IsLocked()))
    {
      ::finroc::blackboard::tAbstractBlackboardServer::ProcessPendingAsynchChangeTasks();
    }
  }
}

void tBlackboardServer::CommitLocked()
{
  assert((locked != NULL && locked->GetCurReference()->IsLocked()));

  // process any waiting asynch change commands
  ProcessPendingAsynchChangeTasks();

  // publish new buffer
  locked->lock_iD = -1;
  this->read_port->Publish(locked);
  locked->GetCurReference()->GetRefCounter()->ReleaseLock();
  published = locked;
  locked = NULL;
  lock_id = lock_iDGen.IncrementAndGet();
  //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = null");
}

void tBlackboardServer::DirectCommit(tBlackboardBuffer* new_buffer)
{
  if (new_buffer == NULL)
  {
    return;
  }

  {
    util::tLock lock2(this->bb_lock);
    if (locked != NULL)    // note: current lock is obsolete, since we have a completely new buffer
    {
      //lockID = lockIDGen.incrementAndGet(); // make sure, next unlock won't do anything => done in commitLocked()
      locked->GetManager()->ReleaseLock();  // discard current lock
      locked = NULL;
    }

    // Clear any asynch change commands from queue, since they were for old buffer
    ClearAsyncChangeTasks();

    // commit new buffer
    locked = new_buffer;
    CommitLocked();
    assert((locked == NULL));

    // any threads that want to lock this?
    ProcessPendingCommands(lock2);
  }
}

void tBlackboardServer::DuplicateAndLock()
{
  assert((locked == NULL));
  lock_id = lock_iDGen.IncrementAndGet();
  lock_time = util::tTime::GetCoarse();
  last_keep_alive = lock_time;
  locked = static_cast<tBlackboardBuffer*>(this->read_port->GetUnusedBufferRaw());
  //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = " + locked.toString());
  locked->GetManager()->GetCurrentRefCounter()->SetOrAddLock();
  CopyBlackboardBuffer(published, locked);
  //locked.
  locked->lock_iD = lock_id;
}

void tBlackboardServer::KeepAlive(int lock_id_)
{
  {
    util::tLock lock2(this->bb_lock);
    if (locked != NULL && this->lock_id == lock_id_)
    {
      last_keep_alive = util::tTime::GetCoarse();
    }
  }
}

void tBlackboardServer::LockCheck()
{
  int64 cur_time = util::tTime::GetCoarse();
  if (last_keep_alive + cUNLOCK_TIMEOUT > cur_time)
  {
    return;
  }

  {
    util::tLock lock2(this->bb_lock);
    CheckCurrentLock(lock2);
  }
}

tBlackboardBuffer* tBlackboardServer::ReadPart(int offset, int length, int timeout)
{
  // current buffer (note: we get it from readPort, since -this way- call does not need to be executed in synchronized context)
  const tBlackboardBuffer* buffer = static_cast<const tBlackboardBuffer*>(this->read_port->GetLockedUnsafeRaw());
  assert((buffer->GetManager()->IsLocked()));

  // prepare and set return value
  tBlackboardBuffer* send = static_cast<tBlackboardBuffer*>(write->GetUnusedBuffer(buffer->GetType()));
  send->Resize(1, 1, length, false);  // ensure minimal size
  send->GetBuffer()->Put(0u, *buffer->GetBuffer(), offset, length);
  send->bb_capacity = buffer->bb_capacity;
  send->elements = buffer->elements;
  send->element_size = buffer->element_size;

  // release old lock
  buffer->GetManager()->GetCurrentRefCounter()->ReleaseLock();

  // return buffer with one read lock
  send->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
  return send;
}

tBlackboardBuffer* tBlackboardServer::WriteLock(int64 timeout)
{
  {
    util::tLock lock2(this->bb_lock);
    if (locked != NULL || PendingTasks())    // make sure lock command doesn't "overtake" others
    {
      CheckCurrentLock(lock2);
      if (locked != NULL || PendingTasks())
      {
        if (timeout <= 0)
        {
          return NULL;  // we do not need to enqueue lock commands with zero timeout
        }
        else
        {
          // wait for lock
          bool have_lock = WaitForLock(lock2, timeout);
          if (!have_lock)
          {
            return NULL;  // we didn't get lock :-/
          }
        }
      }

      // ok... we have lock here
      assert((locked == NULL));
    }

    //System.out.println("Thread " + Thread.currentThread().toString() + ": handleLock");

    DuplicateAndLock();
    assert((locked != NULL && locked->GetManager()->IsLocked()));
    locked->GetManager()->AddLock();
    return locked;  // return buffer with one read lock
    //mc.setReturn(locked, false);
  }
}

void tBlackboardServer::WriteUnlock(tBlackboardBuffer* buf)
{
  if (buf == NULL)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "blackboard write unlock without providing buffer - strange indeed - ignoring");
    return;
  }

  {
    util::tLock lock2(this->bb_lock);
    if (this->lock_id != buf->lock_iD)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Skipping outdated unlock");
      buf->GetManager()->ReleaseLock();
      return;
    }

    assert((locked != NULL));
    assert((buf->GetManager()->IsLocked()));

    if (buf == locked)
    {
      // we got the same buffer back - we only need to release one lock from method call
      buf->GetManager()->GetCurrentRefCounter()->ReleaseLock();
      assert((locked->GetCurReference()->IsLocked()));
    }
    else
    {
      locked->GetManager()->GetCurrentRefCounter()->ReleaseLock();
      locked = buf;
      //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = " + locked.toString());
      assert((locked->GetCurReference()->IsLocked()));
    }

    CommitLocked();
    ProcessPendingCommands(lock2);
  }
}

} // namespace finroc
} // namespace blackboard

