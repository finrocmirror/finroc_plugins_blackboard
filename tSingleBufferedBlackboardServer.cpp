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
#include "blackboard/tSingleBufferedBlackboardServer.h"
#include "blackboard/tBlackboardManager.h"
#include "core/port/tPortFlags.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "finroc_core_utils/tTime.h"
#include "core/port/rpc/tMethodCallException.h"

namespace finroc
{
namespace blackboard
{
const int64 tSingleBufferedBlackboardServer::cUNLOCK_TIMEOUT;

tSingleBufferedBlackboardServer::tSingleBufferedBlackboardServer(const util::tString& description, core::tFrameworkElement* parent) :
    tAbstractBlackboardServer(description, true ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, tBlackboardBuffer::cBUFFER_TYPE->GetRelatedType(), this, true ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    buffer(static_cast<tBlackboardBuffer*>(write->GetUnusedBuffer(tBlackboardBuffer::cBUFFER_TYPE))),
    locks(0),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    revision(0),
    read_copy(NULL),
    read_copy_revision(-1),
    thread_waiting_for_copy(false)
{
  // this(description,BlackboardBuffer.BUFFER_TYPE,parent,true);
  this->read_port = new tBBReadPort(this, core::tPortCreationInfo("read", this, tBlackboardBuffer::cBUFFER_TYPE, core::tPortFlags::cOUTPUT_PORT | (true ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  this->read_port->SetPullRequestHandler(this);
  CheckType(tBlackboardBuffer::cBUFFER_TYPE);
  this->write_port = write;
  buffer->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
}

tSingleBufferedBlackboardServer::tSingleBufferedBlackboardServer(const util::tString& description, core::tDataType* type, int capacity, int elements, int elem_size, core::tFrameworkElement* parent, bool shared) :
    tAbstractBlackboardServer(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, type->GetRelatedType(), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    buffer(static_cast<tBlackboardBuffer*>(write->GetUnusedBuffer(type))),
    locks(0),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    revision(0),
    read_copy(NULL),
    read_copy_revision(-1),
    thread_waiting_for_copy(false)
{
  // this(description,type,parent,shared);
  this->read_port = new tBBReadPort(this, core::tPortCreationInfo("read", this, type, core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  this->read_port->SetPullRequestHandler(this);
  CheckType(type);
  this->write_port = write;
  buffer->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
  buffer->Resize(capacity, elements, elem_size, false);
}

tSingleBufferedBlackboardServer::tSingleBufferedBlackboardServer(const util::tString& description, core::tDataType* type, core::tFrameworkElement* parent, bool shared) :
    tAbstractBlackboardServer(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, type->GetRelatedType(), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    buffer(static_cast<tBlackboardBuffer*>(write->GetUnusedBuffer(type))),
    locks(0),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    revision(0),
    read_copy(NULL),
    read_copy_revision(-1),
    thread_waiting_for_copy(false)
{
  this->read_port = new tBBReadPort(this, core::tPortCreationInfo("read", this, type, core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  this->read_port->SetPullRequestHandler(this);
  CheckType(type);
  this->write_port = write;
  buffer->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
}

void tSingleBufferedBlackboardServer::AsynchChange(int offset, const tBlackboardBuffer* buf, bool check_lock)
{
  {
    util::tLock lock2(this->bb_lock);
    if (check_lock && IsLocked())
    {
      CheckCurrentLock(lock2);
      if (IsLocked())
      {
        DeferAsynchChangeCommand(offset, buf);
        return;
      }
    }

    assert(((!check_lock) || (!IsLocked())));

    buffer->GetBuffer()->Put(offset, *buf->GetBuffer(), 0u, buf->GetSize());
    buf->GetManager()->ReleaseLock();

    // commit changes
    NewBufferRevision(lock2, true);

    assert(((!check_lock) || (!IsLocked())));
  }
}

void tSingleBufferedBlackboardServer::CheckCurrentLock(util::tLock& passed_lock)
{
  if (IsLocked() && util::tTime::GetCoarse() > last_keep_alive + cUNLOCK_TIMEOUT)
  {
    util::tSystem::out.Println("Blackboard server: Lock timed out... unlocking");

    // meh... we have a read or write lock... so a client may make changes to it... or may still read it... it's safer to create new buffer here
    tBlackboardBuffer* new_buffer = static_cast<tBlackboardBuffer*>(this->read_port->GetUnusedBufferRaw());
    new_buffer->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));;

    CopyBlackboardBuffer(buffer, new_buffer);
    buffer->GetCurReference()->GetRefCounter()->ReleaseLock();
    buffer = new_buffer;

    NewBufferRevision(passed_lock, true);
    locks = 0;

    bool p = ProcessPendingCommands(passed_lock);
    if ((!p) && (!IsLocked()))
    {
      ::finroc::blackboard::tAbstractBlackboardServer::ProcessPendingAsynchChangeTasks();
    }
  }
}

tSingleBufferedBlackboardServer::~tSingleBufferedBlackboardServer()
{
  if (read_copy != NULL)
  {
    read_copy->GetManager()->ReleaseLock();
    read_copy = NULL;
  }
  assert((buffer != NULL));
  buffer->GetManager()->ReleaseLock();
  buffer = NULL;
}

void tSingleBufferedBlackboardServer::DirectCommit(tBlackboardBuffer* new_buffer)
{
  if (new_buffer == NULL)
  {
    return;
  }

  {
    util::tLock lock2(this->bb_lock);

    assert((new_buffer != buffer));

    // note: current lock is obsolete, since we have a completely new buffer
    buffer->GetManager()->GetCurrentRefCounter()->ReleaseLock();
    buffer = new_buffer;

    // Clear any asynch change commands from queue, since they were for old buffer
    ClearAsyncChangeTasks();

    lock_id = lock_iDGen.IncrementAndGet();
    assert((buffer->GetManager()->IsLocked()));
    locks = 0;
    ProcessPendingCommands(lock2);
  }
}

void tSingleBufferedBlackboardServer::GetSizeInfo(size_t& element_size, size_t& elements, size_t& capacity)
{
  // ok... three cases... 1) up to date copy  2) no lock  3) lock

  // case 1: get buffer from superclass
  if (read_copy_revision == revision)
  {
    const tBlackboardBuffer* bb = static_cast<const tBlackboardBuffer*>(this->read_port->GetLockedUnsafeRaw());
    element_size = bb->GetElementSize();
    elements = bb->GetElements();
    capacity = bb->GetBbCapacity();
    bb->GetManager()->ReleaseLock();
    return;
  }

  // case 2/3: okay... wait until blackboard has no lock (could be implemented more sophisticated, but that shouldn't matter here...)
  while (true)
  {
    {
      util::tLock lock3(this->bb_lock);
      if (locks >= 0)    // ok, not locked or read locked
      {
        element_size = buffer->GetElementSize();
        elements = buffer->GetElements();
        capacity = buffer->GetBbCapacity();
        return;
      }
      try
      {
        util::tThread::Sleep(50);
      }
      catch (const util::tInterruptedException& e)
      {
      }
    }
  }

}

void tSingleBufferedBlackboardServer::KeepAlive(int lock_id_)
{
  {
    util::tLock lock2(this->bb_lock);
    if (locks != 0 && this->lock_id == lock_id_)
    {
      last_keep_alive = util::tTime::GetCoarse();
    }
  }
}

void tSingleBufferedBlackboardServer::LockCheck()
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

void tSingleBufferedBlackboardServer::NewBufferRevision(util::tLock& passed_lock, bool has_changes)
{
  lock_id = lock_iDGen.IncrementAndGet();
  if (has_changes)
  {
    revision++;
  }

  // process any waiting asynch change commands
  ProcessPendingAsynchChangeTasks();
  if (thread_waiting_for_copy || this->read_port->GetStrategy() > 0)
  {
    UpdateReadCopy(passed_lock);
  }
}

const core::tPortData* tSingleBufferedBlackboardServer::PullRequest(core::tPortBase* origin, int8 add_locks)
{
  {
    util::tLock lock2(this->bb_lock);

    // possibly wait for a copy
    while (read_copy_revision < revision)     // not so clean, but everything else becomes rather complicated
    {

      if (IsLocked())
      {
        WaitForReadCopy(lock2, revision, 2000);
      }
      else
      {
        UpdateReadCopy(lock2);
      }
    }

    // add desired number of locks and return
    read_copy->GetManager()->GetCurrentRefCounter()->AddLocks(add_locks);
    return read_copy;
  }
}

const tBlackboardBuffer* tSingleBufferedBlackboardServer::ReadLock(int64 timeout)
{
  {
    util::tLock lock2(this->bb_lock);

    // Read Lock
    int64 current_revision = revision;
    if (locks < 0 && current_revision != read_copy_revision)
    {
      CheckCurrentLock(lock2);
      if (locks < 0 && current_revision != read_copy_revision)
      {
        if (timeout <= 0)
        {
          return NULL;  // we do not need to enqueue lock commands with zero timeout
        }
        WaitForReadCopy(lock2, current_revision, timeout);
        assert((read_copy_revision >= current_revision));
      }
    }

    if (read_copy_revision >= current_revision)
    {
      // there's a copy... use this
      read_copy->GetManager()->AddLock();
      return read_copy;
    }

    if (locks >= 0)
    {
      // okay, we either have no lock or a read lock
      if (PendingTasks() || thread_waiting_for_copy)    // there are others waiting... make copy
      {
        UpdateReadCopy(lock2);
        assert((read_copy_revision >= current_revision));
        read_copy->GetManager()->AddLock();
        return read_copy;
      }
      else    // no one waiting... simply lock buffer
      {
        if (locks == 0)    // if this is the first lock: increment and set lock id of buffer
        {
          int lock_iDNew = lock_iDGen.IncrementAndGet();
          lock_id = lock_iDNew;
          buffer->lock_iD = lock_iDNew;
        }
        locks++;
        buffer->GetManager()->AddLock();
        return buffer;
      }
    }

    throw core::tMethodCallException(core::tMethodCallException::ePROGRAMMING_ERROR);
  }
}

tBlackboardBuffer* tSingleBufferedBlackboardServer::ReadPart(int offset, int length, int timeout)
{
  {
    util::tLock lock2(this->bb_lock);
    const tBlackboardBuffer* bb = buffer;
    bool unlock = false;
    int64 current_revision = revision;
    if (locks < 0 && current_revision != read_copy_revision)
    {
      CheckCurrentLock(lock2);
      if (locks < 0 && current_revision != read_copy_revision)
      {
        if (timeout <= 0)
        {
          return NULL;  // we do not need to enqueue lock commands with zero timeout
        }

        // okay... we'll do a read lock
        bb = ReadLock(timeout);
        if (bb == NULL)
        {
          return NULL;
        }
        unlock = true;
      }
    }

    if ((!unlock) && current_revision == read_copy_revision)    // can we use read copy?
    {
      bb = read_copy;
    }

    // prepare and set return value
    tBlackboardBuffer* send = static_cast<tBlackboardBuffer*>(write->GetUnusedBuffer(buffer->GetType()));
    send->Resize(1, 1, length, false);  // ensure minimal size
    send->GetBuffer()->Put(0u, *bb->GetBuffer(), offset, length);
    send->bb_capacity = buffer->bb_capacity;
    send->elements = buffer->elements;
    send->element_size = buffer->element_size;

    if (unlock)    // if we have a read lock, we need to release it
    {
      ReadUnlock(lock_id);
    }

    // return buffer with one read lock
    send->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));
    return send;
  }
}

void tSingleBufferedBlackboardServer::ReadUnlock(int lock_id_)
{
  if (lock_id_ < 0)
  {
    return;  // not interested, since it's a copy
  }

  {
    util::tLock lock2(this->bb_lock);
    if (this->lock_id != lock_id_)
    {
      util::tSystem::out.Println("Skipping outdated unlock");
      return;
    }

    // okay, this is unlock for the current lock
    assert((locks > 0));
    locks--;
    if (locks == 0)
    {
      NewBufferRevision(lock2, false);
      ProcessPendingCommands(lock2);
    }
    return;
  }
}

void tSingleBufferedBlackboardServer::UpdateReadCopy(util::tLock& passed_lock)
{
  assert((buffer->GetManager()->IsLocked()));

  if (read_copy_revision < revision)
  {
    // release lock of old read buffer
    if (read_copy != NULL)
    {
      read_copy->GetManager()->ReleaseLock();
    }

    // copy current buffer
    read_copy = static_cast<tBlackboardBuffer*>(this->read_port->GetUnusedBufferRaw());
    read_copy->GetManager()->GetCurrentRefCounter()->SetLocks(static_cast<int8>(1));;

    CopyBlackboardBuffer(buffer, read_copy);
    read_copy->lock_iD = -1;
    this->read_port->Publish(read_copy);

    read_copy_revision = revision;

    // publish read copy
    this->read_port->Publish(read_copy);
  }

  // notify all threads waiting for a buffer copy
  if (thread_waiting_for_copy)
  {
    thread_waiting_for_copy = false;
    this->wakeup_thread = -1;
    this->bb_lock.monitor.NotifyAll(passed_lock);
  }

}

void tSingleBufferedBlackboardServer::WaitForReadCopy(util::tLock& passed_lock, int64 min_revision, int64 timeout)
{
  int64 cur_time = util::tTime::GetCoarse();
  while (read_copy_revision < min_revision)
  {
    int64 wait_for = timeout - (util::tTime::GetCoarse() - cur_time);
    if (wait_for > 0)
    {
      thread_waiting_for_copy = true;
      try
      {
        this->bb_lock.monitor.Wait(passed_lock, wait_for);
      }
      catch (const util::tInterruptedException& e)
      {
        util::tSystem::out.Println("SingleBufferedBlackboardServer: Interrupted while waiting for read copy - strange");
        //e.printStackTrace();
      }
    }
  }
}

tBlackboardBuffer* tSingleBufferedBlackboardServer::WriteLock(int64 timeout)
{
  {
    util::tLock lock2(this->bb_lock);
    if (IsLocked() || PendingTasks())
    {
      CheckCurrentLock(lock2);
      if (IsLocked() || PendingTasks())
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
    }

    assert((!IsLocked()));

    // lock current buffer... and return it with a lock
    int lock_iDNew = lock_iDGen.IncrementAndGet();
    lock_id = lock_iDNew;
    buffer->lock_iD = lock_iDNew;
    locks = -1;
    lock_time = util::tTime::GetCoarse();
    last_keep_alive = lock_time;
    buffer->GetManager()->AddLock();
    return buffer;
  }
}

void tSingleBufferedBlackboardServer::WriteUnlock(tBlackboardBuffer* buf)
{
  if (buf == NULL)
  {
    util::tSystem::out.Println("blackboard write unlock without providing buffer - you shouldn't do that - ignoring");
    return;
  }
  assert(((buf->lock_iD >= 0)) && "lock IDs < 0 are typically only found in read copies");

  {
    util::tLock lock2(this->bb_lock);
    if (this->lock_id != buf->lock_iD)
    {
      util::tSystem::out.Println("Skipping outdated unlock");
      buf->GetManager()->ReleaseLock();
      return;
    }

    assert((locks < 0));  // write lock
    assert((buf->GetManager()->IsLocked()));

    lock_id = lock_iDGen.IncrementAndGet();
    if (buf == buffer)
    {
      // we got the same buffer back - we only need to release one lock from method call
      buf->GetManager()->GetCurrentRefCounter()->ReleaseLock();
    }
    else
    {
      buffer->GetManager()->GetCurrentRefCounter()->ReleaseLock();
      buffer = buf;
      //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = " + buffer.toString());
      assert((buffer->GetCurReference()->IsLocked()));
    }
    locks = 0;
    NewBufferRevision(lock2, true);

    ProcessPendingCommands(lock2);
  }
}

tSingleBufferedBlackboardServer::tBBReadPort::tBBReadPort(tSingleBufferedBlackboardServer* const outer_class_ptr_, core::tPortCreationInfo pci) :
    core::tPort<tBlackboardBuffer>(pci),
    outer_class_ptr(outer_class_ptr_)
{
}

void tSingleBufferedBlackboardServer::tBBReadPort::InitialPushTo(core::tAbstractPort* target, bool reverse)
{
  assert(((!reverse)) && "?!");

  // ok... three cases... 1) up to date copy  2) no lock  3) lock

  // case 1: let super class handle this
  if (outer_class_ptr->read_copy_revision == outer_class_ptr->revision)
  {
    ::finroc::core::tPortBase::InitialPushTo(target, reverse);
    return;
  }

  // case 3: publish will happen anyway - since strategy is > 0

  // case 2: make read copy
  {
    util::tLock lock2(outer_class_ptr->bb_lock);
    if (outer_class_ptr->locks >= 0)    // ok, not locked or read locked
    {
      outer_class_ptr->locks++;
      outer_class_ptr->UpdateReadCopy(lock2);
      outer_class_ptr->locks--;
    }
  }
}

} // namespace finroc
} // namespace blackboard

