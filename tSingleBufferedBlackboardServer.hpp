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
#include "plugins/blackboard/tBlackboardManager.h"
#include "core/port/tPortFlags.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"
#include "core/port/rpc/tInterfaceServerPort.h"
#include "rrlib/finroc_core_utils/tTime.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "core/port/std/tPortDataManager.h"
#include "core/port/rpc/tMethodCallException.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
const int64 tSingleBufferedBlackboardServer<T>::cUNLOCK_TIMEOUT;

template<typename T>
tSingleBufferedBlackboardServer<T>::tSingleBufferedBlackboardServer(const util::tString& description, int capacity, int elements, int elem_size, core::tFrameworkElement* parent, bool shared, rrlib::serialization::tDataTypeBase type) :
    tAbstractBlackboardServer<T>(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, this->GetBlackboardMethodType(type), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    buffer(write->GetBufferForReturn<tBBVector>()),
    locks(0),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    revision(0),
    read_copy(),
    read_copy_revision(-1),
    thread_waiting_for_copy(false)
{
  // this(description,elements,parent,shared,type);
  this->read_port_raw = new tBBReadPort(this, core::tPortCreationInfo("read", this, type.GetListType(), core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  this->read_port_raw->SetPullRequestHandler(this);
  ::finroc::blackboard::tAbstractBlackboardServerRaw::CheckType(type);
  this->write_port_raw = write;

  Resize(*buffer, elements, elements);

  tBlackboardManager::GetInstance()->Init();
  ClassicBlackboardResize(&((*buffer)[0]), capacity, elements, elem_size);
}

template<typename T>
tSingleBufferedBlackboardServer<T>::tSingleBufferedBlackboardServer(const util::tString& description, int elements, core::tFrameworkElement* parent, bool shared, rrlib::serialization::tDataTypeBase type) :
    tAbstractBlackboardServer<T>(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, this->GetBlackboardMethodType(type), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    buffer(write->GetBufferForReturn<tBBVector>()),
    locks(0),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    revision(0),
    read_copy(),
    read_copy_revision(-1),
    thread_waiting_for_copy(false)
{
  this->read_port_raw = new tBBReadPort(this, core::tPortCreationInfo("read", this, type.GetListType(), core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1));
  this->read_port_raw->SetPullRequestHandler(this);
  ::finroc::blackboard::tAbstractBlackboardServerRaw::CheckType(type);
  this->write_port_raw = write;

  Resize(*buffer, elements, elements);

  tBlackboardManager::GetInstance()->Init();
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::AsynchChange(tConstChangeTransactionVar& buf, int index, int offset, bool check_lock)
{
  {
    util::tLock lock2(this->bb_lock);
    if (check_lock && IsLocked())
    {
      CheckCurrentLock(lock2);
      if (IsLocked())
      {
        DeferAsynchChangeCommand(buf, index, offset);
        return;
      }
    }

    assert(((!check_lock) || (!IsLocked())));

    this->ApplyAsynchChange(*buffer, buf, index, offset);

    //buffer.getBuffer().put(offset, buf.getBuffer(), 0, buf.getSize());

    // commit changes
    NewBufferRevision(lock2, true);

    assert(((!check_lock) || (!IsLocked())));
  }
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::CheckCurrentLock(util::tLock& passed_lock)
{
  if (IsLocked() && util::tTime::GetCoarse() > last_keep_alive + cUNLOCK_TIMEOUT)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Blackboard server: Lock timed out... unlocking");

    // meh... we have a read or write lock... so a client may make changes to it... or may still read it... it's safer to create new buffer here
    tBBVectorVar new_buffer = write->GetBufferForReturn<tBBVector>();

    this->CopyBlackboardBuffer(*buffer, *new_buffer);
    buffer = std::move(new_buffer);

    NewBufferRevision(passed_lock, true);
    locks = 0;

    bool p = this->ProcessPendingCommands(passed_lock);
    if ((!p) && (!IsLocked()))
    {
      this->ProcessPendingAsynchChangeTasks();
    }
  }
}

template<typename T>
tSingleBufferedBlackboardServer<T>::~tSingleBufferedBlackboardServer()
{
  assert((buffer != NULL));

}

template<typename T>
void tSingleBufferedBlackboardServer<T>::DirectCommit(tBBVectorVar& new_buffer)
{
  if (new_buffer == NULL)
  {
    return;
  }

  {
    util::tLock lock2(this->bb_lock);

    // note: current lock is obsolete, since we have a completely new buffer
    assert((&(new_buffer) != &(buffer)));

    buffer = std::move(new_buffer);

    // Clear any asynch change commands from queue, since they were for old buffer
    this->ClearAsyncChangeTasks();

    lock_id = lock_iDGen.IncrementAndGet();
    assert((GetManager(buffer)->IsLocked()));
    locks = 0;
    this->ProcessPendingCommands(lock2);
  }
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::KeepAlive(int lock_id_)
{
  {
    util::tLock lock2(this->bb_lock);
    if (locks != 0 && this->lock_id == lock_id_)
    {
      last_keep_alive = util::tTime::GetCoarse();
    }
  }
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::LockCheck()
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

template<typename T>
void tSingleBufferedBlackboardServer<T>::NewBufferRevision(util::tLock& passed_lock, bool has_changes)
{
  lock_id = lock_iDGen.IncrementAndGet();
  if (has_changes)
  {
    revision++;
  }

  // process any waiting asynch change commands
  this->ProcessPendingAsynchChangeTasks();
  if (thread_waiting_for_copy || this->read_port_raw->GetStrategy() > 0)
  {
    UpdateReadCopy(passed_lock);
  }
}

template<typename T>
const core::tPortDataManager* tSingleBufferedBlackboardServer<T>::PullRequest(core::tPortBase* origin, int8 add_locks)
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
    core::tPortDataManager* mgr = GetManager(read_copy);
    mgr->GetCurrentRefCounter()->AddLocks(add_locks);

    return mgr;
  }
}

template<typename T>
typename tAbstractBlackboardServer<T>::tConstBBVectorVar tSingleBufferedBlackboardServer<T>::ReadLock(int64 timeout)
{
  {
    util::tLock lock2(this->bb_lock);
    return ReadLockImpl(lock2, timeout);
  }
}

template<typename T>
typename tAbstractBlackboardServer<T>::tConstBBVectorVar tSingleBufferedBlackboardServer<T>::ReadLockImpl(util::tLock& passed_lock, int64 timeout)
{
  // Read Lock
  int64 current_revision = revision;
  if (locks < 0 && current_revision != read_copy_revision)
  {
    CheckCurrentLock(passed_lock);
    if (locks < 0 && current_revision != read_copy_revision)
    {
      if (timeout <= 0)    // we do not need to enqueue lock commands with zero timeout
      {

        return tConstBBVectorVar(); // we do not need to enqueue lock commands with zero timeout
      }
      WaitForReadCopy(passed_lock, current_revision, timeout);
      assert((read_copy_revision >= current_revision));
    }
  }

  if (read_copy_revision >= current_revision)
  {
    // there's a copy... use this
    GetManager(read_copy)->AddLock();

    return GetManager(read_copy);
  }

  if (locks >= 0)
  {
    // okay, we either have no lock or a read lock
    if (this->PendingTasks() || thread_waiting_for_copy)    // there are others waiting... make copy
    {
      UpdateReadCopy(passed_lock);
      assert((read_copy_revision >= current_revision));
      GetManager(read_copy)->AddLock();

      return GetManager(read_copy);
    }
    else    // no one waiting... simply lock buffer
    {
      if (locks == 0)    // if this is the first lock: increment and set lock id of buffer
      {
        int lock_iDNew = lock_iDGen.IncrementAndGet();
        lock_id = lock_iDNew;
        GetManager(buffer)->lock_iD = lock_iDNew;
      }
      locks++;
      GetManager(buffer)->AddLock();

      return GetManager(buffer);
    }
  }

  throw core::tMethodCallException(core::tMethodCallException::ePROGRAMMING_ERROR, CODE_LOCATION_MACRO);
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::ReadUnlock(int lock_id_)
{
  if (lock_id_ < 0)
  {
    return;  // not interested, since it's a copy
  }

  {
    util::tLock lock2(this->bb_lock);
    ReadUnlockImpl(lock2, lock_id_);
  }
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::ReadUnlockImpl(util::tLock& passed_lock, int lock_id_)
{
  if (lock_id_ < 0)
  {
    return;  // not interested, since it's a copy
  }

  if (this->lock_id != lock_id_)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Skipping outdated unlock");
    return;
  }

  // okay, this is unlock for the current lock
  assert((locks > 0));
  locks--;
  if (locks == 0)
  {
    NewBufferRevision(passed_lock, false);
    this->ProcessPendingCommands(passed_lock);
  }
  return;
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::UpdateReadCopy(util::tLock& passed_lock)
{
  assert((GetManager(buffer)->IsLocked()));

  if (read_copy_revision < revision)
  {
    // copy current buffer

    // copy current buffer

    read_copy = write->GetBufferForReturn<tBBVector>();
    this->CopyBlackboardBuffer(*buffer, *read_copy);

    core::tPortDataManager* copymgr = GetManager(read_copy);
    copymgr->lock_iD = -1;
    this->read_port_raw->Publish(copymgr);

    read_copy_revision = revision;

    // publish read copy
    this->read_port_raw->Publish(copymgr);
  }

  // notify all threads waiting for a buffer copy
  if (thread_waiting_for_copy)
  {
    thread_waiting_for_copy = false;
    this->wakeup_thread = -1;
    this->bb_lock.monitor.NotifyAll(passed_lock);
  }

}

template<typename T>
void tSingleBufferedBlackboardServer<T>::WaitForReadCopy(util::tLock& passed_lock, int64 min_revision, int64 timeout)
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
        FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "SingleBufferedBlackboardServer: Interrupted while waiting for read copy - strange");
        //e.printStackTrace();
      }
    }
  }
}

template<typename T>
typename tAbstractBlackboardServer<T>::tBBVectorVar tSingleBufferedBlackboardServer<T>::WriteLock(int64 timeout)
{
  {
    util::tLock lock2(this->bb_lock);
    if (IsLocked() || this->PendingTasks())
    {
      CheckCurrentLock(lock2);
      if (IsLocked() || this->PendingTasks())
      {
        if (timeout <= 0)    // we do not need to enqueue lock commands with zero timeout
        {

          return tBBVectorVar(); // we do not need to enqueue lock commands with zero timeout
        }
        else
        {
          // wait for lock
          bool have_lock = this->WaitForLock(lock2, timeout);
          if (!have_lock)    // we didn't get lock :-/
          {

            return tBBVectorVar(); // we didn't get lock :-/
          }
        }
      }
    }

    assert((!IsLocked()));

    // lock current buffer... and return it with a lock
    int lock_iDNew = lock_iDGen.IncrementAndGet();
    lock_id = lock_iDNew;
    GetManager(buffer)->lock_iD = lock_iDNew;
    locks = -1;
    lock_time = util::tTime::GetCoarse();
    last_keep_alive = lock_time;

    GetManager(buffer)->AddLock();

    return GetManager(buffer);
  }
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::WriteUnlock(tBBVectorVar& buf)
{
  if (buf == NULL)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "blackboard write unlock without providing buffer - you shouldn't do that - ignoring");
    return;
  }
  core::tPortDataManager* bufmgr = GetManager(buf);
  assert(((bufmgr->lock_iD >= 0)) && "lock IDs < 0 are typically only found in read copies");

  {
    util::tLock lock2(this->bb_lock);
    if (this->lock_id != bufmgr->lock_iD)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Skipping outdated unlock");

      return;
    }

    assert((locks < 0));  // write lock
    assert((bufmgr->IsLocked()));

    lock_id = lock_iDGen.IncrementAndGet();

    if (buf != buffer)
    {
      buffer = std::move(buf);
      assert(GetManager(buffer)->IsLocked());
    }
    else
    {
      buf.reset();
    }

    locks = 0;
    NewBufferRevision(lock2, true);

    this->ProcessPendingCommands(lock2);
  }
}

template<typename T>
tSingleBufferedBlackboardServer<T>::tBBReadPort::tBBReadPort(tSingleBufferedBlackboardServer* const outer_class_ptr_, core::tPortCreationInfo pci) :
    core::tPortBase(pci),
    outer_class_ptr(outer_class_ptr_)
{
}

template<typename T>
void tSingleBufferedBlackboardServer<T>::tBBReadPort::InitialPushTo(core::tAbstractPort* target, bool reverse)
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

