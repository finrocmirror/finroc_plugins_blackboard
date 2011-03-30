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
#include "core/port/tPortCreationInfo.h"
#include "core/port/tPortFlags.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "core/port/std/tPortBase.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"
#include "core/port/rpc/tInterfaceServerPort.h"
#include "rrlib/finroc_core_utils/tTime.h"
#include "core/port/std/tPortDataManager.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
const int64 tBlackboardServer<T>::cUNLOCK_TIMEOUT;

template<typename T>
tBlackboardServer<T>::tBlackboardServer(const util::tString& description, int capacity, int elements, int elem_size, core::tFrameworkElement* parent, bool shared, rrlib::serialization::tDataTypeBase type) :
    tAbstractBlackboardServer<T>(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, type.GetRelatedType(), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    locked(),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    published(),
    read_port()
{
  // this(description,elements,parent,shared,type);
  core::tPortCreationInfo read_pci = core::tPortCreationInfo("read", this, this->GetBlackboardMethodType(type), core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1);

  read_port.reset(new core::tPort<tBBVector>(read_pci));

  this->read_port_raw = static_cast<core::tPortBase*>(read_port->GetWrapped());
  ::finroc::blackboard::tAbstractBlackboardServerRaw::CheckType(type);
  this->write_port_raw = write;
  SetPublished(read_port->GetDefaultBuffer());

  Resize(*published, elements, elements);

  tBlackboardManager::GetInstance()->Init();
}

template<typename T>
tBlackboardServer<T>::tBlackboardServer(const util::tString& description, int elements, core::tFrameworkElement* parent, bool shared, rrlib::serialization::tDataTypeBase type) :
    tAbstractBlackboardServer<T>(description, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
    write(new core::tInterfaceServerPort("write", this, type.GetRelatedType(), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
    locked(),
    lock_time(0),
    last_keep_alive(0),
    lock_iDGen(0),
    lock_id(0),
    published(),
    read_port()
{
  core::tPortCreationInfo read_pci = core::tPortCreationInfo("read", this, this->GetBlackboardMethodType(type), core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0)).LockOrderDerive(core::tLockOrderLevels::cREMOTE_PORT + 1);

  read_port.reset(new core::tPort<tBBVector>(read_pci));

  this->read_port_raw = static_cast<core::tPortBase*>(read_port->GetWrapped());
  ::finroc::blackboard::tAbstractBlackboardServerRaw::CheckType(type);
  this->write_port_raw = write;
  SetPublished(read_port->GetDefaultBuffer());

  Resize(*published, elements, elements);

  tBlackboardManager::GetInstance()->Init();
}

template<typename T>
void tBlackboardServer<T>::AsynchChange(tConstChangeTransactionVar& buf, int index, int offset, bool check_lock)
{
  {
    util::tLock lock2(this->bb_lock);
    if (check_lock && locked != NULL)
    {
      CheckCurrentLock(lock2);
      if (locked != NULL)    // ok, we don't get lock now... defer command to next unlock
      {
        DeferAsynchChangeCommand(buf, index, offset);
        return;
      }
    }

    assert(((!check_lock) || (!this->PendingTasks())));

    // duplicate current buffer
    assert((locked == NULL));
    DuplicateAndLock();

    // apply asynch change

    // apply asynch change
    //locked.getBuffer().put(offset, buf.getBuffer(), 0, buf.getSize());

    this->ApplyAsynchChange(*locked, buf, index, offset);

    // commit changes
    CommitLocked();
    assert((locked == NULL));
    //processPendingCommands();
  }
}

template<typename T>
void tBlackboardServer<T>::CheckCurrentLock(util::tLock& passed_lock)
{
  if (locked != NULL && util::tTime::GetCoarse() > last_keep_alive + cUNLOCK_TIMEOUT)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Blackboard server: Lock timed out... unlocking");

    lock_id = lock_iDGen.IncrementAndGet();

    locked.reset();
    FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Thread ", util::tThread::CurrentThread()->ToString(), ": lock = null");
    bool p = this->ProcessPendingCommands(passed_lock);
    if ((!p) && (!IsLocked()))
    {
      this->ProcessPendingAsynchChangeTasks();
    }
  }
}

template<typename T>
void tBlackboardServer<T>::CommitLocked()
{
  assert((locked != NULL && GetManager(locked)->IsLocked()));

  // process any waiting asynch change commands
  this->ProcessPendingAsynchChangeTasks();

  // publish new buffer
  core::tPortDataManager* mgr = GetManager(locked);
  mgr->lock_iD = -1;

  published = locked.get();
  read_port->Publish(locked);
  locked.reset();

  lock_id = lock_iDGen.IncrementAndGet();
  //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = null");
}

template<typename T>
void tBlackboardServer<T>::DirectCommit(tBBVectorVar& new_buffer)
{
  if (new_buffer == NULL)
  {
    return;
  }

  {
    util::tLock lock2(this->bb_lock);
    if (locked != NULL)    // note: current lock is obsolete, since we have a completely new buffer
    {
      //lockID = lockIDGen.incrementAndGet(); // make sure, next unlock won't do anything => done in commitLocked()  // note: current lock is obsolete, since we have a completely new buffer
      //lockID = lockIDGen.incrementAndGet(); // make sure, next unlock won't do anything => done in commitLocked()

      locked.reset();
    }

    // Clear any asynch change commands from queue, since they were for old buffer
    this->ClearAsyncChangeTasks();

    // commit new buffer

    locked = std::move(new_buffer);

    CommitLocked();

    assert((locked == NULL));

    // any threads that want to lock this?
    this->ProcessPendingCommands(lock2);
  }
}

template<typename T>
void tBlackboardServer<T>::DuplicateAndLock()
{
  assert((locked == NULL));
  lock_id = lock_iDGen.IncrementAndGet();
  lock_time = util::tTime::GetCoarse();
  last_keep_alive = lock_time;

  locked = write->GetBufferForReturn<tBBVector>();

  //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = " + locked.toString());

  this->CopyBlackboardBuffer(*published, *locked);

  GetManager(locked)->lock_iD = lock_id;
}

template<typename T>
void tBlackboardServer<T>::KeepAlive(int lock_id_)
{
  {
    util::tLock lock2(this->bb_lock);
    if (locked != NULL && this->lock_id == lock_id_)
    {
      last_keep_alive = util::tTime::GetCoarse();
    }
  }
}

template<typename T>
void tBlackboardServer<T>::LockCheck()
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
typename tAbstractBlackboardServer<T>::tBBVectorVar tBlackboardServer<T>::WriteLock(int64 timeout)
{
  {
    util::tLock lock2(this->bb_lock);
    if (locked != NULL || this->PendingTasks())    // make sure lock command doesn't "overtake" others
    {
      CheckCurrentLock(lock2);
      if (locked != NULL || this->PendingTasks())
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

      // ok... we have lock here
      assert((locked == NULL));
    }

    //System.out.println("Thread " + Thread.currentThread().toString() + ": handleLock");

    DuplicateAndLock();

    core::tPortDataManager* mgr = GetManager(locked);
    assert((locked != NULL && mgr->IsLocked()));
    mgr->AddLock();  // second lock for PortDataPtr duplication  // return buffer with one read lock

    return tBBVectorVar(mgr);
  }
}

template<typename T>
void tBlackboardServer<T>::WriteUnlock(tBBVectorVar& buf)
{
  if (buf == NULL)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "blackboard write unlock without providing buffer - strange indeed - ignoring");
    return;
  }

  {
    util::tLock lock2(this->bb_lock);
    core::tPortDataManager* bufmgr = GetManager(buf);
    if (this->lock_id != bufmgr->lock_iD)
    {
      FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG, log_domain, "Skipping outdated unlock");

      return;
    }

    assert((locked != NULL));
    assert((bufmgr->IsLocked()));

    if (buf != locked)
    {
      locked = std::move(buf);
      assert(GetManager(locked)->IsLocked());
    }
    else
    {
      buf.reset();
    }

    CommitLocked();
    this->ProcessPendingCommands(lock2);
  }
}

} // namespace finroc
} // namespace blackboard

