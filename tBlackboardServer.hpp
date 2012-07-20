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
#include "core/portdatabase/tFinrocTypeInfo.h"
#include "core/port/tPortCreationInfo.h"
#include "core/port/tPortFlags.h"
#include "core/tCoreFlags.h"
#include "core/tLockOrderLevels.h"
#include "core/port/std/tPortBase.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"
#include "core/port/rpc/tInterfaceServerPort.h"
#include "core/port/std/tPortDataManager.h"

namespace finroc
{
namespace blackboard
{

template<typename T>
tBlackboardServer<T>::tBlackboardServer(const util::tString& name, int capacity, int elements, int elem_size, core::tFrameworkElement* parent, bool shared, rrlib::rtti::tDataTypeBase type) :
  tAbstractBlackboardServer<T>(name, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
  write(new core::tInterfaceServerPort("write", this, this->GetBlackboardMethodType(type), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
  locked(),
  lock_time(rrlib::time::cNO_TIME),
  last_keep_alive(rrlib::time::cNO_TIME),
  lock_id_gen(0),
  lock_id(0),
  published(),
  read_port()
{
  // this(name,1,parent,shared,type);
  assert(((!core::tFinrocTypeInfo::IsMethodType(type))) && "Please provide data type of content here");
  read_port.reset(new core::tPort<tBBVector>("read", this, core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0), core::tLockOrder(core::tLockOrderLevels::cREMOTE_PORT + 1)));

  this->read_port_raw = static_cast<core::tPortBase*>(read_port->GetWrapped());
  ::finroc::blackboard::tAbstractBlackboardServerRaw::CheckType(type);
  this->write_port_raw = write;
  SetPublished(read_port->GetDefaultBuffer());

  Resize(*published, 1, 1);

  tBlackboardManager::GetInstance()->Init();
  ClassicBlackboardResize(&((*published)[0]), capacity, elements, elem_size);
}

template<typename T>
tBlackboardServer<T>::tBlackboardServer(const util::tString& name, int elements, core::tFrameworkElement* parent, bool shared, rrlib::rtti::tDataTypeBase type) :
  tAbstractBlackboardServer<T>(name, shared ? tBlackboardManager::cSHARED : tBlackboardManager::cLOCAL, parent),
  write(new core::tInterfaceServerPort("write", this, this->GetBlackboardMethodType(type), this, shared ? core::tCoreFlags::cSHARED : 0, core::tLockOrderLevels::cREMOTE_PORT + 2)),
  locked(),
  lock_time(rrlib::time::cNO_TIME),
  last_keep_alive(rrlib::time::cNO_TIME),
  lock_id_gen(0),
  lock_id(0),
  published(),
  read_port()
{
  assert(((!core::tFinrocTypeInfo::IsMethodType(type))) && "Please provide data type of content here");
  read_port.reset(new core::tPort<tBBVector>("read", this, core::tPortFlags::cOUTPUT_PORT | (shared ? core::tCoreFlags::cSHARED : 0), core::tLockOrder(core::tLockOrderLevels::cREMOTE_PORT + 1)));

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
    tLock lock2(this->bb_lock);
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
void tBlackboardServer<T>::CheckCurrentLock(tLock& passed_lock)
{
  if (locked != NULL && rrlib::time::Now(false) > last_keep_alive.Load() + this->GetLockTimeout())
  {
    FINROC_LOG_PRINT(DEBUG, "Blackboard server: Lock timed out... unlocking");

    lock_id = lock_id_gen.IncrementAndGet();

    locked.reset();
    FINROC_LOG_PRINT(DEBUG, "Thread ", rrlib::thread::tThread::CurrentThread().GetName(), ": lock = null");
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
  mgr->lock_id = -1;

  published = locked.get();
  read_port->Publish(locked);
  locked.reset();

  lock_id = lock_id_gen.IncrementAndGet();
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
    tLock lock2(this->bb_lock);
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
  lock_id = lock_id_gen.IncrementAndGet();
  rrlib::time::tTimestamp now = rrlib::time::Now(false);
  lock_time.Store(now);
  last_keep_alive.Store(now);

  locked = write->GetBufferForReturn<tBBVector>();

  //System.out.println("Thread " + Thread.currentThread().toString() + ": lock = " + locked.toString());

  this->CopyBlackboardBuffer(*published, *locked);

  GetManager(locked)->lock_id = lock_id;
}

template<typename T>
void tBlackboardServer<T>::KeepAlive(int lock_id_)
{
  {
    tLock lock2(this->bb_lock);
    if (locked != NULL && this->lock_id == lock_id_)
    {
      last_keep_alive.Store(rrlib::time::Now(false));
    }
  }
}

template<typename T>
void tBlackboardServer<T>::LockCheck()
{
  rrlib::time::tTimestamp cur_time = rrlib::time::Now(false);
  if (last_keep_alive.Load() + this->GetLockTimeout() > cur_time)
  {
    return;
  }

  {
    tLock lock2(this->bb_lock);
    CheckCurrentLock(lock2);
  }
}

template<typename T>
typename tAbstractBlackboardServer<T>::tBBVectorVar tBlackboardServer<T>::WriteLock(const rrlib::time::tDuration& timeout)
{
  {
    tLock lock2(this->bb_lock);
    if (locked != NULL || this->PendingTasks())    // make sure lock command doesn't "overtake" others
    {
      CheckCurrentLock(lock2);
      if (locked != NULL || this->PendingTasks())
      {
        if (timeout <= rrlib::time::tDuration::zero())    // we do not need to enqueue lock commands with zero timeout
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
    FINROC_LOG_PRINT(WARNING, "blackboard write unlock without providing buffer - strange indeed - ignoring");
    return;
  }

  {
    tLock lock2(this->bb_lock);
    core::tPortDataManager* bufmgr = GetManager(buf);
    if (this->lock_id != bufmgr->lock_id)
    {
      FINROC_LOG_PRINT(DEBUG, "Skipping outdated unlock");

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

