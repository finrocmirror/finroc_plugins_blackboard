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
#include "blackboard/tAbstractBlackboardServer.h"
#include "blackboard/tBlackboardPlugin.h"
#include "finroc_core_utils/tTime.h"

namespace finroc
{
namespace blackboard
{
core::tPortInterface tAbstractBlackboardServer::cMETHODS("Blackboard Interface");
core::tPort1Method<tAbstractBlackboardServer*, tBlackboardBuffer*, int64> tAbstractBlackboardServer::cLOCK(tAbstractBlackboardServer::cMETHODS, "Lock", "timeout", true);
core::tPort2Method<tAbstractBlackboardServer*, const tBlackboardBuffer*, int64, int> tAbstractBlackboardServer::cREAD_LOCK(tAbstractBlackboardServer::cMETHODS, "Lock", "timeout", "dummy", true);
core::tVoid1Method<tAbstractBlackboardServer*, tBlackboardBuffer*> tAbstractBlackboardServer::cUNLOCK(tAbstractBlackboardServer::cMETHODS, "Unlock", "Blackboard Buffer", false);
core::tVoid1Method<tAbstractBlackboardServer*, int> tAbstractBlackboardServer::cREAD_UNLOCK(tAbstractBlackboardServer::cMETHODS, "Unlock", "Lock ID", false);
core::tVoid2Method<tAbstractBlackboardServer*, int, const tBlackboardBuffer*> tAbstractBlackboardServer::cASYNCH_CHANGE(tAbstractBlackboardServer::cMETHODS, "Asynchronous Change", "Offset", "Blackboard Buffer (null if only read locked", false);
core::tPort3Method<tAbstractBlackboardServer*, tBlackboardBuffer*, int, int, int> tAbstractBlackboardServer::cREAD_PART(tAbstractBlackboardServer::cMETHODS, "Read Part", "Offset", "Length", "Timeout", true);
core::tVoid1Method<tAbstractBlackboardServer*, tBlackboardBuffer*> tAbstractBlackboardServer::cDIRECT_COMMIT(tAbstractBlackboardServer::cMETHODS, "Direct Commit", "Buffer", false);
core::tPort0Method<tAbstractBlackboardServer*, int8> tAbstractBlackboardServer::cIS_SINGLE_BUFFERED(tAbstractBlackboardServer::cMETHODS, "Is Single Buffered?", false);
core::tVoid1Method<tAbstractBlackboardServer*, int> tAbstractBlackboardServer::cKEEP_ALIVE(tAbstractBlackboardServer::cMETHODS, "KeepAliveSignal", "Lock ID", false);

tAbstractBlackboardServer::tAbstractBlackboardServer(const util::tString& bb_name, int category, core::tFrameworkElement* parent) :
    core::tFrameworkElement(bb_name, parent == NULL ? tBlackboardManager::GetInstance()->GetCategory(category) : parent, tBlackboardManager::GetInstance()->GetCategory(category)->default_flags, -1),
    pending_major_tasks(),
    pending_asynch_change_tasks(),
    wakeup_thread(-1),
    bb_lock(core::tLockOrderLevels::cINNER_MOST - 1000),
    read_port(NULL),
    write_port(NULL),
    category_index(category),
    my_category(tBlackboardManager::GetInstance()->GetCategory(category))
{
  // this(bbName,category,BlackboardManager.getInstance().getCategory(category).defaultFlags,parent);
}

void tAbstractBlackboardServer::ClearAsyncChangeTasks()
{
  for (size_t i = 0u; i < pending_asynch_change_tasks.Size(); i++)
  {
    pending_asynch_change_tasks.Get(i)->Recycle2();
  }
  pending_asynch_change_tasks.Clear();
}

void tAbstractBlackboardServer::DeferAsynchChangeCommand(int offset, const tBlackboardBuffer* buf)
{
  tBlackboardTask* task = GetUnusedBlackboardTask();
  task->offset = offset;
  task->buffer = buf;
  pending_asynch_change_tasks.Add(task);
}

tBlackboardTask* tAbstractBlackboardServer::GetUnusedBlackboardTask()
{
  tBlackboardTask* task = tBlackboardPlugin::task_pool->GetUnused();
  if (task == NULL)
  {
    task = new tBlackboardTask();
    tBlackboardPlugin::task_pool->Attach(task, false);
  }
  return task;
}

void tAbstractBlackboardServer::PrepareDelete()
{
  util::tLock lock1(this);
  {
    util::tLock lock2(bb_lock);
    if (tBlackboardManager::GetInstance() != NULL)    // we don't need to remove it, if blackboard manager has already been deleted
    {
      my_category->Remove(this);
    }

    ClearAsyncChangeTasks();
  }
}

void tAbstractBlackboardServer::ProcessPendingAsynchChangeTasks()
{
  for (size_t i = 0u; i < pending_asynch_change_tasks.Size(); i++)
  {
    tBlackboardTask* task = pending_asynch_change_tasks.Get(i);
    AsynchChange(task->offset, task->buffer, false);
    task->buffer = NULL;  // already unlocked by method
    task->Recycle2();
  }
  pending_asynch_change_tasks.Clear();
}

bool tAbstractBlackboardServer::ProcessPendingCommands(util::tLock& passed_lock)
{
  //System.out.println(createThreadString() + ": process pending commands");
  if (pending_major_tasks.Size() == 0)
  {
    //System.out.println(createThreadString() + ": nothing to do");
    return false;
  }
  assert((wakeup_thread == -1));
  tBlackboardTask* next_task = pending_major_tasks.Remove(0);
  wakeup_thread = next_task->thread_uid;
  //System.out.println(createThreadString() + ": waking up thread " + wakeupThread);
  bb_lock.monitor.NotifyAll(passed_lock);
  return true;
}

bool tAbstractBlackboardServer::WaitForLock(util::tLock& passed_lock, int64 timeout)
{
  tBlackboardTask* task = GetUnusedBlackboardTask();
  //task.method = method;
  task->thread_uid = util::sThreadUtil::GetCurrentThreadId();
  pending_major_tasks.Add(task);
  int64 start_time = util::tTime::GetCoarse();
  //long curTime = startTime;
  int64 wait_for = timeout;
  //System.out.println(createThreadString() + ": waiting " + timeout + " ms for lock");
  while (wait_for > 0)
  {
    try
    {
      //System.out.println(createThreadString() + ": entered wait");
      bb_lock.monitor.Wait(passed_lock, wait_for);
    }
    catch (const util::tInterruptedException& e)
    {
      //e.printStackTrace();
      util::tSystem::out.Println("Wait interrupted in AbstractBlackboardServer - shouldn't happen... usually");
    }
    wait_for = timeout - (util::tTime::GetCoarse() - start_time);
    //System.out.println(createThreadString() + ": left wait; waitFor = " + waitFor + "; wakeupThread = " + wakeupThread);
    if (wakeup_thread == util::sThreadUtil::GetCurrentThreadId())
    {
      // ok, it's our turn now
      pending_major_tasks.RemoveElem(task);
      wakeup_thread = -1;
      task->Recycle2();
      assert((!IsLocked()));
      return true;
    }
  }

  // ok, time seems to have run out - we have synchronized context though - so removing task is safe
  //System.out.println(createThreadString() + ": time has run out; isLocked() = " + isLocked());
  pending_major_tasks.RemoveElem(task);
  task->Recycle2();
  assert(((IsLocked())) && "Somebody forgot thread waiting on blackboard");
  return false;
}

} // namespace finroc
} // namespace blackboard

