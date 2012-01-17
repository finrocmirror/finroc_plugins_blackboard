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
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"
#include "core/portdatabase/tFinrocTypeInfo.h"
#include "rrlib/finroc_core_utils/tTime.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"

namespace finroc
{
namespace blackboard
{
tAbstractBlackboardServerRaw::tAbstractBlackboardServerRaw(const util::tString& bb_name, int category, core::tFrameworkElement* parent) :
    core::tFrameworkElement(parent == NULL ? tBlackboardManager::GetInstance()->GetCategory(category) : parent, bb_name, tBlackboardManager::GetInstance()->GetCategory(category)->default_flags, -1),
    pending_major_tasks(),
    wakeup_thread(-1),
    bb_lock(core::tLockOrderLevels::cINNER_MOST - 1000),
    read_port_raw(NULL),
    write_port_raw(NULL),
    category_index(category),
    my_category(tBlackboardManager::GetInstance()->GetCategory(category))
{
  // this(bbName,category,BlackboardManager.getInstance().getCategory(category).defaultFlags,parent);
}

void tAbstractBlackboardServerRaw::CheckType(rrlib::serialization::tDataTypeBase dt)
{
  __attribute__((unused))
  tBlackboardTypeInfo* ti = GetBlackboardTypeInfo(dt);
  assert(((ti != NULL && ti->blackboard_type != NULL && core::tFinrocTypeInfo::IsMethodType(ti->blackboard_type))) && "Please register Blackboard types using BlackboardPlugin class");
}

void tAbstractBlackboardServerRaw::PrepareDelete()
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

bool tAbstractBlackboardServerRaw::ProcessPendingCommands(util::tLock& passed_lock)
{
  //System.out.println(createThreadString() + ": process pending commands");
  if (pending_major_tasks.Size() == 0)
  {
    //System.out.println(createThreadString() + ": nothing to do");
    return false;
  }
  assert((wakeup_thread == -1));
  tBlackboardTask next_task = pending_major_tasks.Remove(0);
  wakeup_thread = next_task.thread_uid;
  //System.out.println(createThreadString() + ": waking up thread " + wakeupThread);
  bb_lock.monitor.NotifyAll(passed_lock);
  return true;
}

bool tAbstractBlackboardServerRaw::WaitForLock(util::tLock& passed_lock, int64 timeout)
{
  tBlackboardTask task;
  //task.method = method;
  task.thread_uid = util::sThreadUtil::GetCurrentThreadId();
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
      FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "Wait interrupted in AbstractBlackboardServer - shouldn't happen... usually");
    }
    wait_for = timeout - (util::tTime::GetCoarse() - start_time);
    //System.out.println(createThreadString() + ": left wait; waitFor = " + waitFor + "; wakeupThread = " + wakeupThread);
    if (wakeup_thread == util::sThreadUtil::GetCurrentThreadId())
    {
      // ok, it's our turn now
      pending_major_tasks.RemoveElem(task);
      wakeup_thread = -1;

      assert((!IsLocked()));
      return true;
    }
  }

  // ok, time seems to have run out - we have synchronized context though - so removing task is safe
  //System.out.println(createThreadString() + ": time has run out; isLocked() = " + isLocked());
  pending_major_tasks.RemoveElem(task);

  assert(((IsLocked())) && "Somebody forgot thread waiting on blackboard");
  return false;
}

} // namespace finroc
} // namespace blackboard

