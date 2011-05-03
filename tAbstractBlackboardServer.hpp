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
#include "plugins/blackboard/tBlackboardTypeInfo.h"
#include "plugins/blackboard/tBlackboardPlugin.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
core::tPortInterface tAbstractBlackboardServer<T>::cMETHODS("Blackboard Interface", true);

template<typename T>
typename core::tPort1Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tBBVectorVar, int> tAbstractBlackboardServer<T>::cLOCK(tAbstractBlackboardServer::cMETHODS, "Write Lock", "timeout", true);

template<typename T>
typename core::tPort2Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tConstBBVectorVar, int, int> tAbstractBlackboardServer<T>::cREAD_LOCK(tAbstractBlackboardServer::cMETHODS, "Read Lock", "timeout", "dummy", true);

template<typename T>
typename core::tVoid1Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tBBVectorVar> tAbstractBlackboardServer<T>::cUNLOCK(tAbstractBlackboardServer::cMETHODS, "Write Unlock", "Blackboard Buffer", false);

template<typename T>
core::tVoid1Method<tAbstractBlackboardServer<T>*, int> tAbstractBlackboardServer<T>::cREAD_UNLOCK(tAbstractBlackboardServer::cMETHODS, "Read Unlock", "Lock ID", false);

template<typename T>
typename core::tVoid3Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tConstChangeTransactionVar, int, int> tAbstractBlackboardServer<T>::cASYNCH_CHANGE(tAbstractBlackboardServer::cMETHODS, "Asynchronous Change", "Blackboard Buffer", "Start Index", "Custom Offset", false);

template<typename T>
typename core::tVoid1Method<tAbstractBlackboardServer<T>*, typename tAbstractBlackboardServer<T>::tBBVectorVar> tAbstractBlackboardServer<T>::cDIRECT_COMMIT(tAbstractBlackboardServer::cMETHODS, "Direct Commit", "Buffer", false);

template<typename T>
core::tPort0Method<tAbstractBlackboardServer<T>*, int8> tAbstractBlackboardServer<T>::cIS_SINGLE_BUFFERED(tAbstractBlackboardServer::cMETHODS, "Is Single Buffered?", false);

template<typename T>
core::tVoid1Method<tAbstractBlackboardServer<T>*, int> tAbstractBlackboardServer<T>::cKEEP_ALIVE(tAbstractBlackboardServer::cMETHODS, "KeepAliveSignal", "Lock ID", false);

template<typename T>
tAbstractBlackboardServer<T>::tAbstractBlackboardServer(const util::tString& bb_name, int category, core::tFrameworkElement* parent) :
    tAbstractBlackboardServerRaw(bb_name, category, parent),
    pending_asynch_change_tasks()
{
}

template<typename T>
void tAbstractBlackboardServer<T>::ClearAsyncChangeTasks()
{
  pending_asynch_change_tasks.clear();
}

template<typename T>
rrlib::serialization::tDataTypeBase tAbstractBlackboardServer<T>::GetBlackboardMethodType(rrlib::serialization::tDataTypeBase dt)
{
  tBlackboardTypeInfo* ti = GetBlackboardTypeInfo(dt);
  if (ti != NULL && ti->blackboard_type != NULL)
  {
    return ti->blackboard_type;
  }
  tBlackboardPlugin::RegisterBlackboardType<T>(dt);
  return GetBlackboardTypeInfo(dt)->blackboard_type;
}

template<typename T>
void tAbstractBlackboardServer<T>::PostChildInit()
{
  // check that methods have correct indices
  assert((cLOCK.GetMethodId() == 0));
  assert((cREAD_LOCK.GetMethodId() == 1));
  ::finroc::blackboard::tAbstractBlackboardServerRaw::PostChildInit();
}

template<typename T>
void tAbstractBlackboardServer<T>::ProcessPendingAsynchChangeTasks()
{
  for (size_t i = 0u; i < pending_asynch_change_tasks.size(); i++)
  {
    tAsynchChangeTask& task = pending_asynch_change_tasks[i];
    AsynchChange(task.buffer, static_cast<int>(task.index), static_cast<int>(task.offset), false);

    task.buffer.reset();
  }
  pending_asynch_change_tasks.clear();
}

template<typename T>
bool tAbstractBlackboardServer<T>::ProcessPendingCommands(util::tLock& passed_lock)
{
  //System.out.println(createThreadString() + ": process pending commands");
  if (this->pending_major_tasks.Size() == 0)
  {
    //System.out.println(createThreadString() + ": nothing to do");
    return false;
  }
  assert((this->wakeup_thread == -1));
  tBlackboardTask next_task = this->pending_major_tasks.Remove(0);
  this->wakeup_thread = next_task.thread_uid;
  //System.out.println(createThreadString() + ": waking up thread " + wakeupThread);
  this->bb_lock.monitor.NotifyAll(passed_lock);
  return true;
}

} // namespace finroc
} // namespace blackboard

