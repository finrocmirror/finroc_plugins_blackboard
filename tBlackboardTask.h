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

#ifndef PLUGINS__BLACKBOARD__TBLACKBOARDTASK_H
#define PLUGINS__BLACKBOARD__TBLACKBOARDTASK_H

#include "core/port/std/tPortDataManager.h"
#include "plugins/blackboard/tBlackboardBuffer.h"
#include "rrlib/finroc_core_utils/container/tReusable.h"

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * Class to store pending blackboard tasks
 */
class tBlackboardTask : public util::tReusable
{
public:
  /* implements Task */

  /*! Method that is pending (possible are lock and asynch_change) */
  //public @Const AbstractMethod method;

  /*! In case a thread is waiting on BlackboardServer - his uid - may only be changed in context synchronized to blackboard server */
  int64 thread_uid;

  ///** Notify waiting thread on lock when ready (instead of calling executeTask()) */
  //public boolean notifyOnly;

  /*! BlackboardBuffer to use for task - if this is set, it will be unlocked with recycle */
  const tBlackboardBuffer* buffer;

  /*! Offset for asynch change command */
  int offset;

  tBlackboardTask() :
      thread_uid(0),
      buffer(NULL),
      offset(0)
  {}

  /*! Recycle task */
  inline void Recycle2()
  {
    if (buffer != NULL)
    {
      buffer->GetManager()->ReleaseLock();
      buffer = NULL;
    }
    //method = null;
    ::finroc::util::tReusable::Recycle();
  }

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TBLACKBOARDTASK_H
