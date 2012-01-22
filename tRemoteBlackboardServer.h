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

#ifndef plugins__blackboard__tRemoteBlackboardServer_h__
#define plugins__blackboard__tRemoteBlackboardServer_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "plugins/blackboard/tBlackboardManager.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "core/datatype/tNumber.h"

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * Dummy object to handle/categorise remote blackboards
 */
class tRemoteBlackboardServer : public tAbstractBlackboardServer<core::tNumber>
{
protected:

  virtual void AsynchChange(tConstChangeTransactionVar& buf, int index, int offset, bool check_lock)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual void DirectCommit(tBBVectorVar& buf)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual bool IsLocked()
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual bool IsSingleBuffered()
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual void KeepAlive(int lock_id)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual tAbstractBlackboardServer<core::tNumber>::tConstBBVectorVar ReadLock(int64 timeout)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  //    @Override
  //    protected BlackboardBuffer readPart(int offset, int length, int timeout) {
  //        throw new RuntimeException("Operation not supported");
  //    }

  virtual void ReadUnlock(int lock_id)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual tAbstractBlackboardServer<core::tNumber>::tBBVectorVar WriteLock(int64 timeout)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual void WriteUnlock(tBBVectorVar& buf)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

public:

  tRemoteBlackboardServer(const util::tString& name) :
    tAbstractBlackboardServer<core::tNumber>(name, tBlackboardManager::cREMOTE, NULL)
  {
  }

  virtual void LockCheck()
  {
    // do nothing
  }

};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tRemoteBlackboardServer_h__
