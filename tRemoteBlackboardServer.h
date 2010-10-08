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
#include "finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__BLACKBOARD__TREMOTEBLACKBOARDSERVER_H
#define PLUGINS__BLACKBOARD__TREMOTEBLACKBOARDSERVER_H

#include "blackboard/tBlackboardManager.h"
#include "blackboard/tAbstractBlackboardServer.h"

namespace finroc
{
namespace blackboard
{
class tBlackboardBuffer;

/*!
 * \author Max Reichardt
 *
 * Dummy object to handle/categorise remote blackboards
 */
class tRemoteBlackboardServer : public tAbstractBlackboardServer
{
protected:

  virtual void AsynchChange(int i, const tBlackboardBuffer* buf, bool check_lock)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual void DirectCommit(tBlackboardBuffer* buf)
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

  virtual const tBlackboardBuffer* ReadLock(int64 timeout)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual tBlackboardBuffer* ReadPart(int offset, int length, int timeout)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual void ReadUnlock(int lock_id)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual tBlackboardBuffer* WriteLock(int64 timeout)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

  virtual void WriteUnlock(tBlackboardBuffer* buf)
  {
    throw util::tRuntimeException("Operation not supported", CODE_LOCATION_MACRO);
  }

public:

  tRemoteBlackboardServer(const util::tString& name) :
      tAbstractBlackboardServer(name, tBlackboardManager::cREMOTE, NULL)
  {
  }

  virtual void LockCheck()
  {
    // do nothing
  }

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TREMOTEBLACKBOARDSERVER_H
