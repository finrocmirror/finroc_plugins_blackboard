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

#ifndef PLUGINS__BLACKBOARD__TBLACKBOARDSERVER_H
#define PLUGINS__BLACKBOARD__TBLACKBOARDSERVER_H

#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "core/port/rpc/tMethodCallException.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"

namespace finroc
{
namespace core
{
class tInterfaceServerPort;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
class tBlackboardBuffer;

/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard server
 */
class tBlackboardServer : public tAbstractBlackboardServer
{
private:

  /*! Unlock timeout in ms - if no keep-alive signal occurs in this period of time */
  static const int64 cUNLOCK_TIMEOUT = 1000;

  /*! Interface port for write access */
  core::tInterfaceServerPort* write;

  /*! Is blackboard currently locked? - in this case points to duplicated buffer */
  tBlackboardBuffer* locked;

  /*! Time when last lock was performed */
  volatile int64 lock_time;

  /*! Last time a keep-alive-signal was received */
  volatile int64 last_keep_alive;

  /*! ID of current lock - against outdated unlocks */
  util::tAtomicInt lock_iDGen;

  int lock_id;

  /*!
   * Currently published MemBuffer - not extra locked - attached to lock of read port
   * In single buffered mode - this is the one and only buffer
   */
  tBlackboardBuffer* published;

  /*!
   * Check if lock timed out (only call in synchronized/exclusive access context)
   */
  void CheckCurrentLock(util::tLock& passed_lock);

  /*! Release lock and commit changed buffer (needs to be called in synchronized context) */
  void CommitLocked();

  /*! Duplicate and lock current buffer (needs to be called in synchronized context) */
  void DuplicateAndLock();

  /*!
   * This method exists due to current imperfectness in java->c++ converter
   *
   * \param p
   */
  inline void SetPublished(tBlackboardBuffer* p)
  {
    published = p;
  }

protected:

  virtual void AsynchChange(int offset, const tBlackboardBuffer* buf, bool check_lock);

  virtual void DirectCommit(tBlackboardBuffer* new_buffer);

  virtual bool IsLocked()
  {
    return locked != NULL;
  }

  virtual bool IsSingleBuffered()
  {
    return false;
  }

  virtual void KeepAlive(int lock_id_);

  virtual const tBlackboardBuffer* ReadLock(int64 timeout)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Client must not attempt read lock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::eINVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual tBlackboardBuffer* ReadPart(int offset, int length, int timeout);

  virtual void ReadUnlock(int lock_id_)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Client must not attempt read unlock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::eINVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual tBlackboardBuffer* WriteLock(int64 timeout);

  virtual void WriteUnlock(tBlackboardBuffer* buf);

public:

  /*!
   * \param description Name/Uid of blackboard
   * @parent parent of BlackboardServer
   */
  tBlackboardServer(const util::tString& description, core::tFrameworkElement* parent = NULL);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param capacity Blackboard capacity (see BlackboardBuffer)
   * \param elements Number of element (see BlackboardBuffer)
   * \param elem_size Element size (see BlackboardBuffer)
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tBlackboardServer(const util::tString& description, core::tDataType* type, int capacity, int elements, int elem_size, core::tFrameworkElement* parent = NULL, bool shared = true);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param mc_type Type of method calls
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tBlackboardServer(const util::tString& description, core::tDataType* type, core::tFrameworkElement* parent = NULL, bool shared = true);

  virtual void LockCheck();

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TBLACKBOARDSERVER_H
