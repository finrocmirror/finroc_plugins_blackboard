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

#ifndef plugins__blackboard__tBlackboardServer_h__
#define plugins__blackboard__tBlackboardServer_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "core/port/tPort.h"
#include "rrlib/serialization/tDataTypeBase.h"
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
/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard server
 */
template<typename T>
class tBlackboardServer : public tAbstractBlackboardServer<T>
{
private:

  typedef typename tAbstractBlackboardServer<T>::tBBVector tBBVector;
  typedef typename tAbstractBlackboardServer<T>::tBBVectorVar tBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tConstBBVectorVar tConstBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransaction tChangeTransaction;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransactionVar tChangeTransactionVar;
  typedef typename tAbstractBlackboardServer<T>::tConstChangeTransactionVar tConstChangeTransactionVar;

  using tAbstractBlackboardServer<T>::log_domain;

  /*! Unlock timeout in ms - if no keep-alive signal occurs in this period of time */
  static const int64 cUNLOCK_TIMEOUT = 1000;

  /*! Interface port for write access */
  core::tInterfaceServerPort* write;

  /*! Is blackboard currently locked? - in this case points to duplicated buffer */
  tBBVectorVar locked;

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
  tBBVector* published;

public:

  /*! read port */
  ::std::shared_ptr<core::tPort<tBBVector> > read_port;

private:

  /*!
   * Check if lock timed out (only call in synchronized/exclusive access context)
   */
  void CheckCurrentLock(util::tLock& passed_lock);

  /*! Release lock and commit changed buffer (needs to be called in synchronized context) */
  void CommitLocked();

  /*! Duplicate and lock current buffer (needs to be called in synchronized context) */
  void DuplicateAndLock();

  /*!
   * \return Port data manager for buffer
   */
  template <typename Q>
  inline static core::tPortDataManager* GetManager(std::shared_ptr<Q>& t)
  {
    return core::tPortDataManager::GetManager(t);
  }

  /*!
   * This method exists due to current imperfectness in java->c++ converter
   *
   * \param p
   */
  inline void SetPublished(tBBVector* p)
  {
    published = p;
  }

protected:

  virtual void AsynchChange(tConstChangeTransactionVar& buf, int index, int offset, bool check_lock);

  virtual void DirectCommit(tBBVectorVar& new_buffer);

  virtual bool IsLocked()
  {
    return locked != NULL;
  }

  virtual bool IsSingleBuffered()
  {
    return false;
  }

  virtual void KeepAlive(int lock_id_);

  virtual typename tAbstractBlackboardServer<T>::tConstBBVectorVar ReadLock(int64 timeout)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Client must not attempt read lock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::eINVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual void ReadUnlock(int lock_id_)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING, log_domain, "warning: Client must not attempt read unlock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::eINVALID_PARAM, CODE_LOCATION_MACRO);
  }

  //    @Override
  //    protected BlackboardBuffer readPart(int offset, int length, int timeout) {
  //        // current buffer (note: we get it from readPort, since -this way- call does not need to be executed in synchronized context)
  //        @Const BlackboardBuffer buffer = (BlackboardBuffer)readPort.getLockedUnsafeRaw();
  //        assert(buffer.getManager().isLocked());
  //
  //        // prepare and set return value
  //        BlackboardBuffer send = (BlackboardBuffer)write.getUnusedBuffer(buffer.getType());
  //        send.resize(1, 1, length, false); // ensure minimal size
  //        send.getBuffer().put(0, buffer.getBuffer(), offset, length);
  //        send.bbCapacity = buffer.bbCapacity;
  //        send.elements = buffer.elements;
  //        send.elementSize = buffer.elementSize;
  //
  //        // release old lock
  //        buffer.getManager().getCurrentRefCounter().releaseLock();
  //
  //        // return buffer with one read lock
  //        send.getManager().getCurrentRefCounter().setLocks((byte)1);
  //        return send;
  //    }

  virtual typename tAbstractBlackboardServer<T>::tBBVectorVar WriteLock(int64 timeout);

  virtual void WriteUnlock(tBBVectorVar& buf);

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
  tBlackboardServer(const util::tString& description, rrlib::serialization::tDataTypeBase type, int capacity, int elements, int elem_size, core::tFrameworkElement* parent = NULL, bool shared = true);

  /*!
   * \param description Name/Uid of blackboard
   * \param type Data Type of blackboard content
   * \param mc_type Type of method calls
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   */
  tBlackboardServer(const util::tString& description, rrlib::serialization::tDataTypeBase type, core::tFrameworkElement* parent = NULL, bool shared = true);

  virtual void LockCheck();

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tBlackboardServer.hpp"

#endif // plugins__blackboard__tBlackboardServer_h__
