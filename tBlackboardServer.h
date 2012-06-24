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
#include "rrlib/rtti/tDataTypeBase.h"
#include "rrlib/finroc_core_utils/log/tLogUser.h"
#include "core/port/rpc/tMethodCallException.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"


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

  /*! Interface port for write access */
  core::tInterfaceServerPort* write;

  /*! Is blackboard currently locked? - in this case points to duplicated buffer */
  tBBVectorVar locked;

  /*! Time when last lock was performed */
  rrlib::time::tAtomicTimestamp lock_time;

  /*! Last time a keep-alive-signal was received */
  rrlib::time::tAtomicTimestamp last_keep_alive;

  /*! ID of current lock - against outdated unlocks */
  util::tAtomicInt lock_id_gen;

  int lock_id;

  /*!
   * Currently published MemBuffer - not extra locked - attached to lock of read port
   * In single buffered mode - this is the one and only buffer
   */
  tBBVector* published;

public:

  /*! read port */
  std::shared_ptr<core::tPort<tBBVector> > read_port;

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
  inline static core::tPortDataManager* GetManager(core::tPortDataPtr<Q>& t)
  {
    return t.GetManager();
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

  virtual typename tAbstractBlackboardServer<T>::tConstBBVectorVar ReadLock(const rrlib::time::tDuration& timeout)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "warning: Client must not attempt read lock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::tType::INVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual void ReadUnlock(int lock_id_)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "warning: Client must not attempt read unlock on multi-buffered blackboard - Call failed");
    throw core::tMethodCallException(core::tMethodCallException::tType::INVALID_PARAM, CODE_LOCATION_MACRO);
  }

  virtual typename tAbstractBlackboardServer<T>::tBBVectorVar WriteLock(const rrlib::time::tDuration& timeout);

  virtual void WriteUnlock(tBBVectorVar& buf);

public:

  /*!
   * \param name Name/Uid of blackboard
   * \param capacity Blackboard capacity (see BlackboardBuffer)
   * \param elements Number of element (see BlackboardBuffer)
   * \param elem_size Element size (see BlackboardBuffer)
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   * \param type Data Type of blackboard content
   */
  tBlackboardServer(const util::tString& name, int capacity, int elements, int elem_size, core::tFrameworkElement* parent = NULL, bool shared = true, rrlib::rtti::tDataTypeBase type = rrlib::rtti::tDataType<T>());

  /*!
   * \param name Name/Uid of blackboard
   * \param elements Initial number of elements
   * \param parent parent of BlackboardServer
   * \param shared Share blackboard with other runtime environments?
   * \param type Data Type of blackboard content
   */
  tBlackboardServer(const util::tString& name, int elements = 0, core::tFrameworkElement* parent = NULL, bool shared = true, rrlib::rtti::tDataTypeBase type = rrlib::rtti::tDataType<T>());

  virtual void LockCheck();

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tBlackboardServer.hpp"

namespace finroc
{
namespace blackboard
{
extern template class tBlackboardServer<tBlackboardBuffer>;
extern template class tBlackboardServer<rrlib::serialization::tMemoryBuffer>;

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardServer_h__
