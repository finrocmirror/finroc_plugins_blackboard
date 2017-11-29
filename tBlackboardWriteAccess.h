//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/blackboard/tBlackboardWriteAccess.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 * \brief   Contains tBlackboardWriteAccess
 *
 * \b tBlackboardWriteAccess
 *
 * Objects of this class are used to acquire write access to blackboards.
 * The lock is released as soon as the objects goes out of scope.
 *
 * This class is derived from tBlackboardReadAccess so that it can also
 * be used in places where only read access is required.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__tBlackboardWriteAccess_h__
#define __plugins__blackboard__tBlackboardWriteAccess_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboardReadAccess.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace blackboard
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Object for blackboard write access
/*!
 * Objects of this class are used to acquire write access to blackboards.
 * The lock is released as soon as the objects goes out of scope.
 *
 * This class is derived from tBlackboardReadAccess so that it can also
 * be used in places where only read access is required.
 */
template <typename T>
class tBlackboardWriteAccess : public tBlackboardReadAccess<T>
{
  typedef tBlackboardReadAccess<T> tBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param blackboard Blackboard to access
   * \param timeout Timeout for lock (in ms)
   * \param deferred_lock_check If set to true, this constructor does not check whether lock could be
   *                            acquired (and does not block). The lock is checked on the first access instead.
   *
   * \exception tLockException is thrown if lock fails (and deferred_lock_check is false)
   */
  tBlackboardWriteAccess(tBlackboardClient<T>& blackboard, const rrlib::time::tDuration& timeout = std::chrono::seconds(10), bool deferred_lock_check = false) :
    tBase(blackboard, blackboard),
    locked_buffer_future(),
    locked_buffer(),
    changed(false)
  {
    this->timeout = timeout;
    ConstructorImplementation(deferred_lock_check);
  }

  /*!
   * \param blackboard Blackboard to access
   * \param timeout Timeout for lock (in ms)
   * \param deferred_lock_check If set to true, this constructor does not check whether lock could be
   *                            acquired (and does not block). The lock is checked on the first access instead.
   *
   * \exception tLockException is thrown if lock fails (and deferred_lock_check is false)
   */
  tBlackboardWriteAccess(tBlackboard<T>& blackboard, const rrlib::time::tDuration& timeout = std::chrono::seconds(10), bool deferred_lock_check = false) :
    tBase(blackboard.GetClient(), blackboard.GetClient()),
    locked_buffer_future(),
    locked_buffer(),
    changed(false)
  {
    this->timeout = timeout;
    ConstructorImplementation(deferred_lock_check);
  }

  ~tBlackboardWriteAccess()
  {
    if (locked_buffer_raw)
    {
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Releasing write lock on blackboard '", blackboard.GetName(), "' at ", rrlib::time::Now());
      if (changed)
      {
        locked_buffer.CommitCurrentBuffer();
      }
      else
      {
        locked_buffer.CommitNoChanges();
      }
    }
  }

  inline T& operator[](size_t index)
  {
    return Get(index);
  }

  /*!
   * \param index Element index
   * \return Element at index
   *
   * \exception tLockException is thrown if lock fails (can only occur if locking was deferred in constructor)
   */
  inline T& Get(size_t index)
  {
    CheckLock();
    if (index >= locked_buffer_raw->size())
    {
      throw std::runtime_error("Blackboard write access out of bounds");
    }
    changed = true;
    return (*locked_buffer.Get())[index];
  }

  /*!
   * \return Timestamp of locked blackboard data
   */
  rrlib::time::tTimestamp GetTimestamp()
  {
    CheckLock();
    return locked_buffer->GetTimestamp();
  }

  /*!
   * \param new_size New size (number of elements) in blackboard
   *
   * \exception tLockException is thrown if lock fails (can only occur if locking was deferred in constructor)
   */
  inline void Resize(size_t new_size)
  {
    if (new_size != this->Size())
    {
      rrlib::rtti::ResizeVector(*locked_buffer.Get(), new_size);
      changed = true;
      locked_buffer_raw = locked_buffer.Get();
    }
  }

  /*!
   * Sets timestamp of locked blackboard data
   *
   * \param timestamp Value to set timestamp to
   */
  void SetTimestamp(const rrlib::time::tTimestamp& timestamp)
  {
    CheckLock();
    locked_buffer.SetTimestamp(timestamp);
  }

  /*!
   * \return Number of elements in blackboard
   *
   * \exception tLockException is thrown if lock fails (can only occur if locking was deferred in constructor)
   */
  inline size_t Size()
  {
    CheckLock();
    return locked_buffer_raw->size();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  typedef typename tBlackboardClient<T>::tBuffer tBuffer;

  using tBase::blackboard;
  using tBase::locked_buffer_raw;
  using tBase::timeout;

  /*! Future for locked buffer */
  rpc_ports::tFuture<internal::tLockedBuffer<tBuffer>> locked_buffer_future;

  /*! Buffer from locked blackboard */
  internal::tLockedBuffer<tBuffer> locked_buffer;

  /*! True as soon as changes have been made to buffer */
  bool changed;


  /*!
   * Check whether locked_buffer has already been obtained.
   * If not, obtain it.
   */
  void CheckLock()
  {
    if (!locked_buffer_raw)
    {
      try
      {
        locked_buffer = locked_buffer_future.Get(timeout);
        locked_buffer_raw = locked_buffer.GetConst();
      }
      catch (const rpc_ports::tRPCException& e)
      {
        throw tLockException(e.GetType());
      }
    }
  }

  /*!
   * Code that would be identical in both constructors
   */
  void ConstructorImplementation(bool deferred_lock_check)
  {
    if (!this->blackboard)
    {
      throw tLockException(rpc_ports::tFutureStatus::INVALID_CALL);
    }
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Acquiring write lock on blackboard '", blackboard.GetName(), "' at ", rrlib::time::Now());
    locked_buffer_future = this->blackboard.WriteLock(timeout);
    if (!deferred_lock_check)
    {
      CheckLock();
    }
  }

  // no heap allocation permitted
  void *operator new(size_t);
  void *operator new[](size_t);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
