//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/blackboard/tBlackboardReadAccess.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 * \brief   Contains tBlackboardReadAccess
 *
 * \b tBlackboardReadAccess
 *
 * Objects of this class are used to acquire read access to blackboards.
 * The lock is released as soon as the objects goes out of scope.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__tBlackboardReadAccess_h__
#define __plugins__blackboard__tBlackboardReadAccess_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tLockException.h"

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
//! Object for blackboard read access
/*!
 * Objects of this class are used to acquire read access to blackboards.
 * The lock is released as soon as the objects goes out of scope.
 */
template <typename T>
class tBlackboardReadAccess : private rrlib::util::tNoncopyable
{

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
  tBlackboardReadAccess(tBlackboardClient<T>& blackboard, const rrlib::time::tDuration& timeout = std::chrono::seconds(10), bool deferred_lock_check = false) :
    blackboard(blackboard),
    locked_buffer_future(),
    locked_buffer(),
    locked_buffer_raw(NULL),
    timeout(timeout)
  {
    ConstructorImplementation(deferred_lock_check);
  }

  /*!
   * \param blackboard Blackboard to access
   * \param timeout Timeout for lock (in ms)
   *
   * \exception tLockException is thrown if lock fails
   */
  tBlackboardReadAccess(tBlackboard<T>& blackboard, const rrlib::time::tDuration& timeout = std::chrono::seconds(10), bool deferred_lock_check = false) :
    blackboard(blackboard.GetClient()),
    locked_buffer_future(),
    locked_buffer(),
    locked_buffer_raw(NULL),
    timeout(timeout)
  {
    ConstructorImplementation(deferred_lock_check);
  }

  ~tBlackboardReadAccess()
  {
    if (locked_buffer)
    {
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Releasing read lock on blackboard '", blackboard.GetName(), "' at ", rrlib::time::Now());
    }
  }

  inline const T& operator[](size_t index)
  {
    return Get(index);
  }

  /*!
   * \param index Element index
   * \return Element at index
   *
   * \exception tLockException is thrown if lock fails (can only occur if locking was deferred in constructor)
   */
  inline const T& Get(size_t index)
  {
    CheckLock();
    if (index >= locked_buffer_raw->size())
    {
      throw std::runtime_error("Blackboard read access out of bounds");
    }
    return (*locked_buffer_raw)[index];
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

  friend class tBlackboardWriteAccess<T>;

  typedef typename tBlackboardClient<T>::tConstBufferPointer tConstBufferPointer;
  typedef typename tBlackboardClient<T>::tBuffer tBuffer;

  /*! Locked blackboard */
  tBlackboardClient<T>& blackboard;

  /*! Future for locked buffer */
  rpc_ports::tFuture<tConstBufferPointer> locked_buffer_future;

  /*! Buffer from locked blackboard */
  tConstBufferPointer locked_buffer;

  /*! Buffer from locked blackboard - as raw pointer (so that it can set from subclass also) */
  const tBuffer* locked_buffer_raw;

  /*! Timeout for buffer lock - stored for deferred locks */
  rrlib::time::tDuration timeout;


  /*! for tBlackboardWriteAccess */
  tBlackboardReadAccess(tBlackboardClient<T>& blackboard, tBlackboardClient<T>& dummy_for_non_ambiguous_overloads) :
    blackboard(blackboard),
    locked_buffer_future(),
    locked_buffer(),
    locked_buffer_raw(NULL)
  {
  }

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
        locked_buffer_raw = locked_buffer.get();
      }
      catch (const rpc_ports::tRPCException& e)
      {
        throw tLockException();
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
      throw tLockException();
    }
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Acquiring read lock on blackboard '", blackboard.GetName(), "' at ", rrlib::time::Now());
    locked_buffer_future = this->blackboard.ReadLock(timeout);
    if (!deferred_lock_check)
    {
      CheckLock();
    }
  }

  inline const char* GetLogDescription()
  {
    return "BlackboardReadAccess";
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
