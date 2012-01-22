/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2011 Max Reichardt,
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

#ifndef plugins__blackboard__tBlackboardWriteAccess_h__
#define plugins__blackboard__tBlackboardWriteAccess_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "plugins/blackboard/tBBLockException.h"
#include "rrlib/serialization/sSerialization.h"
#include "plugins/blackboard/tBlackboardClient.h"
#include "rrlib/finroc_core_utils/tTime.h"

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * Object to use for write-accessing blackboard
 *
 * Derived from tBlackboardReadAccess so that it can also be used in places
 * where only read access is required.
 */
template<typename T>
class tBlackboardWriteAccess : public tBlackboardReadAccess<T>
{
private:

  // no heap allocation permitted
  void *operator new(size_t);
  void *operator new[](size_t);

  /*! not null - if buffer is currently locked for writing */
  typename tBlackboardClient<T>::tBBVector* locked;

  using tBlackboardReadAccess<T>::blackboard;

  inline typename tBlackboardClient<T>::tBBVector* WriteLock(int timeout = 60000)
  {
    FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_1, "Acquiring write lock on blackboard '", blackboard.GetDescription(), "' at ", util::tTime::GetPrecise());
    return blackboard.WriteLock(timeout);
  }

public:

  /*!
   * \param blackboard Blackboard to access
   * \param timeout Timeout for lock (in ms)
   */
  tBlackboardWriteAccess(tBlackboardClient<T>& blackboard, int timeout = 60000) :
    tBlackboardReadAccess<T>(blackboard, blackboard),
    locked(WriteLock(timeout))
  {
    if (locked == NULL)
    {
      throw tBBLockException();
    }
    tBlackboardReadAccess<T>::locked = locked;
  }

  /*!
   * \param blackboard Blackboard to access
   * \param timeout Timeout for lock (in ms)
   */
  tBlackboardWriteAccess(tBlackboard<T>& blackboard, int timeout = 60000) :
    tBlackboardReadAccess<T>(blackboard.GetClient(), blackboard.GetClient()),
    locked(WriteLock(timeout))
  {
    if (locked == NULL)
    {
      throw tBBLockException();
    }
    tBlackboardReadAccess<T>::locked = locked;
  }

  virtual ~tBlackboardWriteAccess()
  {
    tBlackboardReadAccess<T>::locked = NULL;
    if (locked != NULL)
    {
      FINROC_LOG_PRINT(rrlib::logging::eLL_DEBUG_VERBOSE_1, "Releasing write lock on blackboard '", blackboard.GetDescription(), "' at ", util::tTime::GetPrecise());
      blackboard.Unlock();
    }
  }

  inline T& operator[](size_t index)
  {
    return Get(index);
  }

  /*!
   * \param index Element index
   * \return Element at index
   */
  inline T& Get(size_t index)
  {
    assert((index < Size()));

    return (*locked)[index];
  }

  /*!
   * \param new_size New size (number of elements) in blackboard
   */
  inline void Resize(size_t new_size)
  {
    if (new_size != Size())
    {
      rrlib::serialization::sSerialization::ResizeVector(*locked, new_size);
    }
  }

  /*!
   * \return Number of elements in blackboard
   */
  inline size_t Size()
  {
    return locked->size();
  }

};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardWriteAccess_h__
