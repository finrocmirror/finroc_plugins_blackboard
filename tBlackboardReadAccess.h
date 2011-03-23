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

#ifndef plugins__blackboard__tBlackboardReadAccess_h__
#define plugins__blackboard__tBlackboardReadAccess_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "plugins/blackboard/tBBLockException.h"
#include "plugins/blackboard/tBlackboardClient.h"

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * Object to use for read-accessing blackboard
 */
template<typename T>
class tBlackboardReadAccess
{
private:

  /*! Locked blackboard */
  tBlackboardClient<T>& blackboard;

  /*! not null - if buffer is currently locked for writing */
  const typename tBlackboardClient<T>::tBBVectorVar* locked;

public:

  /*!
   * \param blackboard Blackboard to access
   * \param timeout Timeout for lock (in ms)
   */
  tBlackboardReadAccess(tBlackboardClient<T>& blackboard_, int timeout = 60000) :
      blackboard(blackboard_),
      locked(blackboard_.ReadLock(false, timeout))
  {
    if (locked == NULL)
    {
      throw tBBLockException();
    }
  }

  virtual ~tBlackboardReadAccess()
  {
    if (locked != NULL)
    {
      blackboard.Unlock();
    }
  }

  inline const T& operator[](size_t index)
  {
    return Get(index);
  }

  /*!
   * \param index Element index
   * \return Element at index
   */
  inline const T& Get(size_t index)
  {
    assert((index < Size()));

    return (*locked)[index];
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

#endif // plugins__blackboard__tBlackboardReadAccess_h__
