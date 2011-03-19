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

#ifndef plugins__blackboard__tBlackboardBuffer_h__
#define plugins__blackboard__tBlackboardBuffer_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "rrlib/serialization/tMemoryBuffer.h"

namespace rrlib
{
namespace serialization
{
class tInputStream;
} // namespace rrlib
} // namespace serialization

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * Buffer containing blackboard data - in MCA style
 */
class tBlackboardBuffer : public rrlib::serialization::tMemoryBuffer
{
public:

  //    /** Data type of this class */
  //    @Const public final static DataTypeBase TYPE = DataTypeRegister.;

  //    /** Lock ID of buffer - against outdated unlocks */
  //    int lockID;

  /*! Number of entries and entry size */
  int bb_capacity, elements, element_size;

  tBlackboardBuffer() :
      bb_capacity(0),
      elements(0),
      element_size(0)
  {}

  tBlackboardBuffer(tBlackboardBuffer && o) :
      rrlib::serialization::tMemoryBuffer(std::forward<rrlib::serialization::tMemoryBuffer>(o)),
      bb_capacity(0),
      elements(0),
      element_size(0)
  {
    std::swap(bb_capacity, o.bb_capacity);
    std::swap(elements, o.elements);
    std::swap(element_size, o.element_size);
  }

  tBlackboardBuffer& operator=(tBlackboardBuffer && o)
  {
    rrlib::serialization::tMemoryBuffer::operator=(std::forward<rrlib::serialization::tMemoryBuffer>(o));
    std::swap(bb_capacity, o.bb_capacity);
    std::swap(elements, o.elements);
    std::swap(element_size, o.element_size);
    return *this;
  }

  virtual void Deserialize(rrlib::serialization::tInputStream& is);

  void* GetElementPointer(int index)
  {
    assert(index < elements);
    return GetBufferPointer(index * element_size);
  }

  const void* GetElementPointer(int index) const
  {
    assert(index < elements);
    return GetBufferPointer(index * element_size);
  }

  /*!
   * \return Number of elements that fit in blackboard
   */
  inline int GetBbCapacity() const
  {
    return bb_capacity;
  }

  /*!
   * Offset of element in buffer (in bytes)
   *
   * \param index Index of element
   * \return Offset
   */
  inline int GetElementOffset(int index)
  {
    return index * element_size;
  }

  /*!
   * \return Element size
   */
  inline int GetElementSize() const
  {
    return element_size;
  }

  /*!
   * \return Number of elements in blackboard
   */
  inline int GetElements() const
  {
    return elements;
  }

  /*!
   * Resize blackboard
   *
   * \param new_capacity new Capacity
   * \param new_elements new current number of elements
   * \param new_element_size new size of elements
   * \param keep_contents Keep current data - otherwise buffer will be nulled or random
   */
  void Resize(int new_capacity, int new_elements, int new_element_size, bool keep_contents);

  virtual void Serialize(rrlib::serialization::tOutputStream& os) const;

};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardBuffer_h__
