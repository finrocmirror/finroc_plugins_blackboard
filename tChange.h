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
/*!\file    plugins/blackboard/tChange.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-26
 *
 * \brief   Contains tChange
 *
 * \b tChange
 *
 * Change to blackboard element.
 * Such changes can be packed as change sets to perform atomic
 * changes to blackboards asynchronously.
 * The template may be specialized for certain blackboard content types.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__tChange_h__
#define __plugins__blackboard__tChange_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
//! Change to blackboard
/*!
 * Change to blackboard element.
 * Such changes can be packed as change sets to perform atomic
 * changes to blackboards asynchronously.
 * The template may be specialized for certain blackboard content types.
 */
template <typename T>
class tChange : public rrlib::util::tNoncopyable
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tChange() : index(-1), new_element(rrlib::serialization::DefaultInstantiation<T>::Create()) {}

  /*!
   * \param index Index of element in blackboard to change
   * \param new_element New element to place at this index
   */
  tChange(size_t index, T new_element) :
    index(index),
    new_element(new_element)
  {}

  /*! move constructor */
  tChange(tChange && other) : index(-1), new_element(rrlib::serialization::DefaultInstantiation<T>::Create())
  {
    std::swap(index, other.index);
    std::swap(new_element, other.new_element);
  }

  /*! move assignment */
  tChange& operator=(tChange && other)
  {
    std::swap(index, other.index);
    std::swap(new_element, other.new_element);
    return *this;
  }

  /*!
   * Applies change to blackboard buffer
   *
   * \param blackboard_buffer Blackboard buffer to apply change to
   */
  void Apply(std::vector<T>& blackboard_buffer)
  {
    if (index > static_cast<int>(blackboard_buffer.size()))
    {
      FINROC_LOG_PRINTF_STATIC(WARNING, "Blackboard change has index (%d) out of bounds (%d). Ignoring.", index, static_cast<int>(blackboard_buffer.size()));
    }
    else if (index >= 0)
    {
      std::swap(blackboard_buffer[index], new_element);
    }
  }

  void Deserialize(rrlib::serialization::tInputStream& stream)
  {
    index = stream.ReadInt();
    stream >> new_element;
  }

  void Serialize(rrlib::serialization::tOutputStream& stream) const
  {
    stream.WriteInt(index);
    stream << new_element;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Index of element in blackboard to change */
  int index;

  /*! New element to place at this index */
  T new_element;
};

template <typename T>
rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tChange<T>& change)
{
  change.Serialize(stream);
  return stream;
}

template <typename T>
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tChange<T>& change)
{
  change.Deserialize(stream);
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
