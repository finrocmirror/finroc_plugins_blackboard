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
/*!\file    plugins/blackboard/internal/tLockedBufferData.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-30
 *
 * \brief   Contains tLockedBufferData
 *
 * \b tLockedBufferData
 *
 * Information on locked buffer.
 * Used by tLockedBuffer class.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tLockedBufferData_h__
#define __plugins__blackboard__internal__tLockedBufferData_h__

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
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
template <typename T>
class tBlackboardServer;

template <typename T>
class tLockedBuffer;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Information on locked buffer
/*!
 * Information on locked buffer.
 * Used by tLockedBuffer class.
 *
 * \tparam T Type of locked buffer (typically tAbstractBlackboardServer<U>::tBuffer)
 */
template <typename T>
class tLockedBufferData : public boost::noncopyable
{

public:

  tLockedBufferData() :
    const_buffer(),
    buffer(),
    lock_id(0)
  {}

  tLockedBufferData(data_ports::tPortDataPointer<const T> && const_buffer, uint64_t lock_id) :
    const_buffer(std::move(const_buffer)),
    buffer(),
    lock_id(lock_id)
  {}

  tLockedBufferData(data_ports::tPortDataPointer<T> && buffer, uint64_t lock_id) :
    const_buffer(),
    buffer(std::move(buffer)),
    lock_id(lock_id)
  {}

  tLockedBufferData(tLockedBufferData && other) :
    const_buffer(),
    buffer(),
    lock_id(0)
  {
    std::swap(const_buffer, other.const_buffer);
    std::swap(buffer, other.buffer);
    std::swap(lock_id, other.lock_id);
  }

  tLockedBufferData& operator=(tLockedBufferData && other)
  {
    std::swap(const_buffer, other.const_buffer);
    std::swap(buffer, other.buffer);
    std::swap(lock_id, other.lock_id);
    return *this;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  template <typename U>
  friend rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tLockedBufferData<U>& buffer);

  template <typename U>
  friend rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tLockedBufferData<U>& buffer);

  template <typename U>
  friend class tBlackboardServer;

  friend class tLockedBuffer<T>;

  /*! Locked buffer - in case blackboard's current buffer is accessed read-only */
  data_ports::tPortDataPointer<const T> const_buffer;

  /*! Locked buffer - in case blackboard's current buffer is accessed exclusively */
  data_ports::tPortDataPointer<T> buffer;

  /*! Lock id - to avoid obsolete unlocks - if < 0, locked_buffer is a copy */
  uint64_t lock_id;

};

template <typename T>
rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tLockedBufferData<T>& buffer)
{
  stream << buffer.lock_id;
  bool buffer_set = buffer.const_buffer || buffer.buffer;
  stream << buffer_set;
  if (buffer.buffer)
  {
    stream << buffer.buffer;
  }
  else if (buffer.const_buffer)
  {
    stream << buffer.const_buffer;
  }
  return stream;
}

template <typename T>
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tLockedBufferData<T>& buffer)
{
  stream >> buffer.lock_id;
  bool buffer_set;
  stream >> buffer_set;
  buffer.const_buffer.Reset();
  if (buffer_set)
  {
    stream >> buffer.buffer;
  }
  else
  {
    buffer.buffer.Reset();
  }
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
