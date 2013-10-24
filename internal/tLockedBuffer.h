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
/*!\file    plugins/blackboard/internal/tLockedBuffer.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-19
 *
 * \brief   Contains tLockedBuffer
 *
 * \b tLockedBuffer
 *
 * Type returned by blackboard's lock methods.
 * It contains a pointer to the current buffer - and is derived
 * from tPromise<tBuffer>, which ensures/promises that the blackboard
 * will be unlocked.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tLockedBuffer_h__
#define __plugins__blackboard__internal__tLockedBuffer_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tPromise.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/internal/tLockedBufferData.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Locked blackboard buffer
/*!
 * Type returned by blackboard's lock methods.
 * It contains a pointer to the current buffer - and is derived
 * from tPromise<tBuffer>, which ensures/promises that the blackboard
 * will be unlocked.
 *
 * \tparam T Type of locked buffer (typically tAbstractBlackboardServer<U>::tBuffer)
 */
template <typename T>
class tLockedBuffer : public rpc_ports::tPromise<tLockedBufferData<T>>
{
  typedef rpc_ports::tPromise<tLockedBufferData<T>> tBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename ... TArgs>
  tLockedBuffer(TArgs && ... args) :
    data(std::forward<TArgs>(args)...),
    buffer_source()
  {
  }

  tLockedBuffer(tLockedBuffer && other) :
    tBase(std::forward<tBase>(other)),
    data(),
    buffer_source()
  {
    std::swap(data, other.data);
    std::swap(buffer_source, other.buffer_source);
  }

  tLockedBuffer& operator=(tLockedBuffer && other)
  {
    tBase::operator=(std::forward<tBase>(other));
    std::swap(data, other.data);
    std::swap(buffer_source, other.buffer_source);
    return *this;
  }

  /*!
   * Commits current buffer back to blackboard server
   * and releases write lock
   */
  void CommitCurrentBuffer()
  {
    this->SetValue(tLockedBufferData<T>(std::move(data.buffer), data.lock_id));
  }

  /*!
   * Sends no changes back to blackboard server
   * and releases write lock
   */
  void CommitNoChanges()
  {
    this->SetValue(tLockedBufferData<T>(data_ports::tPortDataPointer<T>(), data.lock_id));
  }

  /*!
   * Get non-const version of locked buffer.
   * If only a const-buffer was provided, it is copied to a newly obtained buffer
   */
  T* Get()
  {
    if (!data.buffer)
    {
      // Make copy
      assert(buffer_source.GetWrapped());
      data.buffer = buffer_source.GetUnusedBuffer();
      rrlib::rtti::sStaticTypeInfo<T>::DeepCopy(*data.const_buffer, *data.buffer, NULL);
    }
    return data.buffer.get();
  }

  /*!
   * Get const version of locked buffer.
   */
  const T* GetConst()
  {
    return data.const_buffer ? data.const_buffer.get() : data.buffer.get();
  }

  /*!
   * \param buffer_source Buffer source - set if only const-buffer was provided on write lock
   */
  void SetBufferSource(data_ports::tOutputPort<T> buffer_source)
  {
    this->buffer_source = buffer_source;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  template <typename U>
  friend rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tLockedBuffer<U>& buffer);

  template <typename U>
  friend rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tLockedBuffer<U>& buffer);

  /*! Locked buffer data */
  tLockedBufferData<T> data;

  /*! Buffer source - set if only const-buffer was provided on write lock */
  data_ports::tOutputPort<T> buffer_source;
};


template <typename T>
rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tLockedBuffer<T>& buffer)
{
  stream << buffer.data;
  return stream;
}

template <typename T>
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tLockedBuffer<T>& buffer)
{
  stream >> buffer.data;
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
