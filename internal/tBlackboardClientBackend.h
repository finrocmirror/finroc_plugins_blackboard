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
/*!\file    plugins/blackboard/internal/tBlackboardClientBackend.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-31
 *
 * \brief   Contains tBlackboardClientBackend
 *
 * \b tBlackboardClientBackend
 *
 * Blackboard client backend.
 * Framework element with the two ports - wrapped by by tBlackboardClient<T> class.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tBlackboardClientBackend_h__
#define __plugins__blackboard__internal__tBlackboardClientBackend_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/data_ports/tPort.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/definitions.h"

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
//! Blackboard client backend
/*!
 * Blackboard client backend.
 * Framework element with the two ports - wrapped by by tBlackboardClient<T> class.
 */
class tBlackboardClientBackend : public core::tFrameworkElement
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param name Name/Uid of blackboard
   * \param parent Parent of blackboard client
   * \param write_port Write port
   * \param read_port Read port
   */
  tBlackboardClientBackend(const std::string& name, core::tFrameworkElement* parent, core::tPortWrapperBase& write_port, core::tPortWrapperBase& read_port);

  /*!
   * \return Unused buffer of specified type (typically for asynchronous changes and direct buffer commits)
   */
  template <typename T>
  data_ports::tPortDataPointer<T> GetUnusedBuffer()
  {
    auto pointer = buffer_pool.GetUnusedBuffer(rrlib::rtti::tDataType<T>());
    return data_ports::api::tPortDataPointerImplementation<T, false>(pointer);
  }

  void SetAutoConnectMode(tAutoConnectMode new_mode)
  {
    // TODO
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Buffer pool for asynchronous changes and direct buffer commits */
  data_ports::standard::tMultiTypePortBufferPool buffer_pool;

  /*! Read and write port */
  core::tAbstractPort * read_port, * write_port;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
