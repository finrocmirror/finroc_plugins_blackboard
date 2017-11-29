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
/*!\file    plugins/blackboard/tBlackboard.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 * \brief   Contains tBlackboard
 *
 * \b tBlackboard
 *
 * This is a convenience class to create a blackboard server
 * (and possibly client) in a (composite) component.
 *
 * Read ports in data port interface allow considering blackboard edges in scheduling.
 *
 * This class can furthermore be used in a composite component to allow access to a blackboard
 * of an inner component.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__tBlackboard_h__
#define __plugins__blackboard__tBlackboard_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboardClient.h"
#include "plugins/blackboard/internal/tBlackboardBase.h"

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
//! Blackboard (server)
/*!
 * This is a convenience class to create a blackboard server
 * (and possibly client) in a (composite) component.
 *
 * Read ports in data port interface allow considering blackboard edges in scheduling.
 *
 * This class can furthermore be used in a composite component to allow access to a blackboard
 * of an inner component.
 */
template <typename T>
class tBlackboard : public internal::tBlackboardBase
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef tBlackboardWriteAccess<T> tWriteAccess;
  typedef tBlackboardReadAccess<T> tReadAccess;
  typedef internal::tBlackboardServer<T> tServer;
  typedef typename tServer::tBuffer tBuffer;
  typedef typename tServer::tReadPort tReadPort;

  struct tBlackboardBufferModeParameter
  {
    tBlackboardBufferModeParameter(tBlackboardBufferMode buffer_mode) : buffer_mode(buffer_mode) {}

    // Constructor for legacy compatibility
    tBlackboardBufferModeParameter(bool multi_buffered) : buffer_mode(multi_buffered ? tBlackboardBufferMode::MULTI_BUFFERED : tBlackboardBufferMode::MULTI_BUFFERED_ON_PARALLEL_ACCESS) {}

    tBlackboardBufferMode buffer_mode;
  };


  /*!
   * Empty constructor for blackboards that are not initialized in
   * class initializer list (but later)
   */
  tBlackboard() :
    wrapped_server(nullptr),
    wrapped_client(),
    read_port()
  {}

  /*!
   * Constructor for creating a new blackboard in a component.
   *
   * \param name Name of blackboard
   * \param parent Component that contains blackboard
   * \param buffer_mode Buffer mode - whether to use multiple buffers to avoid blocking (at the cost of copying content)
   * \param elements Initial number of elements
   * \param create_client Create Blackboard client? (to access blackboard using this object's GetClient())
   * \param create_read_port_in If not nullptr, creates data port for reading blackboard in specified component interface (possibly relevant for data dependencies -> scheduling order)
   * \param read_port_name Name for read port. If empty, blackboard name will be used.
   * \param create_read_port_in_client Create a read port in the client? This is only relevant when 'create_client' is true and a read port is created. Use 'true', if you wish 'read locks' to occur via the data port. (This parameter exists for minor performance tweaking.)
   */
  template <typename TParent>
  tBlackboard(const std::string& name, TParent* parent, const tBlackboardBufferModeParameter& buffer_mode = tBlackboardBufferMode::MULTI_BUFFERED_ON_PARALLEL_ACCESS, int elements = 0, bool create_client = true,
              tInterface* create_read_port_in = UseDefaultComponentInterface(), const std::string& read_port_name = "", bool create_read_port_in_client = true) :
    wrapped_server(nullptr),
    wrapped_client(),
    read_port()
  {
    create_read_port_in = buffer_mode.buffer_mode == tBlackboardBufferMode::SINGLE_BUFFERED ? nullptr : create_read_port_in;

    // Create blackboard server
    tInterface& blackboards_parent = GetBlackboardsParent(*parent);
    create_read_port_in = create_read_port_in == UseDefaultComponentInterface() ? GetDefaultReadPortInterface(parent, nullptr) : create_read_port_in;
    wrapped_server = new internal::tBlackboardServer<T>(name, blackboards_parent, buffer_mode.buffer_mode, elements, parent->GetServices(), create_read_port_in, read_port_name);
    write_port = wrapped_server->GetWritePort();
    if (create_read_port_in && create_read_port_in_client)
    {
      read_port = wrapped_server->GetReadPort();
    }

    // Create blackboard client
    if (create_client)
    {
      wrapped_client = tBlackboardClient<T>(false, blackboards_parent, name + " Internal Write", (create_read_port_in_client && create_read_port_in) ? (&blackboards_parent) : nullptr, name + " Internal Read");
      wrapped_client.ConnectTo(*this);
    }
  }

  /*!
   * Constructor to replicate access to inner tBlackboard in tGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param parent Composite component that replicates blackboard
   * \param create_read_port_in If not nullptr and blackboard has a read port, creates data port for reading blackboard in specified component interface (possibly relevant for data dependencies -> scheduling order)
   * \param blackboard_name Name for blackboard (== name for write port). If empty, replicated blackboard's name will be used.
   * \param read_port_name Name for read port. If empty, replicated blackboard's read port name will be used.
   */
  template <typename TParent>
  tBlackboard(tBlackboard& replicated_bb, TParent* parent, tInterface* create_read_port_in = UseDefaultComponentInterface(), const std::string& blackboard_name = "", const std::string& read_port_name = "") :
    tBlackboardBase(replicated_bb, parent->GetServices(), blackboard_name),
    wrapped_server(nullptr),
    wrapped_client(),
    read_port()
  {
    // forward read port
    if (replicated_bb.read_port.GetWrapped() && create_read_port_in)
    {
      create_read_port_in = create_read_port_in == UseDefaultComponentInterface() ? GetDefaultReadPortInterface(parent, replicated_bb.GetReadPort().GetWrapped()) : create_read_port_in;
      read_port = tReadPort(read_port_name.length() ? read_port_name : replicated_bb.read_port.GetName(), create_read_port_in, create_read_port_in->GetDefaultPortFlags() | tFlag::EMITS_DATA | tFlag::ACCEPTS_DATA | tFlag::PUSH_STRATEGY | tFlag::OUTPUT_PORT);
      replicated_bb.read_port.ConnectTo(read_port);
    }
  }

  /*! move constructor */
  tBlackboard(tBlackboard && o) :
    tBlackboardBase(std::forward(o)),
    wrapped_server(nullptr),
    wrapped_client(),
    read_port()
  {
    std::swap(wrapped_server, o.wrapped_server);
    std::swap(wrapped_client, o.wrapped_client);
    std::swap(read_port, o.read_port);
  }

  /*! move assignment */
  tBlackboard& operator=(tBlackboard && o)
  {
    tBlackboardBase::operator=(std::forward<tBlackboardBase>(o));
    std::swap(wrapped_server, o.wrapped_server);
    std::swap(wrapped_client, o.wrapped_client);
    std::swap(read_port, o.read_port);
    return *this;
  }

  /*!
   * \return Blackboard's current buffer mode
   */
  tBlackboardBufferMode GetBufferMode() const
  {
    return wrapped_server ? wrapped_server->GetBufferMode() : tBlackboardBufferMode::NONE;
  }

  /*!
   * \return Wrapped blackboard client - contains NULL pointer if no such client was created
   */
  tBlackboardClient<T>& GetClient()
  {
    return wrapped_client;
  }

  /*! same as tFrameworkElement::GetName() */
  const std::string& GetName() const
  {
    return write_port.GetName();
  }

  /*!
   * \return Port to use, when modules inside group containing blackboard want to connect to this blackboard's read port
   */
  tReadPort GetReadPort() const
  {
    return read_port;
  }
  tReadPort GetOutsideReadPort() const __attribute__((deprecated("USE GetReadPort() instead")))
  {
    return read_port;
  }

  /*!
   * \return Revision of blackboard content (is incremented whenever blackboard content changes - signaling that a new version is available)
   */
  uint64_t GetRevision()
  {
    return wrapped_server->GetRevisionCounter();
  }

  /*!
   * \return Port to use, when modules inside group containing blackboard want to connect to this blackboard's primary write port
   */
  core::tPortWrapperBase GetWritePort() const
  {
    return write_port;
  }

  /*!
   * Deletes this blackboard (its server/client/ports in particular). Object is empty afterwards (equals to a tBlackboard()).
   */
  void ManagedDelete()
  {
    if (wrapped_server)
    {
      wrapped_server->ManagedDelete();
    }
    wrapped_client.ManagedDelete();
    read_port.ManagedDelete();
    write_port.ManagedDelete();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Wrapped blackboard server */
  internal::tBlackboardServer<T>* wrapped_server;

  /*! Wrapped blackboard client - contains NULL pointer if no such client was created */
  tBlackboardClient<T> wrapped_client;

  /*! Read Port in component's interface */
  tReadPort read_port;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
