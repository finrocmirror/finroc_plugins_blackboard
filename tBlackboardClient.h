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
/*!\file    plugins/blackboard/tBlackboardClient.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 * \brief   Contains tBlackboardClient
 *
 * \b tBlackboardClient
 *
 * This is a convenience class to create a blackboard client
 * in a group or module.
 *
 * It can also be used in a group to forward internal blackboard ports
 * to the group's interface (to connect the internal clients to the outside).
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__tBlackboardClient_h__
#define __plugins__blackboard__tBlackboardClient_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tProxyPort.h"
#include "plugins/structure/tSenseControlGroup.h"
#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tChange.h"
#include "plugins/blackboard/internal/tBlackboardClientBackend.h"
#include "plugins/blackboard/internal/tBlackboardServer.h"

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

template<typename T>
class tBlackboardReadAccess;
template<typename T>
class tBlackboardWriteAccess;
template<typename T>
class tBlackboard;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Blackboard client
/*!
 * This is a convenience class to create a blackboard client
 * in a group or module.
 *
 * It can also be used in a group to forward internal blackboard ports
 * to the group's interface (to connect the internal clients to the outside).
 */
template <typename T>
class tBlackboardClient : private rrlib::util::tNoncopyable
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef tBlackboardWriteAccess<T> tWriteAccess;
  typedef tBlackboardReadAccess<T> tReadAccess;

  typedef internal::tBlackboardServer<T> tServer;

  typedef std::vector<T> tBuffer;
  typedef data_ports::tPortDataPointer<tBuffer> tBufferPointer;
  typedef data_ports::tPortDataPointer<const tBuffer> tConstBufferPointer;
  typedef typename tServer::tReadLockedBufferPointer tReadLockedBufferPointer;
  typedef std::vector<tChange<T>> tChangeSet;
  typedef data_ports::tPortDataPointer<tChangeSet> tChangeSetPointer;

  /*!
   * Empty constructor for blackboard clients that are not initialized in
   * class initializer list (but later)
   */
  tBlackboardClient() :
    read_port(),
    write_port(),
    backend(NULL),
    outside_write_port1(),
    outside_write_port2(),
    outside_read_port()
  {
  }

  /*!
   * Creates plain blackboard client (without any additional ports in any interfaces)
   *
   * \param name Name/Uid of blackboard
   * \param parent Parent of blackboard client
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param auto_connect_mode Desired mode of auto-connecting
   * \param read_port Create read port?
   */
  tBlackboardClient(const std::string& name, core::tFrameworkElement* parent, bool push_updates = false, bool read_port = true);

  /*!
   * Connects to local blackboard
   *
   * \param server Blackboard server to connect to (in case it is available as object)
   * \param parent Parent of blackboard client
   * \param non_default_name Default name is "<server name> Client". Specifiy non-empty string for other name
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param read_port Create read port?
   */
  tBlackboardClient(internal::tBlackboardServer<T>& server, core::tFrameworkElement* parent,
                    const std::string& non_default_name = "", bool push_updates = false, bool read_port = true);

  /*!
   * Constructor for use with tModule and tSenseControlModule.
   * Does NOT connect to global blackboards - but rather uses group's/module's input/output port groups.
   *
   * (per default, full-blackboard-access-ports are created in Output/ControllerOutput.
   *  If this is not desired, the last two constructor parameters can be used to specify alternatives.)
   *
   * \param name Name of blackboard
   * \param parent Parent of blackboard
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param create_read_port Which read ports to create for blackboard
   * \param create_write_port_in If not NULL, creates write port in specified port group instead of Input/ControllerInput
   * \param create_write_port_in2 If not NULL, creates another write port in specified port group
   */
  tBlackboardClient(const std::string& name, structure::tModuleBase* parent, bool push_updates = false,
                    tReadPorts create_read_port = tReadPorts::EXTERNAL, core::tPortGroup* create_write_port_in = NULL, core::tPortGroup* create_write_port_in2 = NULL);

  /*!
   * Constructor to replicate access to inner tBlackboardClient in tSenseControlGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param parent Parent of blackboard
   * \param create_read_port_in_ci In case we have a plain tModule: Create read port in Controller Input (rather than Sensor Input?)
   * \param forward_write_port_in_controller Forward write ports in controller port groups?
   * \param forward_write_port_in_sensor Forward write ports in sensor port groups?
   */
  tBlackboardClient(const tBlackboardClient& replicated_bb, structure::tSenseControlGroup* parent,
                    bool create_read_port_in_ci = false, bool forward_write_port_in_controller = true, bool forward_write_port_in_sensor = false);

  /*! move constructor */
  tBlackboardClient(tBlackboardClient && o);

  /*! move assignment */
  tBlackboardClient& operator=(tBlackboardClient && o);

  /*!
   * Apply change transaction to blackboard asynchronously.
   * Blackboard does not need to be locked for this operation.
   *
   * If blackboard client is not connected to server, call has no effect.
   *
   * \param change_set Change set to apply
   */
  void AsynchronousChange(tChangeSetPointer& change_set)
  {
    write_port.Call(&tServer::AsynchronousChange, std::move(change_set));
  }

  /*!
   * Connect to outside ports of specified blackboard
   *
   * \param blackboard Blackboard to connect to
   */
  void ConnectTo(const tBlackboard<T>& blackboard);

  /*!
   * Connect to blackboard client in outer group
   *
   * \param blackboard Blackboard client to connect to
   */
  void ConnectTo(const tBlackboardClient<T>& client);

  /*!
   * \return Blackboard name
   */
  inline std::string GetName() const
  {
    return backend->GetName();
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's read port. May not exist.
   */
  data_ports::tPort<tBuffer> GetOutsideReadPort() const
  {
    return outside_read_port;
  }

  /*!
   * \return Revision of blackboard content (is incremented whenever blackboard content changes - signaling that a new version is available)
   * \throws Throws an tRPCException if blackboard is not connected or timeout expired
   */
  uint64_t GetRevision(const rrlib::time::tDuration& timeout = std::chrono::seconds(2))
  {
    return write_port.CallSynchronous(timeout, &tServer::GetRevisionCounter);
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's primary write port
   */
  rpc_ports::tProxyPort<tServer, false> GetOutsideWritePort() const
  {
    return outside_write_port1;
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's secondary write port
   */
  rpc_ports::tProxyPort<tServer, false> GetOutsideWritePort2() const
  {
    return outside_write_port2;
  }

  /*!
   * \return Handle of server that handles calls (can be used to detect whether we're connected to a different blackboard). 0 if not connected to a server.
   */
  typename core::tFrameworkElement::tHandle GetServerHandle()
  {
    return write_port.GetServerHandle();
  }

  /*!
   * \return Unused buffer (may be published/committed directly using Publish())
   */
  inline tBufferPointer GetUnusedBuffer()
  {
    return backend->GetUnusedBuffer<tBuffer>();
  }

  /*!
   * \return Unused change set buffer (may be used in AsynchronousChange())
   */
  inline tChangeSetPointer GetUnusedChangeBuffer()
  {
    return backend->GetUnusedBuffer<tChangeSet>();
  }

  /*!
   * \return Wrapped raw blackboard backend
   */
  inline internal::tBlackboardClientBackend* GetBackend() const
  {
    return backend;
  }

  /*!
   * (only works properly if push_updates in constructor was set to true)
   *
   * \return Has port changed since last changed-flag-reset?
   */
  bool HasChanged() const;

//  /*!
//   * \return Is client currently holding read or write lock?
//   */
//  inline bool HasLock()
//  {
//    return wrapped->HasLock();
//  }
//
//  /*!
//   * \return Is client currently holding read lock?
//   */
//  inline bool HasReadLock()
//  {
//    return wrapped->HasReadLock();
//  }
//
//  /*!
//   * \return Is client currently holding write lock?
//   */
//  inline bool HasWriteLock()
//  {
//    return wrapped->HasWriteLock();
//  }

  /*!
   * Initialize blackboard client
   */
  inline void Init()
  {
    backend->Init();
  }

  /*!
   * Directly commit/publish completely new buffer
   * Does not require lock.
   *
   * \param buffer Buffer to publish
   */
  void Publish(tBufferPointer& buffer)
  {
    write_port.Call(&tServer::DirectCommit, std::move(buffer));
  }

  /*!
   * Reads blackboard contents.
   * This will never block, if a read_port with update pushes was created.
   *
   * Otherwise, it will hardly ever block (apart from calls over the network).
   * (only exception: this is the first read lock performed on a
   * single-buffered blackboard that currently has a write lock)
   *
   * \param timeout Timeout for call
   * \return Current Blackboard Contents
   *
   * \exception rpc_ports::tRPCException is thrown if call fails
   */
  inline tReadLockedBufferPointer Read(const rrlib::time::tDuration& timeout = std::chrono::seconds(2))
  {
    rpc_ports::tFuture<tReadLockedBufferPointer> future = ReadLock(timeout);
    return future.Get(timeout);
  }

  /*!
   * Acquire read "lock" on blackboard.
   *
   * Waiting for future will never block, if a read_port with update pushes was created.
   *
   * Otherwise, waiting will hardly ever block (apart from calls over the network).
   * (only exception: this is the first read lock performed on a
   * single-buffered blackboard that currently has a write lock)
   *
   * \param timeout Timeout for call
   * \return Future on locked buffer
   *
   * \exception rpc_ports::tRPCException is thrown if call fails
   */
  rpc_ports::tFuture<tReadLockedBufferPointer> ReadLock(const rrlib::time::tDuration& timeout = std::chrono::seconds(10));

  /*!
   * (only works properly if pushUpdates in constructor was set to true)
   *
   * Reset changed flag.
   */
  void ResetChanged()
  {
    if (read_port.GetWrapped())
    {
      read_port.ResetChanged();
    }
  }

  /*!
   * Acquire write lock on blackboard.
   *
   * This will block if another client has a write lock on this blackboard.
   *
   * \param timeout Timeout for call
   * \return Future on locked buffer
   *
   * \exception rpc_ports::tRPCException is thrown if call fails
   */
  rpc_ports::tFuture<internal::tLockedBuffer<tBuffer>> WriteLock(const rrlib::time::tDuration& timeout = std::chrono::seconds(10))
  {
    return write_port.NativeFutureCall(&tServer::WriteLock, internal::tLockParameters(timeout));
  }


  operator bool() const
  {
    return backend != NULL;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Port for reading - in case pushing of updates is activated */
  data_ports::tInputPort<tBuffer> read_port;

  /*! Port for full blackboard access */
  rpc_ports::tClientPort<tServer> write_port;

  /*! Wrapped blackboard backend */
  internal::tBlackboardClientBackend* backend;

  /*! Replicated ports in group's/module's tPortGroups */
  rpc_ports::tProxyPort<tServer, false> outside_write_port1, outside_write_port2;

  /*! Replicated read port in group's/module's tPortGroups */
  data_ports::tPort<tBuffer> outside_read_port;


  /*!
   * Check whether these ports can be connected - if yes, do so
   * (connecting to blackboard)
   */
  void CheckConnect(core::tPortWrapperBase p1, core::tPortWrapperBase p2);

  /*!
   * Check whether these ports can be connected - if yes, do so
   * (connecting to outer blackboard client)
   */
  void CheckClientConnect(core::tPortWrapperBase p1, core::tPortWrapperBase p2);

  /*!
   * \return Log description
   */
  inline const core::tFrameworkElement& GetLogDescription() const
  {
    return *backend;
  }

  /*!
   * (Helper to make constructors shorter)
   * Creates read port
   *
   * \param create Actually create port?
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port)
   * \return Created read port
   */
  static data_ports::tInputPort<tBuffer> PossiblyCreateReadPort(bool create, bool push_updates)
  {
    if (create)
    {
      data_ports::tInputPort<tBuffer> read_port = data_ports::tInputPort<tBuffer>("read");
      if (!push_updates)
      {
        read_port.SetPushStrategy(false);
      }
      return read_port;
    }
    return data_ports::tInputPort<tBuffer>();
  }

  /*!
   * Create blackboard write port in specified port group and connect it to
   * (original blackboard client) write port.
   *
   * \param write_port Port to connect newly created port to
   * \param port_group Port group to create port in
   * \param name Name of new port
   * \return Created Port
   */
  rpc_ports::tProxyPort<tServer, false> ReplicateWritePort(core::tPortWrapperBase& write_port, core::tFrameworkElement* port_group, const std::string& name)
  {
    typename core::tFrameworkElement::tFlags extra_flags;
    if (typeid(*port_group) == typeid(core::tPortGroup))
    {
      extra_flags |= static_cast<core::tPortGroup&>(*port_group).GetDefaultPortFlags();
    }
    rpc_ports::tProxyPort<tServer, false> new_port(name, port_group, extra_flags);
    write_port.ConnectTo(new_port);
    return new_port;
  }
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/blackboard/tBlackboardClient.hpp"

#endif
