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
#include "plugins/blackboard/internal/tBlackboardBase.h"
#include "plugins/blackboard/tChange.h"
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
  typedef std::vector<tChange<T>> tChangeSet;
  typedef data_ports::tPortDataPointer<tChangeSet> tChangeSetPointer;
  typedef typename data_ports::standard::tStandardPort::tBufferPool tChangeSetBufferPool;

  typedef core::tFrameworkElement::tFlag tFlag;
  typedef core::tFrameworkElement::tFlag tFlags;
  typedef data_ports::tInputPort<tBuffer> tReadPort;
  typedef rpc_ports::tClientPort<tServer> tWritePort;


  /*!
   * Empty constructor for blackboard clients that are not initialized in
   * class initializer list (but later)
   */
  tBlackboardClient() :
    read_port(),
    write_port(),
    change_set_buffer_pool()
  {
  }

  /*!
   * Creates blackboard client
   *
   * \param name Name/Uid of blackboard (will be name of write port)
   * \param parent Parent component of blackboard client
   * \param push_updates Use push strategy for read port? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param create_read_port_in If not nullptr, creates data port for reading blackboard in specified component interface (possibly relevant for data dependencies -> scheduling order)
   * \param read_port_name Name for read port. If empty, blackboard name will be used.
   */
  template <typename TParent>
  tBlackboardClient(const std::string& name, TParent* parent, bool push_updates = false, tInterface* create_read_port_in = UseDefaultComponentInterface(), const std::string& read_port_name = "") :
    tBlackboardClient(push_updates, parent->GetServices(), name, create_read_port_in == UseDefaultComponentInterface() ? GetDefaultReadPortInterface(parent, nullptr) : create_read_port_in, read_port_name.length() ? read_port_name : name)
  {
  }

  /*!
   * Constructor to replicate access to inner tBlackboardClient in tSenseControlGroup.
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
  tBlackboardClient(const tBlackboardClient& replicated_bb, TParent* parent, tInterface* create_read_port_in = UseDefaultComponentInterface(),
                    const std::string& blackboard_name = "", const std::string& read_port_name = "") :
    read_port(),
    write_port(),
    change_set_buffer_pool()
  {
    // hack to get rpc port that is actually a proxy into a client port
    static_cast<core::tPortWrapperBase&>(write_port) = core::tPortWrapperBase(parent->GetServices().CreatePort(blackboard_name.length() ? blackboard_name : replicated_bb.write_port.GetName(), replicated_bb.write_port.GetDataType(),
        core::tFrameworkElement::tFlag::ACCEPTS_DATA | core::tFrameworkElement::tFlag::EMITS_DATA | (replicated_bb.write_port.GetFlag(tFlag::OUTPUT_PORT) ? tFlag::OUTPUT_PORT : tFlag::PORT)));
    write_port.ConnectTo(replicated_bb.write_port);
    if (replicated_bb.read_port.GetWrapped() && create_read_port_in)
    {
      create_read_port_in = create_read_port_in == UseDefaultComponentInterface() ? GetDefaultReadPortInterface(parent, replicated_bb.read_port.GetWrapped()) : create_read_port_in;
      read_port = tReadPort(read_port_name.length() ? read_port_name : replicated_bb.read_port.GetName(), create_read_port_in, create_read_port_in->GetDefaultPortFlags() | tFlag::EMITS_DATA | tFlag::ACCEPTS_DATA | tFlag::PUSH_STRATEGY);
      replicated_bb.read_port.GetWrapped()->ConnectTo(*read_port.GetWrapped());
    }
  }

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
    return write_port.GetName();
  }

  /*!
   * \return Port for reading - in case pushing of updates is activated
   */
  tReadPort GetReadPort() const
  {
    return read_port;
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
   * \return Handle of server that handles calls (can be used to detect whether we're connected to a different blackboard). 0 if not connected to a server.
   */
  typename core::tFrameworkElement::tHandle GetServerHandle()
  {
    return write_port.GetServerHandle();
  }

  /*!
   * \return Port for full blackboard access
   */
  tWritePort GetWritePort() const
  {
    return write_port;
  }

  /*!
   * \return Unused buffer (may be published/committed directly using Publish())
   */
  inline tBufferPointer GetUnusedBuffer()
  {
    return read_port.GetUnusedBuffer();
  }

  /*!
   * \return Unused change set buffer (may be used in AsynchronousChange())
   */
  inline tChangeSetPointer GetUnusedChangeBuffer()
  {
    auto buffer = change_set_buffer_pool->GetUnusedBuffer(rrlib::rtti::tDataType<tChangeSet>());
    buffer->SetUnused(true);
    return data_ports::api::tPortDataPointerImplementation<tChangeSet, false>(buffer);
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
    write_port.Init();
    if (read_port.GetWrapped())
    {
      read_port.Init();
    }
  }

  /*!
   * Deletes this blackboard (its server/client/ports in particular). Object is empty afterwards (equals to a tBlackboard()).
   */
  void ManagedDelete()
  {
    read_port.ManagedDelete();
    write_port.ManagedDelete();
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
  inline tConstBufferPointer Read(const rrlib::time::tDuration& timeout = std::chrono::seconds(2))
  {
    if (read_port && read_port.GetFlag(core::tFrameworkElement::tFlag::PUSH_STRATEGY))
    {
      return read_port.GetPointer();
    }
    rpc_ports::tFuture<tConstBufferPointer> future = ReadLock(timeout);
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
  rpc_ports::tFuture<tConstBufferPointer> ReadLock(const rrlib::time::tDuration& timeout = std::chrono::seconds(10));

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

  /*! \return Returns constant that indicates that default component interface for read port should be used */
  static tInterface* UseDefaultComponentInterface()
  {
    return internal::tBlackboardBase::UseDefaultComponentInterface();
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
    return change_set_buffer_pool.get();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tBlackboard<T>;

  /*! Port for reading - in case pushing of updates is activated */
  tReadPort read_port;

  /*! Port for full blackboard access */
  tWritePort write_port;

  /*! Additional port buffer pool */
  std::unique_ptr<tChangeSetBufferPool> change_set_buffer_pool;


  /*!
   * Creates blackboard client
   *
   * \param push_updates Use push strategy for read port? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param create_write_port_in Interface to create RPC for blackboard access in
   * \param write_port_name Name for write port
   * \param create_read_port_in If not nullptr, creates data port for reading blackboard in specified component interface (possibly relevant for data dependencies -> scheduling order)
   * \param read_port_name Name for read port. If empty, blackboard name will be used.
   */
  tBlackboardClient(bool push_updates, tInterface& create_write_port_in, const std::string& write_port_name, tInterface* create_read_port_in, const std::string& read_port_name);

  /*!
   * \return Log description
   */
  inline const core::tFrameworkElement& GetLogDescription() const
  {
    return *write_port.GetWrapped();
  }

  /*! Helper function to determine component interface to create read ports in by default */
  static tInterface* GetDefaultReadPortInterface(structure::tSenseControlGroup* g, core::tAbstractPort* replicated_port)
  {
    if (replicated_port && replicated_port->GetParent()->GetFlag(tFlag::CONTROLLER_DATA))
    {
      return &g->GetControllerInputs();
    }
    return &g->GetSensorInputs();
  }
  static tInterface* GetDefaultReadPortInterface(structure::tSenseControlModule* g, core::tAbstractPort* replicated_port)
  {
    if (replicated_port && replicated_port->GetParent()->GetFlag(tFlag::CONTROLLER_DATA))
    {
      return &g->GetControllerInputs();
    }
    return &g->GetSensorInputs();
  }
  static tInterface* GetDefaultReadPortInterface(structure::tGroup* g, core::tAbstractPort* replicated_port)
  {
    return &g->GetInputs();
  }
  static tInterface* GetDefaultReadPortInterface(structure::tModule* g, core::tAbstractPort* replicated_port)
  {
    return &g->GetInputs();
  }
  static tInterface* GetDefaultReadPortInterface(core::tFrameworkElement* g, core::tAbstractPort* replicated_port)
  {
    return nullptr;
  }
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/blackboard/tBlackboardClient.hpp"

#endif
