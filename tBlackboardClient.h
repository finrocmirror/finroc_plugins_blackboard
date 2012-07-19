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

#ifndef plugins__blackboard__tBlackboardClient_h__
#define plugins__blackboard__tBlackboardClient_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/rtti/tDataTypeBase.h"

#include "core/structure/tGroup.h"
#include "core/structure/tModule.h"
#include "core/structure/tSenseControlModule.h"
#include "core/port/tPortUtil.h"

#include "plugins/blackboard/tRawBlackboardClient.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
class tBlackboardReadAccess;
template<typename T>
class tBlackboardWriteAccess;
template <typename T>
class tBlackboard;

/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard client
 */
template<typename T>
class tBlackboardClient : public util::tObject
{
public:
  typedef typename tAbstractBlackboardServer<T>::tBBVector tBBVector;
  typedef typename tAbstractBlackboardServer<T>::tBBVectorVar tBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tConstBBVectorVar tConstBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransaction tChangeTransaction;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransactionVar tChangeTransactionVar;
  typedef typename tAbstractBlackboardServer<T>::tConstChangeTransactionVar tConstChangeTransactionVar;

  typedef tBlackboardWriteAccess<T> tWriteAccess;
  typedef tBlackboardReadAccess<T> tReadAccess;

private:

  /*! Wrapped raw blackboard client */
  tRawBlackboardClient* wrapped;

protected:

  /*! not null - if buffer is currently locked for writing */
  tBBVectorVar locked;

  /*! not null - if buffer is currently locked for reading */
  tConstBBVectorVar read_locked;

  /*! Replicated ports in group's/module's tPortGroups */
  core::tAbstractPort * write_port1, * write_port2;

  /*! Replicated read port in group's/module's tPortGroups */
  std::unique_ptr<core::tPort<std::vector<T>>> read_port;

private:

  /*!
   * Make sure specified type is registered for blackboards
   *
   * \param dt Type
   * \return The same as parameter type
   */
  rrlib::rtti::tDataTypeBase InitBlackboardType(rrlib::rtti::tDataTypeBase dt);

  /*!
   * Check whether these ports can be connected - if yes, do so
   * (connecting to blackboard)
   */
  void CheckConnect(core::tAbstractPort* p1, core::tAbstractPort* p2);

  /*!
   * Check whether these ports can be connected - if yes, do so
   * (connecting to outer blackboard client)
   */
  void CheckClientConnect(core::tAbstractPort* p1, core::tAbstractPort* p2);

  /*!
   * Reset variables after unlock
   */
  inline void ResetVariables()
  {
    wrapped->cur_lock_id = -1;
    wrapped->lock_type = tRawBlackboardClient::eNONE;

    locked.reset();
    read_locked.reset();
  }

  /*!
   * Create blackboard write port in specified port group and connect it to
   * (original blackboard client) write port.
   *
   * \param write_port Port to connect newly created port to
   * \param pg Port group to create port in
   * \param name Name of new port
   * \return Created Port
   */
  static core::tAbstractPort* ReplicateWritePort(core::tAbstractPort* write_port, core::tFrameworkElement* pg, const util::tString& name)
  {
    core::tInterfacePort* new_port = new core::tInterfacePort(name, pg, write_port->GetDataType(), core::tInterfacePort::eRouting);
    write_port->ConnectToSource(*new_port);
    return new_port;
  }

protected:

  /*!
   * \return log description
   */
  inline const core::tFrameworkElement& GetLogDescription() const
  {
    return *wrapped;
  }

public:

  // Empty constructor for blackboard clients that are not initialized in
  // class initializer list (but later)
  tBlackboardClient() :
    wrapped(NULL),
    locked(),
    read_locked(),
    write_port1(NULL),
    write_port2(NULL),
    read_port()
  {
  }

  /*!
   * Connects to global blackboards.
   *
   * \param name Name/Uid of blackboard
   * \param parent Parent of blackboard client
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param auto_connect Auto-Connect blackboard client to matching server?
   * \param auto_connect_category If auto-connect is active: Limit auto-connecting to a specific blackboard category? (-1 is no)
   * \param read_port Create read port?
   * \param write_port Create write port?
   * \param type Data Type of blackboard content
   */
  tBlackboardClient(const util::tString& name, core::tFrameworkElement* parent = NULL, bool push_updates = false, bool auto_connect = true, int auto_connect_category = -1, bool read_port = true, bool write_port = true, rrlib::rtti::tDataTypeBase type = rrlib::rtti::tDataType<T>());

  /*!
   * Connects to local blackboard
   *
   * \param parent Parent of blackboard client
   * \param server Blackboard server to connect to (in case it is available as object)
   * \param non_default_name Default name is "<server name> Client". Specifiy non-empty string for other name
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param read_port Create read port?
   * \param write_port Create write port?
   */
  tBlackboardClient(const tAbstractBlackboardServerRaw* server, core::tFrameworkElement* parent, const util::tString& non_default_name = "", bool push_updates = false, bool read_port = true, bool write_port = true);

  /*!
   * Constructor for use in tGroup, tModule and tSenseControlModule.
   * Does NOT connect to global blackboards - but rather uses group's/module's input/output port groups.
   *
   * (per default, full-blackboard-access-ports are created in Output/ControllerOutput.
   *  If this is not desired, the last two constructor parameters can be used to specify alternatives.)
   *
   * \param name Name of blackboard
   * \param parent Parent of blackboard
   * \param push_updates Use push strategy? (Any blackboard updates will be pushed to read port; required for changed-flag to work properly; disabled by default (network-bandwidth))
   * \param create_read_port Create read port for blackboard? (0 = no, 1 = in internal client, 2 = also in (Sensor)Output)
   * \param create_write_port_in If not NULL, creates write port in specified port group instead of Input/ControllerInput
   * \param create_write_port_in2 If not NULL, creates another write port in specified port group
   */
  tBlackboardClient(const util::tString& name, core::structure::tModuleBase* parent, bool push_updates = false, int create_read_port = 2, core::tPortGroup* create_write_port_in = NULL, core::tPortGroup* create_write_port_in2 = NULL);

  /*!
   * Constructor to replicate access to inner tBlackboardClient in tGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param parent Parent of blackboard
   * \param create_read_port_in_ci In case we have a plain tModule: Create read port in Controller Input (rather than Sensor Input?)
   * \param forward_write_port_in_controller Forward write ports in controller port groups?
   * \param forward_write_port_in_sensor Forward write ports in sensor port groups?
   */
  tBlackboardClient(const tBlackboardClient& replicated_bb, core::structure::tGroup* parent, bool create_read_port_in_ci = false, bool forward_write_port_in_controller = true, bool forward_write_port_in_sensor = false);


  // move assignment
  tBlackboardClient& operator=(tBlackboardClient && o)
  {
    std::swap(wrapped, o.wrapped);
    std::swap(locked, o.locked);
    std::swap(read_locked, o.read_locked);
    std::swap(read_port, o.read_port);
    std::swap(write_port1, o.write_port1);
    std::swap(write_port2, o.write_port2);
    return *this;
  }

  /*!
   * Commit asynchronous change to blackboard. Blackboard does
   * not need to be locked for this operation.
   * (if connection is broken, there's no guarantee that this will work or that an exception is thrown otherwise)
   *
   * \param change_buf Contents to write to this position (unlocked buffer retrieved via getUnusedBuffer OR a used buffer with an additional lock)
   * \param index First element to change
   * \param offset Some custom offset in element (optional)
   * \return Did operation succeed? (usual reason for failing is that blackboard is not connected)
   */
  bool CommitAsynchChange(tChangeTransactionVar& change_buf, int index, int offset);

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
  inline util::tString GetName()
  {
    return wrapped->GetName();
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's read port
   */
  core::tPort<std::vector<T>>* GetOutsideReadPort() const
  {
    return read_port.get();
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's primary write port
   */
  core::tAbstractPort* GetOutsideWritePort() const
  {
    return write_port1;
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's secondary write port
   */
  core::tAbstractPort* GetOutsideWritePort2() const
  {
    return write_port2;
  }

  /*!
   * \return Handle of server that handles calls (can be used to detect whether we're connected to a different blackboard). 0 if not connected to a server.
   */
  int GetServerHandle() const
  {
    return wrapped->GetWritePort()->GetServerHandle();
  }

  /*!
   * \return unused buffer - may be published/committed directly
   */
  inline tBBVectorVar GetUnusedBuffer()
  {
    return wrapped->GetWritePort()->GetBufferForCall<tBBVector>();
  }

  /*!
   * \return unused change buffer - to be used in commitAsynchChange
   */
  inline tChangeTransactionVar GetUnusedChangeBuffer()
  {
    return wrapped->GetWritePort()->GetBufferForCall<tChangeTransaction>();
  }

  /*!
   * \return Wrapped raw blackboard client
   */
  inline tRawBlackboardClient* GetWrapped() const
  {
    return wrapped;
  }

  /*!
   * (only works properly if pushUpdates in constructor was set to true)
   *
   * \return Has port changed since last changed-flag-reset?
   */
  bool HasChanged() const;

  /*!
   * \return Is client currently holding read or write lock?
   */
  inline bool HasLock()
  {
    return wrapped->HasLock();
  }

  /*!
   * \return Is client currently holding read lock?
   */
  inline bool HasReadLock()
  {
    return wrapped->HasReadLock();
  }

  /*!
   * \return Is client currently holding write lock?
   */
  inline bool HasWriteLock()
  {
    return wrapped->HasWriteLock();
  }

  /*!
   * Initialize blackboard client
   */
  inline void Init()
  {
    wrapped->Init();
  }

  /*!
   * Directly commit/publish buffer - without lock
   *
   * \param buffer Buffer to publish (unlocked buffer retrieved via getUnusedBuffer OR a used buffer with an additional lock)
   */
  void Publish(tBBVectorVar& buffer);

  /*!
   * Often Non-blocking, safe blackboard read operation
   * (will always operate on read copy - for SingleBufferedBlackboardServers readLock can be more efficient, but also more blocking)
   *
   * \param timeout (relevant for SingleBufferedBlackboardClients only) Timeout for lock attempt
   * \return Raw memory buffer containing blackboard contents - locked - don't forget to release read lock
   */
  inline tConstBBVectorVar Read(const rrlib::time::tDuration& timeout = std::chrono::seconds(2))
  {
    return core::tPortUtil<tBBVector>::GetValueWithLock(wrapped->GetReadPort());
  }

  /*!
   * Read Lock on blackboard.
   *
   * Blackboard locked using this method needs to be unlocked via unlock() method!
   *
   * In most cases it will return a read copy (this can be forced).
   * On local single buffered blackboard servers - the same buffer might be used for reading (blocks more, but less copying)
   *
   * \param force_read_copy_to_avoid_blocking Force read copy to avoid blocking? (only relevant for single buffered blackboard servers)
   * \param timeout Timeout for call
   */
  const typename tAbstractBlackboardServer<T>::tBBVector* ReadLock(bool force_read_copy_to_avoid_blocking = false, const rrlib::time::tDuration& timeout = std::chrono::minutes(1));

  operator bool()
  {
    return wrapped != NULL;
  }

  /*!
   * (only works properly if pushUpdates in constructor was set to true)
   *
   * Reset changed flag.
   */
  void ResetChanged();

  /*!
   * Commit changes of previously locked buffer
   */
  void Unlock();

  /*!
   * Lock blackboard in order to read and commit changes
   * (synchronous/blocking... only use if absolutely necessary)
   *
   * \param timeout timeout for lock
   * \return Lock Locked buffer - or null if lock failed - this buffer may be modified -
   * call unlock() after modifications are complete - locks of buffer should normally not be modified -
   * except of it should be used in some other port or stored for longer than the unlock() operation
   */
  typename tAbstractBlackboardServer<T>::tBBVector* WriteLock(const rrlib::time::tDuration& timeout = std::chrono::minutes(1));

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tBlackboard.h"
#include "plugins/blackboard/tBlackboardClient.hpp"

#include "plugins/blackboard/tBlackboardReadAccess.h"
#include "plugins/blackboard/tBlackboardWriteAccess.h"

namespace finroc
{
namespace blackboard
{
extern template class tBlackboardClient<tBlackboardBuffer>;
extern template class tBlackboardClient<rrlib::serialization::tMemoryBuffer>;

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardClient_h__
