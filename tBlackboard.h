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

#ifndef plugins__blackboard__tBlackboard_h__
#define plugins__blackboard__tBlackboard_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "plugins/blackboard/tSingleBufferedBlackboardServer.h"
#include "plugins/blackboard/tBlackboardServer.h"
#include "plugins/blackboard/tBlackboardClient.h"
#include "core/structure/tGroup.h"
#include "core/structure/tSenseControlModule.h"
#include "core/structure/tModule.h"

namespace finroc
{
namespace blackboard
{
namespace internal
{

/*! Helper to determine port type for any read port replications */
template <typename T, bool MOD, bool SCMOD, bool GRP>
struct tGetReadPortTypeBase
{
  typedef core::tPort<std::vector<T>> type;
};

template <typename T>
struct tGetReadPortTypeBase<T, true, false, false>
{
  typedef core::structure::tModule::tOutput<std::vector<T>> type;
};

template <typename T>
struct tGetReadPortTypeBase<T, false, true, false>
{
  typedef core::structure::tSenseControlModule::tSensorOutput<std::vector<T>> type;
};

template <typename T>
struct tGetReadPortTypeBase<T, false, false, true>
{
  typedef core::structure::tGroup::tSensorOutput<std::vector<T>> type;
};

template <typename T, typename P>
struct tGetReadPortType : public tGetReadPortTypeBase<T, std::is_base_of<core::structure::tModule, P>::value, std::is_base_of<core::structure::tSenseControlModule, P>::value, std::is_base_of<core::structure::tGroup, P>::value>
{
};

/*!
 * internal base class for tBlackboard<T> class
 */
class tBlackboardBase
{

protected:

  /*! Replicated ports in group's/module's tPortGroups */
  core::tAbstractPort * write_port1, * write_port2;

  /*! default parameter for tBlackboard constructor */
  static core::tPortGroup* default_port_group;

  tBlackboardBase() :
    write_port1(NULL),
    write_port2(NULL)
  {}

  /*!
   * Constructor to replicate access to inner tBlackboard in tGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param parent Parent of blackboard
   * \param create_read_port_in_co In case we have a plain tModule: Create read port in Controller Output (rather than Sensor Output?)
   * \param forward_write_port_in_controller Forward write ports in controller port groups?
   * \param forward_write_port_in_sensor Forward write ports in sensor port groups?
   */
  tBlackboardBase(const tBlackboardBase& replicated_bb, core::structure::tGroup* parent, bool create_read_port_in_co = false, bool forward_write_port_in_controller = true, bool forward_write_port_in_sensor = false);

  /*!
   * Create blackboard write port in specified port group and connect it to
   * (original blackboard) write port.
   *
   * \param write_port Port to connect newly created port to
   * \param pg Port group to create port in
   * \param name Name of new port
   * \return Created Port
   */
  static core::tAbstractPort* ReplicateWritePort(core::tAbstractPort* write_port, core::tFrameworkElement* pg, const util::tString& name);

  /*! Helper function to determine port group to create write ports in by default */
  static core::tPortGroup* GetWritePortGroup(core::structure::tGroup* g)
  {
    return &g->GetControllerInputs();
  }

  static core::tPortGroup* GetWritePortGroup(core::structure::tSenseControlModule* g)
  {
    return &g->GetControllerInputs();
  }

  static core::tPortGroup* GetWritePortGroup(core::structure::tModule* g)
  {
    return &g->GetInputs();
  }

  static core::tPortGroup* GetWritePortGroup(core::tFrameworkElement* g)
  {
    return NULL;
  }
};

} // namespace

/*!
 * \author Max Reichardt
 *
 * This is a convenience class to create a blackboard server
 * (and possibly client) in a group or module.
 * This allows considering blackboard edges in scheduling.
 *
 * It can also be used in a group to allow access to a blackboard
 * of an inner module or group.
 */
template<typename T>
class tBlackboard : internal::tBlackboardBase
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

  /*! Wrapped blackboard server */
  tAbstractBlackboardServer<T>* wrapped_server;

  /*! Wrapped blackboard client - contains NULL pointer if no such client was created */
  tBlackboardClient<T> wrapped_client;

  /*! Replicated read port in group's/module's tPortGroups */
  std::unique_ptr<core::tPort<std::vector<T>>> read_port;

public:

  /*!
   * Empty constructor for blackboards that are not initialized in
   * class initializer list (but later)
   */
  tBlackboard() :
    wrapped_server(NULL),
    wrapped_client()
  {}

  /*!
   * Constructor for use in tGroup, tModule and tSenseControlModule
   * (per default, full-blackboard-access-ports are created in Input/ControllerInput.
   *  If this is not desired, the last two constructor parameters can be used to specify alternatives.)
   *
   * \param name Name of blackboard
   * \param parent Parent of blackboard
   * \param multi_buffered Create multi-buffered blackboard?
   * \param elements Initial number of elements
   * \param create_client Create Blackboard client?
   * \param create_read_port Create read port for blackboard? (0 = no, 1 = in internal client, 2 = also in (Sensor)Output)
   * \param create_write_port_in If not NULL, creates write port in specified port group
   * \param create_write_port_in2 If not NULL, creates another write port in specified port group
   */
  template <typename P>
  tBlackboard(const util::tString& name, P* parent, bool multi_buffered = false, int elements = 0, bool create_client = true, int create_read_port = 2, core::tPortGroup* create_write_port_in = default_port_group, core::tPortGroup* create_write_port_in2 = NULL) :
    wrapped_server(NULL),
    wrapped_client()
  {
    // Get/create Framework element to put blackboard stuff beneath
    core::tFrameworkElement* bbs = parent->GetChild("Blackboards");
    if (bbs == NULL)
    {
      bbs = new core::tFrameworkElement(parent, "Blackboards");
    }

    // Create blackboard server
    wrapped_server = multi_buffered ?
                     static_cast<tAbstractBlackboardServer<T>*>(new tBlackboardServer<T>(name, elements, bbs, false)) :
                     static_cast<tAbstractBlackboardServer<T>*>(new tSingleBufferedBlackboardServer<T>(name, elements, bbs, false));

    // Create blackboard client
    if (create_client)
    {
      wrapped_client = tBlackboardClient<T>(wrapped_server, bbs, "", false, create_read_port > 0);
    }

    // Possibly create read ports in module
    if (create_read_port >= 2 && GetWritePortGroup(parent) != NULL)
    {
      typedef typename internal::tGetReadPortType<T, typename std::remove_pointer<P>::type>::type tReadPort;
      read_port.reset(new tReadPort(name, parent, core::tPortFlags::cOUTPUT_PROXY));
      wrapped_server->read_port_raw->ConnectToTarget(*read_port->GetWrapped());
    }

    // create write/full-access ports
    if (create_write_port_in != NULL && GetWritePortGroup(parent) != NULL)
    {
      write_port1 = ReplicateWritePort(wrapped_server->write_port_raw, create_write_port_in != default_port_group ? create_write_port_in : GetWritePortGroup(parent), name);
    }
    if (create_write_port_in2 != NULL && GetWritePortGroup(parent) != NULL)
    {
      write_port2 = ReplicateWritePort(wrapped_server->write_port_raw, create_write_port_in2, name);
    }
  }

  /*!
   * Constructor to replicate access to inner tBlackboard in tGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param parent Parent of blackboard
   * \param create_read_port_in_co In case we have a plain tModule: Create read port in Controller Output (rather than Sensor Output?)
   * \param forward_write_port_in_controller Forward write ports in controller port groups?
   * \param forward_write_port_in_sensor Forward write ports in sensor port groups?
   */
  tBlackboard(const tBlackboard& replicated_bb, core::structure::tGroup* parent, bool create_read_port_in_co = false, bool forward_write_port_in_controller = true, bool forward_write_port_in_sensor = false) :
    tBlackboardBase(replicated_bb, parent, create_read_port_in_co, forward_write_port_in_controller, forward_write_port_in_sensor),
    wrapped_server(NULL),
    wrapped_client()
  {
    // forward read port
    if (replicated_bb.read_port)
    {
      // where do we create port?
      core::tFrameworkElement* pg = replicated_bb.read_port->GetParent();
      if (pg->NameEquals("Sensor Output"))
      {
        create_read_port_in_co = false;
      }
      else if (pg->NameEquals("Controller Output"))
      {
        create_read_port_in_co = true;
      }
      else
      {
        assert(pg->NameEquals("Output") && "Have names changed?");
      }

      // create port
      if (create_read_port_in_co)
      {
        read_port.reset(new core::structure::tGroup::tControllerOutput<std::vector<T>>(replicated_bb.read_port->GetName(), parent));
      }
      else
      {
        read_port.reset(new core::structure::tGroup::tSensorOutput<std::vector<T>>(replicated_bb.read_port->GetName(), parent));
      }
      replicated_bb.read_port->ConnectToTarget(*read_port);
    }

  }

  /*!
   * \return Wrapped blackboard client - contains NULL pointer if no such client was created
   */
  tBlackboardClient<T>& GetClient()
  {
    return wrapped_client;
  }

  /*! same as tFrameworkElement::GetCName() */
  const char* GetCName() const
  {
    return wrapped_server->GetCName();
  }

  /*! same as tFrameworkElement::GetName() */
  util::tString GetName() const
  {
    return wrapped_server->GetName();
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
   * \return Port to use, when modules inside group containing blackboard want to connect to this blackboard's read port
   */
  core::tAbstractPort* GetReadPort() const
  {
    return wrapped_server->read_port_raw;
  }

  /*!
   * \return Port to use, when modules inside group containing blackboard want to connect to this blackboard's primary write port
   */
  core::tAbstractPort* GetWritePort() const
  {
    return wrapped_server->write_port_raw;
  }
};

}
}

#endif // plugins__blackboard__tBlackboard_h__
