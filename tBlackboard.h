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
 * (and possibly client) in a group or module.
 *
 * This allows considering blackboard edges in scheduling.
 *
 * It can also be used in a group to allow access to a blackboard
 * of an inner module or group.
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
 * (and possibly client) in a group or module.
 *
 * This allows considering blackboard edges in scheduling.
 *
 * It can also be used in a group to allow access to a blackboard
 * of an inner module or group.
 */
template <typename T>
class tBlackboard : public internal::tBlackboardBase
{
  template <typename TParent>
  struct tGetReadPortType;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef tBlackboardWriteAccess<T> tWriteAccess;
  typedef tBlackboardReadAccess<T> tReadAccess;

  typedef internal::tBlackboardServer<T> tServer;

  typedef typename tServer::tBuffer tBuffer;

  /*!
   * Empty constructor for blackboards that are not initialized in
   * class initializer list (but later)
   */
  tBlackboard() :
    wrapped_server(NULL),
    wrapped_client(),
    read_port()
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
   * \param create_read_port Which read ports to create for blackboard
   * \param create_write_port_in If not NULL, creates write port in specified port group
   * \param create_write_port_in2 If not NULL, creates another write port in specified port group
   */
  template <typename TParent>
  tBlackboard(const std::string& name, TParent* parent, bool multi_buffered = false, int elements = 0, bool create_client = true,
              tReadPorts create_read_port = tReadPorts::EXTERNAL, core::tPortGroup* create_write_port_in = default_port_group, core::tPortGroup* create_write_port_in2 = NULL) :
    wrapped_server(NULL),
    wrapped_client(),
    read_port()
  {
    // Get/create Framework element to put blackboard stuff beneath
    core::tFrameworkElement* blackboard_parent = parent->GetChild("Blackboards");
    if (!blackboard_parent)
    {
      blackboard_parent = new core::tFrameworkElement(parent, "Blackboards");
    }

    // Create blackboard server
    wrapped_server = new internal::tBlackboardServer<T>(name, blackboard_parent, multi_buffered, elements, false);

    // Create blackboard client
    if (create_client)
    {
      wrapped_client = tBlackboardClient<T>(*wrapped_server, blackboard_parent, "", false, create_read_port != tReadPorts::NONE);
    }

    // Possibly create read ports in module
    if (create_read_port == tReadPorts::EXTERNAL && GetWritePortGroup(parent) != NULL)
    {
      typedef typename tGetReadPortType<typename std::remove_pointer<TParent>::type>::type tReadPort;
      read_port = tReadPort(name, parent, core::tFrameworkElement::tFlag::ACCEPTS_DATA); // make this a proxy port
      wrapped_server->GetReadPort().ConnectTo(read_port);
    }

    // create write/full-access ports
    if (create_write_port_in != NULL && GetWritePortGroup(parent) != NULL)
    {
      write_port1 = ReplicateWritePort(*wrapped_server->GetWritePort().GetWrapped(),
                                       *(create_write_port_in != default_port_group ? create_write_port_in : GetWritePortGroup(parent)), name);
    }
    if (create_write_port_in2 != NULL && GetWritePortGroup(parent) != NULL)
    {
      write_port2 = ReplicateWritePort(*wrapped_server->GetWritePort().GetWrapped(), *create_write_port_in2, name);
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
  tBlackboard(tBlackboard& replicated_bb, structure::tSenseControlGroup* parent, bool create_read_port_in_co = false,
              bool forward_write_port_in_controller = true, bool forward_write_port_in_sensor = false) :
    tBlackboardBase(replicated_bb, parent, create_read_port_in_co, forward_write_port_in_controller, forward_write_port_in_sensor),
    wrapped_server(NULL),
    wrapped_client(),
    read_port()
  {
    // forward read port
    if (replicated_bb.read_port.GetWrapped())
    {
      // where do we create port?
      core::tFrameworkElement* pg = replicated_bb.read_port.GetParent();
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
        read_port = structure::tSenseControlGroup::tControllerOutput<std::vector<T>>(replicated_bb.read_port.GetName(), parent);
      }
      else
      {
        read_port = structure::tSenseControlGroup::tSensorOutput<std::vector<T>>(replicated_bb.read_port.GetName(), parent);
      }
      replicated_bb.read_port.ConnectTo(read_port);
    }
  }

  /*! move constructor */
  tBlackboard(tBlackboard && o) :
    tBlackboardBase(std::forward(o)),
    wrapped_server(NULL),
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
  std::string GetName() const
  {
    return wrapped_server->GetName();
  }

  /*!
   * \return Port to use, when modules outside of group/module containing blackboard want to connect to this blackboard's read port
   */
  data_ports::tPort<std::vector<T>> GetOutsideReadPort() const
  {
    return read_port;
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
  data_ports::tOutputPort<tBuffer> GetReadPort() const
  {
    return wrapped_server->GetReadPort();
  }

  /*!
   * \return Port to use, when modules inside group containing blackboard want to connect to this blackboard's primary write port
   */
  rpc_ports::tServerPort<internal::tBlackboardServer<T>> GetWritePort() const
  {
    return rpc_ports::tServerPort<internal::tBlackboardServer<T>>::Wrap(*wrapped_server->GetWritePort().GetWrapped());
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Wrapped blackboard server */
  internal::tBlackboardServer<T>* wrapped_server;

  /*! Wrapped blackboard client - contains NULL pointer if no such client was created */
  tBlackboardClient<T> wrapped_client;

  /*! Replicated read port in group's/module's tPortGroups */
  data_ports::tPort<std::vector<T>> read_port;


  template <typename TParent>
  struct tGetReadPortType
  {
    typedef std::vector<T> tBuffer;
    typedef typename std::conditional < std::is_base_of<structure::tModule, TParent>::value, structure::tModule::tOutput<tBuffer>,
            typename std::conditional < std::is_base_of<structure::tSenseControlModule, TParent>::value, structure::tSenseControlModule::tSensorOutput<tBuffer>,
            typename std::conditional < std::is_base_of<structure::tSenseControlGroup, TParent>::value, structure::tSenseControlGroup::tSensorOutput<tBuffer>,
            data_ports::tPort<tBuffer >>::type >::type >::type type;
  };
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
