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
/*!\file    plugins/blackboard/internal/tBlackboardBase.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 * \brief   Contains tBlackboardBase
 *
 * \b tBlackboardBase
 *
 * internal base class for tBlackboard<T> class
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tBlackboardBase_h__
#define __plugins__blackboard__internal__tBlackboardBase_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tProxyPort.h"
#include "plugins/structure/tGroup.h"
#include "plugins/structure/tModule.h"
#include "plugins/structure/tSenseControlGroup.h"
#include "plugins/structure/tSenseControlModule.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Base class for tBlackboard<T>
/*!
 * Internal base class for tBlackboard<T> class
 * Common functionality is placed here, which should reduce code size
 */
class tBlackboardBase
{

//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:

  /*! Replicated ports in group's/module's tPortGroups */
  core::tAbstractPort * write_port1, * write_port2;

  /*! default parameter for tBlackboard constructor */
  static core::tPortGroup* default_port_group;

  tBlackboardBase();

  /*!
   * Constructor to replicate access to inner tBlackboard in tSenseControlGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param parent Parent of blackboard
   * \param create_read_port_in_co In case we have a plain tModule: Create read port in Controller Output (rather than Sensor Output?)
   * \param forward_write_port_in_controller Forward write ports in controller port groups?
   * \param forward_write_port_in_sensor Forward write ports in sensor port groups?
   */
  tBlackboardBase(const tBlackboardBase& replicated_bb, structure::tSenseControlGroup* parent, bool create_read_port_in_co = false, bool forward_write_port_in_controller = true, bool forward_write_port_in_sensor = false);

  /*! move constructor */
  tBlackboardBase(tBlackboardBase && other);

  /*! move assignment */
  tBlackboardBase& operator=(tBlackboardBase && other);

  /*! Helper function to determine port group to create write ports in by default */
  static core::tPortGroup* GetWritePortGroup(structure::tSenseControlGroup* g)
  {
    return &g->GetControllerInputs();
  }

  static core::tPortGroup* GetWritePortGroup(structure::tSenseControlModule* g)
  {
    return &g->GetControllerInputs();
  }

  static core::tPortGroup* GetWritePortGroup(structure::tGroup* g)
  {
    return &g->GetInputs();
  }

  static core::tPortGroup* GetWritePortGroup(structure::tModule* g)
  {
    return &g->GetInputs();
  }

  static core::tPortGroup* GetWritePortGroup(core::tFrameworkElement* g)
  {
    return NULL;
  }

  /*!
   * Create blackboard write port in specified port group and connect it to
   * (original blackboard) write port.
   *
   * \param write_port Port to connect newly created port to
   * \param pg Port group to create port in
   * \param name Name of new port
   * \return Created Port
   */
  static core::tAbstractPort* ReplicateWritePort(core::tAbstractPort& write_port, core::tFrameworkElement& port_group, const std::string& name);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
