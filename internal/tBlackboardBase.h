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
 * Internal base class for tBlackboard<T> class
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
//! Base class for tBlackboard<T>
/*!
 * Internal base class for tBlackboard<T> class
 * Common functionality is placed here, which should reduce code size
 */
class tBlackboardBase : public rrlib::util::tNoncopyable
{

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  /*!
   * Get (possibly create) hidden framework element below component to put blackboard stuff beneath
   */
  static tInterface& GetBlackboardsParent(core::tFrameworkElement& component);

  /*! \return Returns constant that indicates that default component interface for read port should be used */
  static tInterface* UseDefaultComponentInterface();

//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:

  typedef core::tFrameworkElement::tFlag tFlag;
  typedef core::tFrameworkElement::tFlag tFlags;

  /*! Replicated ports in group's/module's tPortGroups */
  core::tPortWrapperBase write_port;


  tBlackboardBase();

  /*!
   * Constructor to replicate access to inner tBlackboard in tSenseControlGroup.
   * (per default, full-blackboard-access-ports are created in the same port groups as in the inner group/module.
   *  In case of a plain tModule, ports in tSensorOutput and tControllerInput are created by default.)
   *
   * \param replicated_bb Blackboard to provide access to
   * \param create_write_port_in Interface to create RPC write port in
   * \param write_port_name Name for write port. If empty, replicated blackboard's write port name will be used.
   */
  tBlackboardBase(const tBlackboardBase& replicated_bb, tInterface& create_write_port_in, const std::string& write_port_name);

  /*! move constructor */
  tBlackboardBase(tBlackboardBase && other);

  /*! move assignment */
  tBlackboardBase& operator=(tBlackboardBase && other);

  /*! Helper functions to determine component interface to create read ports in by default */
  static tInterface* GetDefaultReadPortInterface(structure::tSenseControlGroup* g, core::tAbstractPort* replicated_port)
  {
    if (replicated_port && replicated_port->GetParent()->GetFlag(tFlag::CONTROLLER_DATA))
    {
      return &g->GetControllerOutputs();
    }
    return &g->GetSensorOutputs();
  }
  static tInterface* GetDefaultReadPortInterface(structure::tSenseControlModule* g, core::tAbstractPort* replicated_port)
  {
    if (replicated_port && replicated_port->GetParent()->GetFlag(tFlag::CONTROLLER_DATA))
    {
      return &g->GetControllerOutputs();
    }
    return &g->GetSensorOutputs();
  }
  static tInterface* GetDefaultReadPortInterface(structure::tGroup* g, core::tAbstractPort* replicated_port)
  {
    return &g->GetOutputs();
  }
  static tInterface* GetDefaultReadPortInterface(structure::tModule* g, core::tAbstractPort* replicated_port)
  {
    return &g->GetOutputs();
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
}


#endif
