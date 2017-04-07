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
/*!\file    plugins/blackboard/mBlackboardServer.h
 *
 * \author  Max Reichardt
 *
 * \date    2016-07-07
 *
 * \brief Contains mBlackboardServer
 *
 * \b mBlackboardServer
 *
 * Component that provides a blackboard server only.
 * May be useful in a composite with only blackboard clients.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__mBlackboardServer_h__
#define __plugins__blackboard__mBlackboardServer_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboard.h"

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
//! Blackboard Server component
/*!
 * Component that provides a blackboard server only.
 * May be useful in a composite with only blackboard clients.
 */
template <typename T, int Tinitial_element_count = 0, bool Tcreate_read_port = true>
class mBlackboardServer : public structure::tModuleBase
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*! Actual blackboard */
  tBlackboard<T> blackboard;

  /*! Blackboard parameters (see tBlackboard constructor) */
  tStaticParameter<int> par_initial_element_count;
  tStaticParameter<bool> par_create_read_port;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mBlackboardServer(core::tFrameworkElement *parent, const std::string &name, tFlags extra_flags = tFlags(), bool share_ports = false) :
    tModuleBase(parent, name, extra_flags, share_ports),
    blackboard(),
    par_initial_element_count(Tinitial_element_count),
    par_create_read_port(Tcreate_read_port)
  {
    this->CheckStaticParameters();
  }

  /*!
   * \return Output interface
   */
  inline tInterface& GetOutputs()
  {
    return this->GetInterface(structure::tModule::cOUTPUT_INTERFACE_INFO, GetFlag(tFlag::SHARED));
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mBlackboardServer() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual void OnStaticParameterChange() override
  {
    bool blackboard_has_read_port = blackboard.GetReadPort().GetWrapped();
    bool create_blackbard = (!blackboard.GetWritePort().GetWrapped()) || par_create_read_port.Get() != blackboard_has_read_port || ((!IsReady()) && par_initial_element_count.HasChanged());
    if (create_blackbard)
    {
      blackboard.ManagedDelete();
      blackboard = tBlackboard<T>("Blackboard", this, false, par_initial_element_count.Get(), false, par_create_read_port.Get() ? &GetOutputs() : nullptr, "Blackboard Data");
    }
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
