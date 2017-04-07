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
/*!\file    plugins/blackboard/internal/tBlackboardBase.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-18
 *
 */
//----------------------------------------------------------------------
#include "plugins/blackboard/internal/tBlackboardBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/string.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tBlackboardBase::tBlackboardBase() :
  write_port(nullptr)
{}

tBlackboardBase::tBlackboardBase(const tBlackboardBase& replicated_bb, tInterface& create_write_port_in, const std::string& write_port_name) :
  write_port(create_write_port_in.CreatePort(write_port_name.length() ? write_port_name : replicated_bb.write_port.GetName(), replicated_bb.write_port.GetDataType(),
             core::tFrameworkElement::tFlag::ACCEPTS_DATA | core::tFrameworkElement::tFlag::EMITS_DATA | (replicated_bb.write_port.GetFlag(tFlag::OUTPUT_PORT) ? tFlag::OUTPUT_PORT : tFlag::PORT)))
{
  write_port.ConnectTo(replicated_bb.write_port);
}

tBlackboardBase::tBlackboardBase(tBlackboardBase && other) :
  write_port(nullptr)
{
  std::swap(write_port, other.write_port);
}

tBlackboardBase& tBlackboardBase::operator=(tBlackboardBase && other)
{
  std::swap(write_port, other.write_port);
  return *this;
}

tInterface& tBlackboardBase::GetBlackboardsParent(core::tFrameworkElement& component)
{
  core::tFrameworkElement* blackboard_parent = component.GetChild("Blackboards");
  if (blackboard_parent)
  {
    return static_cast<tInterface&>(*blackboard_parent);
  }
  tInterface* result = new tInterface(&component, "Blackboards", tFlag::INTERFACE, tFlags());
  core::tFrameworkElementTags::AddTag(*result, core::tFrameworkElementTags::cHIDDEN_IN_TOOLS);
  result->Init();
  return *result;
}

tInterface* tBlackboardBase::UseDefaultComponentInterface()
{
  return reinterpret_cast<tInterface*>(1);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
