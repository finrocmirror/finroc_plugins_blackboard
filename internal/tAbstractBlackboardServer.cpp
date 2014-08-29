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
/*!\file    plugins/blackboard/internal/tAbstractBlackboardServer.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-19
 *
 */
//----------------------------------------------------------------------
#include "plugins/blackboard/internal/tAbstractBlackboardServer.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tLockOrderLevel.h"

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

/*! Default lock timeout - if no keep-alive signal occurs in this period of time, blackboard is unlocked */
//static constexpr rrlib::time::tDuration cDEFAULT_LOCK_TIMEOUT = std::chrono::seconds(1);

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
tAbstractBlackboardServer::tAbstractBlackboardServer(core::tFrameworkElement* parent, const std::string& name, tFrameworkElement::tFlags flags) :
  tFrameworkElement(parent, name, flags),
  blackboard_mutex("Blackboard", static_cast<int>(core::tLockOrderLevel::INNER_MOST) - 1000),
  revision_counter(0)
{
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
