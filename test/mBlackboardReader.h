//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    mBlackboardReader.h
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 * \brief Contains mBlackboardReader
 *
 * \b mBlackboardReader
 *
 */
//----------------------------------------------------------------------
#ifndef _blackboard__mBlackboardReader_h_
#define _blackboard__mBlackboardReader_h_

#include "core/structure/tModule.h"
#include "plugins/blackboard/tBlackboardClient.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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
//! Reads contents of blackboard and prints them to console.
class mBlackboardReader : public finroc::core::structure::tModule
{
  static finroc::core::tStandardCreateModuleAction<mBlackboardReader> cCREATE_ACTION;

  tBlackboardClient<float> bb_client;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------

  virtual void Update();

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  mBlackboardReader(finroc::core::tFrameworkElement *parent, const std::string& bb_name, const finroc::util::tString &name = "BlackboardReader");

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
