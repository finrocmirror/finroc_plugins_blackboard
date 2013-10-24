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
/*!\file    plugins/blackboard/test/mBlackboardReader.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 */
//----------------------------------------------------------------------
#include "plugins/blackboard/test/mBlackboardReader.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mBlackboardReader> cCREATE_ACTION_FOR_M_BLACKBOARD_READER("BlackboardReader");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBlackboardReader constructors
//----------------------------------------------------------------------
mBlackboardReader::mBlackboardReader(core::tFrameworkElement *parent, const std::string &name)
  : tModule(parent, name),
    bb_client("blackboard", this)
{}

//----------------------------------------------------------------------
// mBlackboardReader Update
//----------------------------------------------------------------------
void mBlackboardReader::Update()
{
  try
  {
    // Acquire read lock
    tBlackboardClient<float>::tReadAccess acc(bb_client);

    // Print blackboard contents
    std::stringstream output;
    output << "Current blackboard content:";
    for (size_t i = 0; i < acc.Size(); i++)
    {
      output << " " << acc[i];
    }
    FINROC_LOG_PRINT(USER, output.str());
  }
  catch (tLockException& e)
  {
    FINROC_LOG_PRINT(WARNING, "Could not lock blackboard");
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
