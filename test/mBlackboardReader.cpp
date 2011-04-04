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
/*!\file    mBlackboardReader.cpp
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
using namespace finroc::blackboard;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
finroc::core::tStandardCreateModuleAction<mBlackboardReader> mBlackboardReader::cCREATE_ACTION("BlackboardReader");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBlackboardReader constructors
//----------------------------------------------------------------------
mBlackboardReader::mBlackboardReader(finroc::core::tFrameworkElement *parent, const std::string& bb_name, const finroc::util::tString &name)
    : tModule(parent, name),
    bb_client(bb_name, this)
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
    rrlib::logging::tLogStream output = FINROC_LOG_STREAM(rrlib::logging::eLL_USER);
    output << "Current blackboard content:";
    for (size_t i = 0; i < acc.Size(); i++)
    {
      output << " " << acc[i];
    }

  }
  catch (tBBLockException& e)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING) << "Could not lock blackboard";
  }
}
