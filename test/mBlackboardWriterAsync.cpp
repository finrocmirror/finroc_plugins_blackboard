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
/*!\file    mBlackboardWriterAsync.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 */
//----------------------------------------------------------------------
#include "plugins/blackboard/test/mBlackboardWriterAsync.h"

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
finroc::core::tStandardCreateModuleAction<mBlackboardWriterAsync> mBlackboardWriterAsync::cCREATE_ACTION("BlackboardWriterAsync");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBlackboardWriterAsync constructors
//----------------------------------------------------------------------
mBlackboardWriterAsync::mBlackboardWriterAsync(finroc::core::tFrameworkElement *parent, const std::string& bb_name, const finroc::util::tString &name)
    : tModule(parent, name),
    bb_client(bb_name, this),
    update_counter(0)
{}

//----------------------------------------------------------------------
// mBlackboardWriterAsync Update
//----------------------------------------------------------------------
void mBlackboardWriterAsync::Update()
{
  // acquire buffer for async change-transaction
  core::tPortDataPtr<std::vector<float> > change_buf(bb_client.GetUnusedChangeBuffer());

  // fill buffer
  change_buf->resize(3);
  (*change_buf)[0] = update_counter;
  (*change_buf)[1] = update_counter + 1;
  (*change_buf)[2] = update_counter + 2;

  // Commit asynch change
  bool success = bb_client.CommitAsynchChange(change_buf, 15, 0); // last parameter is unused

  // optional: print error message if operation fails
  if (!success)
  {
    FINROC_LOG_STREAM(rrlib::logging::eLL_WARNING) << "Could not commit change to blackboard";
  }

  // increment update counter
  update_counter++;
}

