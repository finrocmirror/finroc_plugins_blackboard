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
finroc::runtime_construction::tStandardCreateModuleAction<mBlackboardWriterAsync> mBlackboardWriterAsync::cCREATE_ACTION("BlackboardWriterAsync");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBlackboardWriterAsync constructors
//----------------------------------------------------------------------
mBlackboardWriterAsync::mBlackboardWriterAsync(finroc::core::tFrameworkElement *parent, const std::string &name)
  : tModule(parent, name),
    bb_client("blackboard", this),
    update_counter(0)
{}

//----------------------------------------------------------------------
// mBlackboardWriterAsync Update
//----------------------------------------------------------------------
void mBlackboardWriterAsync::Update()
{
  // acquire buffer for async change-transaction
  data_ports::tPortDataPointer<std::vector<tChange<float>>> change_buf(bb_client.GetUnusedChangeBuffer());

  // fill buffer
  change_buf->clear();
  change_buf->push_back(tChange<float>(15, update_counter));
  change_buf->push_back(tChange<float>(16, update_counter + 1));
  change_buf->push_back(tChange<float>(17, update_counter + 2));

  // Commit asynch change
  bb_client.AsynchronousChange(change_buf);

  // increment update counter
  update_counter++;
}

