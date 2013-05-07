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
/*!\file    plugins/blackboard/test/mBlackboardWriter.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 */
//----------------------------------------------------------------------
#include "plugins/blackboard/test/mBlackboardWriter.h"

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
finroc::runtime_construction::tStandardCreateModuleAction<mBlackboardWriter> mBlackboardWriter::cCREATE_ACTION("BlackboardWriter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBlackboardWriter constructors
//----------------------------------------------------------------------
mBlackboardWriter::mBlackboardWriter(finroc::core::tFrameworkElement *parent, const std::string &name)
  : tModule(parent, name),
    bb_client("blackboard", this),
    update_counter(0)
{}

//----------------------------------------------------------------------
// mBlackboardWriter Update
//----------------------------------------------------------------------
void mBlackboardWriter::Update()
{
  try
  {
    // Acquire write lock
    tBlackboardClient<float>::tWriteAccess acc(bb_client);

    if (acc.Size() < 10)
    {
      acc.Resize(20);
    }

    // Change elements 0 to 9
    for (size_t i = 0; i < 10; i++)
    {
      acc[i] = update_counter;
    }
  }
  catch (tLockException& e)
  {
    FINROC_LOG_PRINT(WARNING, "Could not lock blackboard");
  }
  update_counter++;
}

