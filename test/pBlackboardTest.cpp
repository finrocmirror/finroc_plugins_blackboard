//
// You received this file as part of Finroc
// A framework for integrated robot control
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
/*!\file    pBlackboardTest.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 * This is a simple test program that demonstrates how to use blackboards.
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboard.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/test/mBlackboardReader.h"
#include "plugins/blackboard/test/mBlackboardWriter.h"
#include "plugins/blackboard/test/mBlackboardWriterAsync.h"

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
const char * const cPROGRAM_VERSION = "ver 1.0";
const char * const cPROGRAM_DESCRIPTION = "This is a simple test program that demonstrates how to use blackboards.";

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// InitMainGroup
//----------------------------------------------------------------------
void InitMainGroup(finroc::structure::tThreadContainer *main_thread, std::vector<char*> remaining_args)
{
  std::string bb_name("Float Blackboard");

  // create single-buffered float blackboard with capacity 20
  tBlackboard<float> blackboard(bb_name, main_thread, false, 20, false, tReadPorts::NONE, NULL);

  // create modules that access blackboard
  mBlackboardReader* reader = new mBlackboardReader(main_thread);
  mBlackboardWriter* writer = new mBlackboardWriter(main_thread);
  mBlackboardWriterAsync* async_writer = new mBlackboardWriterAsync(main_thread);

  // connect modules with blackboard
  blackboard.GetWritePort().ConnectTo(writer->bb_client.GetOutsideWritePort());
  blackboard.GetWritePort().ConnectTo(async_writer->bb_client.GetOutsideWritePort());
  blackboard.GetWritePort().ConnectTo(reader->bb_client.GetOutsideWritePort());
  blackboard.GetReadPort().ConnectTo(reader->bb_client.GetOutsideReadPort());

  main_thread->SetCycleTime(500);
}
