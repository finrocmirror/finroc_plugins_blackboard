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
/*!\file    pBlackboardTest.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 * This is a simple test program that demonstrates how to use blackboards.
 */
//----------------------------------------------------------------------
#include "core/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/test/mBlackboardReader.h"
#include "plugins/blackboard/test/mBlackboardWriter.h"
#include "plugins/blackboard/test/mBlackboardWriterAsync.h"
#include "plugins/blackboard/tSingleBufferedBlackboardServer.h"

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
void InitMainGroup(finroc::core::tThreadContainer *main_thread, const rrlib::getopt::tOption& option)
{
  std::string bb_name("Float Blackboard");

  // create single-buffered float blackboard with capacity 20
  new tSingleBufferedBlackboardServer<float>(bb_name, 20);

  // create modules that access blackboard
  new mBlackboardReader(main_thread, bb_name);
  new mBlackboardWriter(main_thread, bb_name);
  new mBlackboardWriterAsync(main_thread, bb_name);

  main_thread->SetCycleTime(500);
}