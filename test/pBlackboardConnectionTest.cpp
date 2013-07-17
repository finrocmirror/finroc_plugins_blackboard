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
/*!\file    plugins/blackboard/test/pBlackboardConnectionTest.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-07-19
 *
 * This is a simple test program for the different constructor variants in tBlackboard
 * and tBlackboardClient to connect and replicate local blackboards.
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/tBlackboard.h"
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
using namespace finroc::core;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This is a simple test program for connectiion variants of local blackboards.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
const std::string cMAIN_THREAD_CONTAINER_NAME = "Main Thread";
bool make_all_port_links_unique = true;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
void StartUp()
{}

/*! Module containing Blackboard */
class mBlackboardServer : public finroc::structure::tModule
{
public:
  tBlackboard<float> blackboard;

  mBlackboardServer(tFrameworkElement* parent) :
    tModule(parent, "BlackboardServer"),
    blackboard("Float Blackboard", this)
  {}

private:
  virtual void Update() {}
};

/*! Group replicating Blackboard */
class gBlackboardServer : public finroc::structure::tSenseControlGroup
{
public:
  tBlackboard<float> blackboard;

  gBlackboardServer(tFrameworkElement* parent) :
    tSenseControlGroup(parent, "BlackboardServerGroup", ""),
    blackboard()
  {
    // Create server module
    mBlackboardServer* server = new mBlackboardServer(this);

    // Replicate blackboard in group
    blackboard = tBlackboard<float>(server->blackboard, this);

    // Create asynch writer and connect it to blackboard
    mBlackboardWriterAsync* async_writer = new mBlackboardWriterAsync(this);
    async_writer->bb_client.ConnectTo(server->blackboard);
  }
};

/*! Group replicating BlackboardClient */
class gBlackboardClient : public finroc::structure::tSenseControlGroup
{
public:
  tBlackboardClient<float> bb_client;

  gBlackboardClient(tFrameworkElement* parent) :
    tSenseControlGroup(parent, "BlackboardClientGroup", ""),
    bb_client()
  {
    // Create client modules
    mBlackboardWriter* writer = new mBlackboardWriter(this);
    mBlackboardReader* reader = new mBlackboardReader(this);

    // Replicate blackboard in group
    bb_client = tBlackboardClient<float>(writer->bb_client, this);
    reader->bb_client.ConnectTo(bb_client);
  }
};


void InitMainGroup(finroc::structure::tThreadContainer *main_thread, const std::vector<std::string> &remaining_arguments)
{
  // Create groups and connect them
  gBlackboardServer* server = new gBlackboardServer(main_thread);
  gBlackboardClient* client = new gBlackboardClient(main_thread);
  client->bb_client.ConnectTo(server->blackboard);

  main_thread->SetCycleTime(500);
}
