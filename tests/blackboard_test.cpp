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
/*!\file    plugins/blackboard/tests/blackboard_test.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2014-07-07
 *
 * Tests basic blackboard operations and connecting blackboards across group boundaries
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"
#include "core/tRuntimeEnvironment.h"
#include "plugins/structure/tTopLevelThreadContainer.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/blackboard/mBlackboardServer.h"
#include "plugins/blackboard/tBlackboard.h"
#include "plugins/blackboard/tests/mBlackboardReader.h"
#include "plugins/blackboard/tests/mBlackboardWriter.h"
#include "plugins/blackboard/tests/mBlackboardWriterAsync.h"

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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

/*! Group replicating Blackboard */
class gBlackboardServer : public finroc::structure::tGroup
{
public:
  tBlackboard<float> blackboard;

  gBlackboardServer(tFrameworkElement* parent) :
    tGroup(parent, "BlackboardServerGroup", ""),
    blackboard()
  {
    // Create server module
    mBlackboardServer<float>* server = new mBlackboardServer<float>(this, "BlackboardServer");

    // Replicate blackboard in group
    blackboard = tBlackboard<float>(server->blackboard, this);

    // Create asynch writer and connect it to blackboard
    mBlackboardWriter* async_writer = new mBlackboardWriter(this);
    async_writer->bb_client.ConnectTo(server->blackboard);
  }
};

/*! Group replicating BlackboardClient */
class gBlackboardClient : public finroc::structure::tGroup
{
public:
  tBlackboardClient<float> bb_client;
  mBlackboardReader* reader;

  gBlackboardClient(tFrameworkElement* parent) :
    tGroup(parent, "BlackboardClientGroup", ""),
    bb_client(),
    reader(nullptr)
  {
    // Create client modules
    mBlackboardWriterAsync* writer = new mBlackboardWriterAsync(this);
    reader = new mBlackboardReader(this);

    // Replicate blackboard in group
    bb_client = tBlackboardClient<float>(writer->bb_client, this);
    reader->bb_client.ConnectTo(bb_client);
  }
};

structure::tTopLevelThreadContainer<>* main_thread;

class BlackboardTest : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(BlackboardTest);
  RRLIB_UNIT_TESTS_ADD_TEST(TestBlackboardOperations);
  RRLIB_UNIT_TESTS_ADD_TEST(TestBlackboardConnecting);
  RRLIB_UNIT_TESTS_END_SUITE;

  void TestBlackboardOperations()
  {
    main_thread = new structure::tTopLevelThreadContainer<>("BlackboardTest", "");
    core::tRuntimeEnvironment::GetInstance().InitAll();

    core::tFrameworkElement* parent = new core::tFrameworkElement(main_thread, "TestBlackboardOperations");

    std::string bb_name("Float Blackboard");

    // create single-buffered float blackboard with capacity 20
    mBlackboardServer<float, 20, false>* server_module = new mBlackboardServer<float, 20, false>(parent, bb_name);

    // create modules that access blackboard
    mBlackboardWriter* writer = new mBlackboardWriter(parent);
    mBlackboardWriterAsync* async_writer = new mBlackboardWriterAsync(parent);
    mBlackboardReader* reader = new mBlackboardReader(parent);

    // connect modules with blackboard
    // TODO: change when blackboards are located beneath 'blackboards' element
    writer->bb_client.GetReadPort().ConnectTo(server_module->blackboard.GetReadPort());
    writer->bb_client.GetWritePort().ConnectTo(server_module->blackboard.GetWritePort());
    async_writer->bb_client.GetReadPort().ConnectTo(server_module->blackboard.GetReadPort());
    async_writer->bb_client.GetWritePort().ConnectTo(server_module->blackboard.GetWritePort());
    reader->bb_client.GetReadPort().ConnectTo(server_module->blackboard.GetReadPort());
    reader->bb_client.GetWritePort().ConnectTo(server_module->blackboard.GetWritePort());

    // start thread
    main_thread->Init();

    for (size_t i = 0; i < 10; i++)
    {
      main_thread->ExecuteCycle();
      CheckVector(reader->GetBlackboardCopy(), i);
    }

    // disconnect read ports
    server_module->blackboard.GetReadPort().DisconnectAll();

    for (size_t i = 10; i < 20; i++)
    {
      main_thread->ExecuteCycle();
      CheckVector(reader->GetBlackboardCopy(), i);
    }
  }

  void CheckVector(const std::vector<float> blackboard_values, size_t iteration)
  {
    RRLIB_UNIT_TESTS_ASSERT(blackboard_values.size() == 20);
    for (size_t i = 0; i < 20; i++)
    {
      int expected = 0;
      if (i < 10 || i == 15)
      {
        expected = iteration;
      }
      else if (i == 16)
      {
        expected = iteration + 1;
      }
      else if (i == 17)
      {
        expected = iteration + 2;
      }
      RRLIB_UNIT_TESTS_ASSERT_MESSAGE("Invalid entry in blackboard: " + std::to_string(blackboard_values[i]) +
                                      " (expected: " + std::to_string(expected) + "; iteration: " + std::to_string(iteration) + ")", blackboard_values[i] == expected);
    }
  }

  void TestBlackboardConnecting()
  {
    core::tFrameworkElement* parent = new core::tFrameworkElement(main_thread, "TestBlackboardConnecting");

    // Create groups and connect them
    gBlackboardServer* server = new gBlackboardServer(parent);
    gBlackboardClient* client = new gBlackboardClient(parent);
    client->bb_client.ConnectTo(server->blackboard);
    main_thread->Init();

    for (size_t i = 0; i < 10; i++)
    {
      main_thread->ExecuteCycle();
      CheckVector(client->reader->GetBlackboardCopy(), i);
    }
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(BlackboardTest);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
