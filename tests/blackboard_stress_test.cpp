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
 * \date    2017-11-27
 *
 * Stress test for blackboard operations in different configurations.
 * (Note that timing parameters may need to be adjusted if executed on processors
 *  much slower than a Core i7)
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
#include "plugins/blackboard/tBlackboard.h"

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
typedef float tBlackboardDataType;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

// for evaluation
std::atomic<const void*> last_buffer_address(nullptr);
std::atomic<uint> buffer_address_changes(0);

class mBlackboardTestServer : public finroc::structure::tModule
{
public:
  tBlackboard<tBlackboardDataType> blackboard;
  tBlackboard<tBlackboardDataType> blackboard_no_copy;

  mBlackboardTestServer(tFrameworkElement* parent, const std::string& name = "Server") :
    tModule(parent, name),
    blackboard("Test Blackboard", this, tBlackboardBufferMode::MULTI_BUFFERED, 20, false, finroc::blackboard::tReadPorts::EXTERNAL),
    blackboard_no_copy("Test Blackboard No Copy", this, tBlackboardBufferMode::SINGLE_BUFFERED, 20, false, finroc::blackboard::tReadPorts::NONE)
  {
  }

  void Update() override
  {
  }
};

class mBlackboardTestClient : public finroc::structure::tThreadContainer<finroc::structure::tModule>
{
public:

  tBlackboardClient<tBlackboardDataType> blackboard_client;

  bool reader;
  std::atomic<int> outside_lock_delay_ms, inside_lock_delay_ms;
  std::atomic<bool> random_delay;

  uint blackboard_size;

  // for evaluation
  std::atomic<int> max_lock_delay_ms;
  std::atomic<int> lock_count;

  std::mt19937 eng;

  mBlackboardTestClient(tFrameworkElement* parent, const std::string& name, bool read_ports) :
    finroc::structure::tThreadContainer<finroc::structure::tModule>(parent, name),
    blackboard_client("Blackboard Client", this, false, read_ports ? finroc::blackboard::tReadPorts::EXTERNAL : finroc::blackboard::tReadPorts::NONE),
    reader(false),
    outside_lock_delay_ms(0),
    inside_lock_delay_ms(0),
    random_delay(false),
    blackboard_size(20),
    max_lock_delay_ms(0),
    lock_count(0)
  {
  }

  void Update() override
  {
    bool random_delay = this->random_delay.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(random_delay ? GetRandomInt(outside_lock_delay_ms.load()) : outside_lock_delay_ms.load()));

    if (reader)
    {
      try
      {
        // Acquire read lock
        //FINROC_LOG_PRINT(USER, *this, " Try blackboard: ", rrlib::time::ToIsoString(rrlib::time::Now()));
        tBlackboardClient<tBlackboardDataType>::tReadAccess acc(blackboard_client, std::chrono::milliseconds(max_lock_delay_ms.load()));
        //FINROC_LOG_PRINT(USER, *this, " Lock blackboard: ", &acc[0], " ", rrlib::time::ToIsoString(rrlib::time::Now()));
        RRLIB_UNIT_TESTS_ASSERT(acc.Size());
        if (acc[1] == 0)
        {
          return;
        }

        // Check content for consistency
        auto first_element = acc[0];
        auto half_size = acc.Size() / 2;
        for (size_t i = 1; i < acc.Size(); i++)
        {
          RRLIB_UNIT_TESTS_EQUALITY(first_element + i, acc[i]);
          if (i == half_size)
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(random_delay ? GetRandomInt(inside_lock_delay_ms.load()) : inside_lock_delay_ms.load()));
          }
        }

        // Update info on buffer changes
        lock_count++;
        auto old = last_buffer_address.exchange(&acc[0]);
        if (&acc[0] != old)
        {
          buffer_address_changes++;
        }

        //FINROC_LOG_PRINT(USER, *this, " Read blackboard: ", &acc[0], " ", rrlib::time::ToIsoString(rrlib::time::Now()));
      }
      catch (const tLockException& e)
      {
        FINROC_LOG_PRINT(USER, GetName(), " could not lock blackboard: ", e);
        RRLIB_UNIT_TESTS_ASSERT(false);
      }
    }
    else
    {
      try
      {
        // Acquire write lock
        //FINROC_LOG_PRINT(USER, *this, " Try blackboard: ", rrlib::time::ToIsoString(rrlib::time::Now()));
        tBlackboardClient<tBlackboardDataType>::tWriteAccess acc(blackboard_client, std::chrono::milliseconds(max_lock_delay_ms.load()));
        uint size = blackboard_size + GetRandomInt(10) - 5;
        //FINROC_LOG_PRINT(USER, *this, " Lock blackboard: ", &acc[0], " ", rrlib::time::ToIsoString(rrlib::time::Now()), " ", size);
        acc.Resize(size);
        int first_element = GetRandomInt(1000);
        auto half_size = size / 2;
        for (size_t i = 0; i < size; i++)
        {
          acc[i] = first_element + i;
          if (i == half_size)
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(random_delay ? GetRandomInt(inside_lock_delay_ms.load()) : inside_lock_delay_ms.load()));
          }
        }
        //FINROC_LOG_PRINT(USER, *this, "Changed content: ", " ", rrlib::time::ToIsoString(rrlib::time::Now()));

        // Update info on buffer changes
        lock_count++;
        auto old = last_buffer_address.exchange(&acc[0]);
        if (&acc[0] != old)
        {
          buffer_address_changes++;
        }

        //FINROC_LOG_PRINT(USER, *this, "Changed blackboard: ", " ", rrlib::time::ToIsoString(rrlib::time::Now()));
      }
      catch (const tLockException& e)
      {
        FINROC_LOG_PRINT(USER, GetName(), " could not lock blackboard: ", e);
        RRLIB_UNIT_TESTS_ASSERT(false);
      }
    }
  }

  int GetRandomInt(int max)
  {
    std::uniform_int_distribution<int> distribution(0, max);
    return distribution(eng);
  }
};

structure::tTopLevelThreadContainer<>* main_thread;

class BlackboardStressTest : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(BlackboardStressTest);
  RRLIB_UNIT_TESTS_ADD_TEST(BlackboardStressTests);
  RRLIB_UNIT_TESTS_END_SUITE;

  void BlackboardStressTests()
  {
    enum tTiming
    {
      NO_DELAY,
      RANDOM_DELAY,
      MIXED_DELAY,
      FIXED_DELAY,
      DIMENSION
    };

    const int cTHREAD_DELAYS[3] = { 3, 4, 5 };
    const int cLOCK_CHECK_EPSILONS[2] = { 50, 150 };

    int test_time = DIMENSION;
    auto test_time_string = getenv("TEST_TIME");
    if (test_time_string)
    {
      test_time = atoi(test_time_string);
    }

    main_thread = new structure::tTopLevelThreadContainer<>("BlackboardTest", "");
    std::string bb_name("Float Blackboard");

    // create two variants of blackboard with capacity 20
    mBlackboardTestServer* test_server = new mBlackboardTestServer(main_thread);
    std::vector<mBlackboardTestClient*> readers, writers;

    for (int mode = 0; mode < 2; mode++)
    {
      auto& server = mode == 0 ? test_server->blackboard : test_server->blackboard_no_copy;
      for (int thread_combo = 0; thread_combo < 4; thread_combo++)
      {
        for (int blackboard_size_index = 0; blackboard_size_index < 2; blackboard_size_index++)
        {

          // Setup test
          const int blackboard_size = blackboard_size_index == 0 ? 20 : 100100;
          const int reader_count = thread_combo & 1 ? 3 : 1;
          const int writer_count = thread_combo & 2 ? 3 : 1;
          const int max_reader_lock_delay = cTHREAD_DELAYS[reader_count - 1];
          const int total_writer_lock_delay = cTHREAD_DELAYS[0] + (writer_count == 1 ? 0 : (cTHREAD_DELAYS[1] + cTHREAD_DELAYS[2]));
          //const int total_lock_delay = max_reader_lock_delay + total_writer_lock_delay;
          const int cLOCK_CHECK_EPSILON = cLOCK_CHECK_EPSILONS[blackboard_size_index];
          FINROC_LOG_PRINT(USER, "\nTesting ", mode ? "single" : "multi", "-buffered blackboard of size ", blackboard_size, " with readers/writers: ", reader_count, "/", writer_count);

          for (int i = 0; i < reader_count; i++)
          {
            readers.push_back(new mBlackboardTestClient(main_thread, "Reader " + std::to_string(i + 1), mode == 0));
            readers.back()->reader = true;
            readers.back()->blackboard_size = blackboard_size;
            readers.back()->max_lock_delay_ms.store((mode == 0 ? 0 : total_writer_lock_delay) + cLOCK_CHECK_EPSILON);
            readers.back()->SetCycleTime(std::chrono::seconds(0));
            readers.back()->blackboard_client.ConnectTo(server);
          }
          for (int i = 0; i < writer_count; i++)
          {
            writers.push_back(new mBlackboardTestClient(main_thread, "Writer " + std::to_string(i + 1), mode == 0));
            writers.back()->reader = false;
            writers.back()->blackboard_size = blackboard_size;
            writers.back()->max_lock_delay_ms.store((mode == 0 ? 0 : max_reader_lock_delay) + total_writer_lock_delay + cLOCK_CHECK_EPSILON - cTHREAD_DELAYS[i]);
            writers.back()->SetCycleTime(std::chrono::seconds(0));
            writers.back()->blackboard_client.ConnectTo(server);
          }
          core::tRuntimeEnvironment::GetInstance().InitAll();

          for (auto writer : writers)
          {
            writer->StartExecution();
          }
          for (auto reader : readers)
          {
            reader->StartExecution();
          }

          // Run test (and change timing mode every second)
          for (int second = 0; second < test_time; second++)
          {
            // Setup timing mode
            int timing_mode = second % DIMENSION;
            switch (timing_mode)
            {
            case NO_DELAY:
              FINROC_LOG_PRINT(USER, "  No Delays");
              for (auto & client : readers)
              {
                client->outside_lock_delay_ms.store(0);
                client->inside_lock_delay_ms.store(0);
                client->random_delay.store(false);
              }
              for (auto & client : writers)
              {
                client->outside_lock_delay_ms.store(0);
                client->inside_lock_delay_ms.store(0);
                client->random_delay.store(false);
              }
              break;
            default:
              RRLIB_UNIT_TESTS_ASSERT(false);
              break;
            case RANDOM_DELAY:
            case MIXED_DELAY:
            case FIXED_DELAY:
              int count = 0;
              std::stringstream stream;
              for (auto & client : readers)
              {
                bool random_delay = timing_mode == RANDOM_DELAY || (timing_mode == MIXED_DELAY && count == 1);
                int outside_delay = (count == 1 && timing_mode != FIXED_DELAY) ? 0 : cTHREAD_DELAYS[count];
                int inside_delay = cTHREAD_DELAYS[count];
                client->outside_lock_delay_ms.store(random_delay ? outside_delay : client->GetRandomInt(outside_delay));
                client->inside_lock_delay_ms.store(random_delay ? inside_delay : client->GetRandomInt(inside_delay));
                client->random_delay.store(random_delay);
                count++;
                stream << "  Reader #" << count << ": " << client->outside_lock_delay_ms.load() << "/" << client->inside_lock_delay_ms.load() << "ms " << (random_delay ? "random" : "fixed");
              }
              count = 0;
              for (auto & client : writers)
              {
                bool random_delay = timing_mode == RANDOM_DELAY || (timing_mode == MIXED_DELAY && count == 1);
                int outside_delay = (count == 1 && timing_mode != FIXED_DELAY) ? 0 : cTHREAD_DELAYS[count];
                int inside_delay = cTHREAD_DELAYS[count];
                client->outside_lock_delay_ms.store(random_delay ? outside_delay : client->GetRandomInt(outside_delay));
                client->inside_lock_delay_ms.store(random_delay ? inside_delay : client->GetRandomInt(inside_delay));
                client->random_delay.store(random_delay);
                count++;
                stream << "  Writer #" << count << ": " << client->outside_lock_delay_ms.load() << "/" << client->inside_lock_delay_ms.load() << "ms " << (random_delay ? "random" : "fixed");
              }
              FINROC_LOG_PRINT(USER, stream.str());
              break;
            }

            // Sleep
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Check results
            bool buffers_should_change = mode == 0;
            bool buffers_change = buffer_address_changes.load() > 4;
            RRLIB_UNIT_TESTS_ASSERT(buffers_change == buffers_should_change);
            buffer_address_changes.store(0);
            for (auto & client : readers)
            {
              RRLIB_UNIT_TESTS_ASSERT(client->lock_count > 4);
              client->lock_count.store(0);
            }
            for (auto & client : writers)
            {
              RRLIB_UNIT_TESTS_ASSERT(client->lock_count > 4);
              client->lock_count.store(0);
            }
          }

          // Cleanup test
          for (auto & client : readers)
          {
            client->PauseExecution();
            client->ManagedDelete();
          }
          readers.clear();
          for (auto & client : writers)
          {
            client->PauseExecution();
            client->ManagedDelete();
          }
          writers.clear();
        }
      }
    }
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(BlackboardStressTest);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
