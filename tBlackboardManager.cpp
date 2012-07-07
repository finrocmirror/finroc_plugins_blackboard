/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2007-2010 Max Reichardt,
 *   Robotics Research Lab, University of Kaiserslautern
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include "plugins/blackboard/tBlackboardManager.h"
#include "core/tCoreFlags.h"
#include "plugins/blackboard/tRawBlackboardClient.h"
#include "core/plugin/tPlugins.h"
#include "plugins/blackboard/tBlackboardPlugin.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"
#include "core/port/std/tPortBase.h"
#include "plugins/blackboard/tRemoteBlackboardServer.h"
#include "core/port/tPortCreationInfo.h"
#include "core/port/tPortFlags.h"
#include "core/port/rpc/tInterfacePort.h"

namespace finroc
{
namespace blackboard
{
const int tBlackboardManager::cALL, tBlackboardManager::cSHARED, tBlackboardManager::cLOCAL, tBlackboardManager::cREMOTE, tBlackboardManager::cDIMENSION;
util::tString tBlackboardManager::cNAME = "Blackboards";
util::tString tBlackboardManager::cSLASHED_NAME = std::string("/") + tBlackboardManager::cNAME + "/";
util::tString tBlackboardManager::cREAD_PORT_NAME = "read", tBlackboardManager::cWRITE_PORT_NAME = "write";
util::tString tBlackboardManager::cREAD_POSTFIX = std::string("/") + tBlackboardManager::cREAD_PORT_NAME, tBlackboardManager::cWRITE_POSTFIX = std::string("/") + tBlackboardManager::cWRITE_PORT_NAME;
tBlackboardManager* volatile tBlackboardManager::instance;

tBlackboardManager::tBlackboardManager() :
  core::tFrameworkElement(core::tRuntimeEnvironment::GetInstance(), cNAME),
  categories(cDIMENSION),
  temp_buffer(),
  bb_clients(10),
  auto_connect_clients(core::tLockOrderLevels::cINNER_MOST - 50)
{
  categories[cLOCAL] = new tBlackboardCategory(this, "Local", core::tCoreFlags::cALLOWS_CHILDREN);
  categories[cSHARED] = new tBlackboardCategory(this, "Shared", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cSHARED | core::tCoreFlags::cGLOBALLY_UNIQUE_LINK);
  categories[cREMOTE] = new tBlackboardCategory(this, "Remote", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT);
  tLockCheckerThread* checker = new tLockCheckerThread(this);
  checker->SetAutoDelete();
  checker->Start();
}

void tBlackboardManager::AddClient(tRawBlackboardClient* client, bool auto_connect)
{
  {
    rrlib::thread::tLock lock2(bb_clients.GetMutex());
    bb_clients.Add(client, false);
  }
  if (!auto_connect)
  {
    return;
  }

  rrlib::thread::tLock lock2(auto_connect_clients);
  auto_connect_clients.Add(client);

  for (int j = 0; (j < cDIMENSION) && (!client->IsConnected()); j++)
  {
    categories[j]->CheckConnect(client);
  }
}

void tBlackboardManager::CheckAutoConnect(tAbstractBlackboardServerRaw* server)
{
  if (server->read_port_raw == NULL || server->write_port_raw == NULL)
  {
    return;
  }

  rrlib::thread::tLock lock2(auto_connect_clients);
  for (size_t i = 0u; i < auto_connect_clients.Size(); i++)
  {
    auto_connect_clients.Get(i)->CheckConnect(server);
  }
}

void tBlackboardManager::CreateBlackboardManager()
{
  rrlib::thread::tLock lock2(core::tRuntimeEnvironment::GetInstance()->GetRegistryLock());
  if (instance == NULL)
  {
    instance = new tBlackboardManager();
    instance->Init();
    core::tRuntimeEnvironment::GetInstance()->AddListener(*instance);
  }
}

tAbstractBlackboardServerRaw* tBlackboardManager::GetBlackboard(const util::tString& name, int category, rrlib::rtti::tDataTypeBase type)
{
  int start_cat = category < 0 ? 0 : category;
  int end_cat = category < 0 ? cDIMENSION - 1 : start_cat;
  return GetBlackboard(name, start_cat, end_cat, type);
}

tAbstractBlackboardServerRaw* tBlackboardManager::GetBlackboard(const util::tString& name, int start_cat, int end_cat, rrlib::rtti::tDataTypeBase type)
{
  if (type.GetListType() != NULL)
  {
    type = type.GetListType();
  }
  for (int c = start_cat; c <= end_cat; c++)
  {
    tBlackboardCategory* cat = categories[c];
    util::tArrayWrapper<tAbstractBlackboardServerRaw*>* it = cat->blackboards.GetIterable();
    for (size_t i = 0u; i < it->Size(); i++)
    {
      tAbstractBlackboardServerRaw* info = it->Get(i);
      if (boost::equals(info->GetName(), name) && (type == NULL || info->read_port_raw->GetDataType() == type))
      {
        return info;
      }
    }
  }
  return NULL;
}

tAbstractBlackboardServerRaw* tBlackboardManager::GetBlackboard(size_t index, int category)
{
  int start_cat = category < 0 ? 0 : category;
  int end_cat = category < 0 ? cDIMENSION - 1 : start_cat;
  return GetBlackboard(index, start_cat, end_cat);
}

tAbstractBlackboardServerRaw* tBlackboardManager::GetBlackboard(size_t index, int start_cat, int end_cat)
{
  for (int c = start_cat; c <= end_cat; c++)
  {
    tBlackboardCategory* cat = categories[c];
    util::tArrayWrapper<tAbstractBlackboardServerRaw*>* it = cat->blackboards.GetIterable();
    if (index >= it->Size())
    {
      index -= it->Size();
      continue;
    }
    return it->Get(index);
  }
  return NULL;
}

util::tString tBlackboardManager::GetBlackboardNameFromQualifiedLink(const util::tString& qname)
{
  bool b = boost::starts_with(qname, cSLASHED_NAME);
  if (b && (boost::ends_with(qname, cREAD_POSTFIX) || boost::ends_with(qname, cWRITE_POSTFIX)))
  {
    util::tString qname2 = qname.substr(0, qname.rfind("/"));
    return qname2.substr(qname2.rfind("/") + 1);
  }
  return "";
}

tBlackboardManager* tBlackboardManager::GetInstance()
{
  if (instance == NULL && (!core::tRuntimeEnvironment::ShuttingDown()))
  {
    CreateBlackboardManager();  // should be okay with respect to double-checked locking
  }
  return instance;
}

size_t tBlackboardManager::GetNumberOfBlackboards(int category)
{
  int start_cat = category < 0 ? 0 : category;
  int end_cat = category < 0 ? cDIMENSION - 1 : start_cat;
  return GetNumberOfBlackboards(start_cat, end_cat);
}

size_t tBlackboardManager::GetNumberOfBlackboards(int start_cat, int end_cat)
{
  int result = 0;
  for (int c = start_cat; c <= end_cat; c++)
  {
    result += categories[c]->blackboards.Size();
  }
  return result;
}

void tBlackboardManager::PrepareDelete()
{
  core::tRuntimeEnvironment::GetInstance()->RemoveListener(*this);
  instance = NULL;
  ::finroc::core::tFrameworkElement::PrepareDelete();
}

void tBlackboardManager::RemoveClient(tRawBlackboardClient* client)
{
  {
    rrlib::thread::tLock lock2(bb_clients.GetMutex());
    bb_clients.Remove(client);
  }
  if (!client->AutoConnectClient())
  {
    return;
  }
  {
    rrlib::thread::tLock lock2(auto_connect_clients);
    auto_connect_clients.RemoveElem(client);
  }
}

void tBlackboardManager::RuntimeChange(int8 change_type, core::tFrameworkElement& element)
{
  if (change_type == ::finroc::core::tRuntimeListener::cADD)
  {
    // Is this a remote blackboard? -> Create proxy
    if (element.GetFlag(core::tCoreFlags::cNETWORK_ELEMENT) && element.GetFlag(core::tCoreFlags::cIS_PORT) && (!element.IsChildOf(*this)))
    {
      element.GetQualifiedLink(temp_buffer);
      util::tString qname = temp_buffer;
      util::tString name = GetBlackboardNameFromQualifiedLink(temp_buffer);
      bool read = boost::ends_with(qname, cREAD_POSTFIX);
      bool write = boost::ends_with(qname, cWRITE_POSTFIX);

      if (name.length() > 0)
      {
        tAbstractBlackboardServerRaw* info = GetBlackboard(name, cREMOTE, NULL);

        // okay create blackboard proxy
        bool add = (info == NULL);
        if (add)
        {
          info = new tRemoteBlackboardServer(name);
        }
        if (read && info->read_port_raw == NULL)
        {
          core::tPortBase& port = static_cast<core::tPortBase&>(element);
          info->read_port_raw = new core::tPortBase(core::tPortCreationInfoBase(cREAD_PORT_NAME, info, port.GetDataType(), core::tPortFlags::cOUTPUT_PROXY | core::tCoreFlags::cNETWORK_ELEMENT));
          info->Init();
          info->read_port_raw->ConnectToSource(qname);
        }
        else if (write && info->write_port_raw == NULL)
        {
          core::tInterfacePort& port = static_cast<core::tInterfacePort&>(element);
          info->write_port_raw = new core::tInterfacePort(cWRITE_PORT_NAME, info, port.GetDataType(), core::tInterfacePort::eRouting, core::tCoreFlags::cNETWORK_ELEMENT);
          info->Init();
          info->write_port_raw->ConnectToSource(qname);
        }
        CheckAutoConnect(info);
      }
    }
  }
}

tBlackboardManager::tBlackboardCategory::tBlackboardCategory(tBlackboardManager* const outer_class_ptr_, const util::tString& category_name, int default_flags_) :
  core::tFrameworkElement(outer_class_ptr_, category_name, default_flags_, -1),
  outer_class_ptr(outer_class_ptr_),
  default_flags(default_flags_),
  blackboards(100)
{
}

void tBlackboardManager::tBlackboardCategory::Add(tAbstractBlackboardServerRaw* blackboard)
{
  {
    rrlib::thread::tLock lock2(outer_class_ptr->auto_connect_clients);
    blackboards.Add(blackboard, false);
    outer_class_ptr->CheckAutoConnect(blackboard);
  }
}

void tBlackboardManager::tBlackboardCategory::CheckConnect(tRawBlackboardClient* client)
{
  util::tArrayWrapper<tAbstractBlackboardServerRaw*>* it = blackboards.GetIterable();
  for (size_t i = 0u; i < it->Size(); i++)
  {
    tAbstractBlackboardServerRaw* info = it->Get(i);
    if (client->CheckConnect(info))
    {
      return;
    }
  }
}

void tBlackboardManager::tBlackboardCategory::Remove(tAbstractBlackboardServerRaw* blackboard)
{
  {
    rrlib::thread::tLock lock2(outer_class_ptr->auto_connect_clients);
    blackboards.Remove(blackboard);
  }
}

/*! Frequency to check for locks */
static rrlib::time::tDuration cLOCK_CHECKER_CYCLE_TIME = std::chrono::milliseconds(250);

tBlackboardManager::tLockCheckerThread::tLockCheckerThread(tBlackboardManager* const outer_class_ptr_) :
  core::tCoreLoopThreadBase(cLOCK_CHECKER_CYCLE_TIME),
  outer_class_ptr(outer_class_ptr_)
{
  SetName("Blackboard Lock-Checker Thread");
}

void tBlackboardManager::tLockCheckerThread::MainLoopCallback()
{
  // send keep-alive signals
  util::tArrayWrapper<tRawBlackboardClient*>* it = outer_class_ptr->bb_clients.GetIterable();
  for (size_t i = 0u; i < it->Size(); i++)
  {
    tRawBlackboardClient* client = it->Get(i);
    if (client != NULL && client->IsReady())
    {
      client->SendKeepAlive();
    }
  }

  // check for outdated locks (do this for local and shared blackboards)
  for (int i = 0; i < 2; i++)
  {
    tBlackboardManager::tBlackboardCategory* cat = outer_class_ptr->GetCategory(i == 0 ? tBlackboardManager::cLOCAL : tBlackboardManager::cSHARED);
    util::tArrayWrapper<tAbstractBlackboardServerRaw*>* it2 = cat->blackboards.GetIterable();
    for (size_t j = 0u; j < it2->Size(); j++)
    {
      tAbstractBlackboardServerRaw* bb = it2->Get(j);
      if (bb != NULL && bb->IsReady())
      {
        bb->LockCheck();
      }
    }
  }

}

} // namespace finroc
} // namespace blackboard

