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
#include "blackboard/tAbstractBlackboardServer.h"
#include "blackboard/tRawBlackboardClient.h"

#include "blackboard/tBlackboardManager.h"
#include "core/tRuntimeEnvironment.h"
#include "core/tCoreFlags.h"
#include "finroc_core_utils/thread/sThreadUtil.h"
#include "core/plugin/tPlugins.h"
#include "blackboard/tBlackboardPlugin.h"
#include "blackboard/tRemoteBlackboardServer.h"
#include "core/port/std/tPortBase.h"
#include "core/port/rpc/tInterfacePort.h"

namespace finroc
{
namespace blackboard
{
util::tMutex tBlackboardManager::static_obj_synch;
const int tBlackboardManager::cALL, tBlackboardManager::cSHARED, tBlackboardManager::cLOCAL, tBlackboardManager::cREMOTE, tBlackboardManager::cDIMENSION;
util::tString tBlackboardManager::cNAME = "Blackboards";
util::tString tBlackboardManager::cSLASHED_NAME = util::tStringBuilder("/") + tBlackboardManager::cNAME + "/";
util::tString tBlackboardManager::cREAD_PORT_NAME = "read", tBlackboardManager::cWRITE_PORT_NAME = "write";
util::tString tBlackboardManager::cREAD_POSTFIX = util::tStringBuilder("/") + tBlackboardManager::cREAD_PORT_NAME, tBlackboardManager::cWRITE_POSTFIX = util::tStringBuilder("/") + tBlackboardManager::cWRITE_PORT_NAME;
tBlackboardManager* volatile tBlackboardManager::instance;

tBlackboardManager::tBlackboardManager() :
    core::tFrameworkElement(cNAME, core::tRuntimeEnvironment::GetInstance()),
    categories(cDIMENSION),
    temp_buffer(),
    bb_clients(10u, 4u),
    auto_connect_clients()
{
  categories[cLOCAL] = new tBlackboardCategory(this, "Local", core::tCoreFlags::cALLOWS_CHILDREN);
  categories[cSHARED] = new tBlackboardCategory(this, "Shared", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cSHARED | core::tCoreFlags::cGLOBALLY_UNIQUE_LINK);
  categories[cREMOTE] = new tBlackboardCategory(this, "Remote", core::tCoreFlags::cALLOWS_CHILDREN | core::tCoreFlags::cNETWORK_ELEMENT);
  tLockCheckerThread* checker = new tLockCheckerThread(this);
  util::sThreadUtil::SetAutoDelete(*checker);
  checker->Start();
}

void tBlackboardManager::AddClient(tRawBlackboardClient* client, bool auto_connect)
{
  {
    util::tLock lock2(bb_clients.obj_synch);
    bb_clients.Add(client, false);
  }
  if (!auto_connect)
  {
    return;
  }
  {
    util::tLock lock2(auto_connect_clients.obj_synch);
    auto_connect_clients.Add(client);

    for (int j = 0; (j < cDIMENSION) && (!client->IsConnected()); j++)
    {
      categories[j]->CheckConnect(client);
    }
  }
}

void tBlackboardManager::CheckAutoConnect(tAbstractBlackboardServer* server)
{
  if (server->read_port == NULL || server->write_port == NULL)
  {
    return;
  }
  {
    util::tLock lock2(auto_connect_clients.obj_synch);
    for (size_t i = 0u; i < auto_connect_clients.Size(); i++)
    {
      auto_connect_clients.Get(i)->CheckConnect(server);
    }
  }
}

void tBlackboardManager::CreateBlackboardManager()
{
  util::tLock lock1(static_obj_synch);
  if (instance == NULL)
  {
    instance = new tBlackboardManager();
    core::tRuntimeEnvironment::GetInstance()->AddListener(instance);

    // TODO do this properly
    core::tPlugins::GetInstance()->AddPlugin(new tBlackboardPlugin());
    // core::tPlugins::GetInstance()->AddPlugin(new tBlackboard2Plugin());
  }
}

tAbstractBlackboardServer* tBlackboardManager::GetBlackboard(const util::tString& name, int category, core::tDataType* type)
{
  int start_cat = category < 0 ? 0 : category;
  int end_cat = category < 0 ? cDIMENSION - 1 : start_cat;
  return GetBlackboard(name, start_cat, end_cat, type);
}

tAbstractBlackboardServer* tBlackboardManager::GetBlackboard(const util::tString& name, int start_cat, int end_cat, core::tDataType* type)
{
  for (int c = start_cat; c <= end_cat; c++)
  {
    tBlackboardCategory* cat = categories[c];
    util::tArrayWrapper<tAbstractBlackboardServer*>* it = cat->blackboards.GetIterable();
    for (size_t i = 0u; i < it->Size(); i++)
    {
      tAbstractBlackboardServer* info = it->Get(i);
      if (info->GetDescription().Equals(name) && (type == NULL || info->read_port->GetDataType() == type))
      {
        return info;
      }
    }
  }
  return NULL;
}

tAbstractBlackboardServer* tBlackboardManager::GetBlackboard(size_t index, int category)
{
  int start_cat = category < 0 ? 0 : category;
  int end_cat = category < 0 ? cDIMENSION - 1 : start_cat;
  return GetBlackboard(index, start_cat, end_cat);
}

tAbstractBlackboardServer* tBlackboardManager::GetBlackboard(size_t index, int start_cat, int end_cat)
{
  for (int c = start_cat; c <= end_cat; c++)
  {
    tBlackboardCategory* cat = categories[c];
    util::tArrayWrapper<tAbstractBlackboardServer*>* it = cat->blackboards.GetIterable();
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
  bool b = qname.StartsWith(cSLASHED_NAME);
  if (b && (qname.EndsWith(cREAD_POSTFIX) || qname.EndsWith(cWRITE_POSTFIX)))
  {
    util::tString qname2 = qname.Substring(0, qname.LastIndexOf("/"));
    return qname2.Substring(qname2.LastIndexOf("/") + 1);
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
  core::tRuntimeEnvironment::GetInstance()->RemoveListener(this);
  instance = NULL;
  ::finroc::core::tFrameworkElement::PrepareDelete();
}

void tBlackboardManager::RemoveClient(tRawBlackboardClient* client)
{
  {
    util::tLock lock2(bb_clients.obj_synch);
    bb_clients.Add(client, false);
  }
  if (!client->AutoConnectClient())
  {
    return;
  }
  {
    util::tLock lock2(auto_connect_clients.obj_synch);
    auto_connect_clients.RemoveElem(client);
  }
}

void tBlackboardManager::RuntimeChange(int8 change_type, core::tFrameworkElement* element)
{
  if (change_type == ::finroc::core::tRuntimeListener::cREMOVE || change_type == ::finroc::core::tRuntimeListener::cPRE_INIT)
  {
    // Is this a remote blackboard?
    if (element->GetFlag(core::tCoreFlags::cNETWORK_ELEMENT) && element->GetFlag(core::tCoreFlags::cIS_PORT))
    {
      element->GetQualifiedLink(temp_buffer);
      util::tString qname = temp_buffer.ToString();
      util::tString name = GetBlackboardNameFromQualifiedLink(temp_buffer.ToString());
      bool read = qname.EndsWith(cREAD_POSTFIX);
      bool write = qname.EndsWith(cWRITE_POSTFIX);

      if (name.Length() > 0)
      {
        tAbstractBlackboardServer* info = GetBlackboard(name, cREMOTE, NULL);

        if (change_type == ::finroc::core::tRuntimeListener::cPRE_INIT)
        {
          // okay create/this blackboard
          bool add = (info == NULL);
          if (add)
          {
            info = new tRemoteBlackboardServer(name);
          }
          if (read)
          {
            assert((info->read_port == NULL));
            info->read_port = static_cast<core::tPortBase*>(element);
            info->read_port->Link(info, cREAD_PORT_NAME);
          }
          else if (write)
          {
            assert((info->write_port == NULL));
            info->write_port = static_cast<core::tInterfacePort*>(element);
            info->write_port->Link(info, cWRITE_PORT_NAME);
          }
          if (add)
          {
            info->Init();
          }
          CheckAutoConnect(info);

        }
        else if (change_type == ::finroc::core::tRuntimeListener::cREMOVE)
        {
          // okay delete this blackboard
          assert(((info != NULL)) && "Remote Blackboard deleted that was never created");
          if (read)
          {
            assert((info->read_port != NULL));
            info->read_port = NULL;
          }
          else if (write)
          {
            assert((info->write_port != NULL));
            info->write_port = NULL;
          }
          if (info->read_port == NULL && info->write_port == NULL)
          {
            info->ManagedDelete();
          }
        }
      }
    }
  }
}

tBlackboardManager::tBlackboardCategory::tBlackboardCategory(tBlackboardManager* const outer_class_ptr_, const util::tString& category_name, int default_flags_) :
    core::tFrameworkElement(category_name, outer_class_ptr_),
    outer_class_ptr(outer_class_ptr_),
    default_flags(default_flags_),
    blackboards(100u, 4u)
{
}

void tBlackboardManager::tBlackboardCategory::Add(tAbstractBlackboardServer* blackboard)
{
  {
    util::tLock lock2(outer_class_ptr->auto_connect_clients.obj_synch);
    blackboards.Add(blackboard, false);
    outer_class_ptr->CheckAutoConnect(blackboard);
  }
}

void tBlackboardManager::tBlackboardCategory::CheckConnect(tRawBlackboardClient* client)
{
  util::tArrayWrapper<tAbstractBlackboardServer*>* it = blackboards.GetIterable();
  for (size_t i = 0u; i < it->Size(); i++)
  {
    tAbstractBlackboardServer* info = it->Get(i);
    if (client->CheckConnect(info))
    {
      return;
    }
  }
}

void tBlackboardManager::tBlackboardCategory::Remove(tAbstractBlackboardServer* blackboard)
{
  {
    util::tLock lock2(outer_class_ptr->auto_connect_clients.obj_synch);
    blackboards.Remove(blackboard);
  }
}

const int tBlackboardManager::tLockCheckerThread::cCYCLE_TIME;

tBlackboardManager::tLockCheckerThread::tLockCheckerThread(tBlackboardManager* const outer_class_ptr_) :
    core::tCoreLoopThreadBase(cCYCLE_TIME),
    outer_class_ptr(outer_class_ptr_)
{
  SetName("Blackboard Lock-Checker Thread");
  SetDaemon(true);
}

void tBlackboardManager::tLockCheckerThread::MainLoopCallback()
{
  // send keep-alive signals
  util::tArrayWrapper<tRawBlackboardClient*>* it = outer_class_ptr->bb_clients.GetIterable();
  for (size_t i = 0u; i < it->Size(); i++)
  {
    tRawBlackboardClient* client = it->Get(i);
    client->SendKeepAlive();
  }

  // check for outdated locks (do this for local and shared blackboards)
  for (int i = 0; i < 2; i++)
  {
    tBlackboardManager::tBlackboardCategory* cat = outer_class_ptr->GetCategory(i == 0 ? tBlackboardManager::cLOCAL : tBlackboardManager::cSHARED);
    util::tArrayWrapper<tAbstractBlackboardServer*>* it2 = cat->blackboards.GetIterable();
    for (size_t j = 0u; j < it2->Size(); j++)
    {
      tAbstractBlackboardServer* bb = it2->Get(j);
      if (bb != NULL && bb->IsReady())
      {
        bb->LockCheck();
      }
    }
  }

}

} // namespace finroc
} // namespace blackboard

