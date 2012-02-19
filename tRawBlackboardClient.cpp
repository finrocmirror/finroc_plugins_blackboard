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
#include "rrlib/finroc_core_utils/log/tLogUser.h"

#include "core/port/rpc/tInterfacePort.h"
#include "core/port/rpc/tMethodCallException.h"
#include "core/port/tAbstractPort.h"

#include "plugins/blackboard/tRawBlackboardClient.h"
#include "plugins/blackboard/tBlackboardManager.h"

namespace finroc
{
namespace blackboard
{
const int tRawBlackboardClient::cNET_TIMEOUT;

bool tRawBlackboardClient::CheckConnect(tAbstractBlackboardServerRaw* server)
{
  assert((auto_connect));
  if (server->read_port_raw == NULL || server->write_port_raw == NULL)
  {
    return false;
  }
  if (IsConnected())
  {
    return true;  // already connected
  }
  if (auto_connect_category >= 0 && auto_connect_category != server->category_index)
  {
    return false;  // wrong category
  }
  if (read_port != NULL && server->read_port_raw->GetDataType() != read_port->GetDataType())
  {
    return false;  // data types don't fit
  }
  if (write_port.get() != NULL && server->write_port_raw->GetDataType() != write_port->GetDataType())
  {
    return false;  // data types don't fit
  }
  if (!GetName().Equals(server->GetName()))
  {
    return false;  // names don't match
  }

  // checks passed => connect
  if (read_port != NULL)
  {
    read_port->ConnectToSource(server->read_port_raw);
  }
  if (write_port.get() != NULL)
  {
    write_port->ConnectToSource(server->write_port_raw);
  }
  return true;
}

void tRawBlackboardClient::CheckSingleBuffered()
{
  if (server_buffers != eUNKNOWN)
  {
    return;
  }
  try
  {
    int8 result = (*is_single_buffered_func)(write_port.get());
    server_buffers = (result == 0) ? eMULTI : eSINGLE;
  }
  catch (const core::tMethodCallException& e)
  {
    server_buffers = eUNKNOWN;
  }
}

void tRawBlackboardClient::PostChildInit()
{
  ::finroc::core::tFrameworkElement::PostChildInit();
  assert(((tBlackboardManager::GetInstance() != NULL)) && "truly strange");
  tBlackboardManager::GetInstance()->AddClient(this, auto_connect);
}

void tRawBlackboardClient::PrepareDelete()
{
  tBlackboardManager* instance = tBlackboardManager::GetInstance();
  if (instance != NULL)    // we don't need to remove it, if blackboard manager has already been deleted
  {
    instance->RemoveClient(this);
  }
  ::finroc::core::tFrameworkElement::PrepareDelete();
}

void tRawBlackboardClient::SendKeepAlive()
{
  int cur_lock_iD = this->cur_lock_iD;
  if (cur_lock_iD >= 0)
  {
    try
    {
      (*keep_alive_func)(write_port.get(), cur_lock_iD);

    }
    catch (const core::tMethodCallException& e)
    {
      FINROC_LOG_PRINT(rrlib::logging::eLL_WARNING, "warning: Sending Keep-Alive failed");
    }
  }
}

tRawBlackboardClient::tReadPort::tReadPort(tRawBlackboardClient* const outer_class_ptr_, core::tPortCreationInfoBase pci) :
  core::tPortBase(pci),
  outer_class_ptr(outer_class_ptr_)
{
}

void tRawBlackboardClient::tReadPort::NewConnection(core::tAbstractPort* partner)
{
  ::finroc::core::tAbstractPort::NewConnection(partner);
  if (outer_class_ptr->write_port.get() != NULL)
  {
    ::finroc::core::tFrameworkElement* w = partner->GetParent()->GetChild("write");
    if (w != NULL)
    {
      outer_class_ptr->write_port->ConnectToSource(static_cast< ::finroc::core::tAbstractPort*>(w));
    }
  }
  outer_class_ptr->server_buffers = tRawBlackboardClient::eUNKNOWN;
}

tRawBlackboardClient::tWritePort::tWritePort(tRawBlackboardClient* const outer_class_ptr_, rrlib::rtti::tDataTypeBase type) :
  core::tInterfaceClientPort("write", outer_class_ptr_, type),
  outer_class_ptr(outer_class_ptr_)
{
}

void tRawBlackboardClient::tWritePort::NewConnection(core::tAbstractPort* partner)
{
  ::finroc::core::tInterfaceClientPort::NewConnection(partner);
  if (outer_class_ptr->read_port != NULL)
  {
    core::tFrameworkElement* w = partner->GetParent()->GetChild("read");
    if (w != NULL)
    {
      outer_class_ptr->read_port->ConnectToSource(static_cast<core::tAbstractPort*>(w));
    }
  }
  outer_class_ptr->server_buffers = tRawBlackboardClient::eUNKNOWN;
}

} // namespace finroc
} // namespace blackboard

