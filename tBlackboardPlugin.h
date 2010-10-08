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
#include "finroc_core_utils/tJCBase.h"
#include "core/portdatabase/tDataType.h"

#ifndef PLUGINS__BLACKBOARD__TBLACKBOARDPLUGIN_H
#define PLUGINS__BLACKBOARD__TBLACKBOARDPLUGIN_H

#include "finroc_core_utils/container/tReusablesPoolCR.h"
#include "core/portdatabase/tDataTypeRegister.h"
#include "blackboard/tAbstractBlackboardServer.h"
#include "core/plugin/tPlugin.h"

namespace finroc
{
namespace core
{
class tPluginManager;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
class tBlackboardTask;

/*!
 * \author Max Reichardt
 *
 * Object to initialize the blackboard 2 mechanism
 */
class tBlackboardPlugin : public util::tUncopyableObject, public core::tPlugin
{
public:

  //  /** Marks copy-on-write blackboard server ports */
  //  public static int SINGLE_BUFFERED = PortFlags.FIRST_CUSTOM_PORT_FLAG;

  /*! Reusable blackboard tasks */
  static util::tReusablesPoolCR<tBlackboardTask>* task_pool;

  tBlackboardPlugin() {}

  virtual void Init(core::tPluginManager& mgr);

  //wrapper for below
  template <typename T>
  static core::tDataType* RegisterBlackboardType(const finroc::util::tString& name)
  {
    return RegisterBlackboardType<T>(finroc::util::tTypedClass<T>(), name);
  }

  /*!
   * Registers blackboard data type
   * (actually two: one for buffer and one for method calls)
   *
   * \param clazz Type
   * \param name Blackboard buffer type name
   * \return Blackboard buffer type
   */
  template <typename T>
  inline static core::tDataType* RegisterBlackboardType(finroc::util::tTypedClass<T> clazz, const util::tString& name)
  {
    core::tDataType* dt = core::tDataTypeRegister::GetInstance()->GetDataType(clazz, name);
    core::tDataType* mc = core::tDataTypeRegister::GetInstance()->AddMethodDataType(name + " blackboard method calls", &(tAbstractBlackboardServer::cMETHODS));
    dt->SetRelatedType(mc);
    mc->SetRelatedType(dt);
    return dt;
  }

  /*!
   * Registers blackboard data type
   * (actually two: one for buffer and one for method calls)
   *
   * \param clazz Type
   * \return Blackboard buffer type
   */
  template <typename T>
  inline static core::tDataType* RegisterBlackboardType(finroc::util::tTypedClass<T> clazz)
  {
    return RegisterBlackboardType(clazz, core::tDataTypeRegister::GetCleanClassName(clazz));
  }

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TBLACKBOARDPLUGIN_H
