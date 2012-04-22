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

#ifndef plugins__blackboard__tBlackboardPlugin_h__
#define plugins__blackboard__tBlackboardPlugin_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/rtti/rtti.h"

#include "core/plugin/tPlugin.h"
#include "core/portdatabase/tRPCInterfaceType.h"

namespace finroc
{
namespace blackboard
{
template<typename T>
class tAbstractBlackboardServer;

/*!
 * \author Max Reichardt
 *
 * Object to initialize the blackboard mechanism
 */
class tBlackboardPlugin : public core::tPlugin
{
public:

  tBlackboardPlugin() {}

  virtual void Init();

  //wrapper for below
  template <typename T>
  static rrlib::rtti::tDataTypeBase RegisterBlackboardType(const finroc::util::tString& name)
  {
    return RegisterBlackboardType<T>(rrlib::rtti::tDataType<T>(), name);
  }

  /*!
   * Registers blackboard data type
   * (actually two: one for buffer and one for method calls)
   *
   * \param dt Data type to create blackboard type for
   * \return Blackboard buffer type
   */
  template <typename T>
  inline static rrlib::rtti::tDataTypeBase RegisterBlackboardType(rrlib::rtti::tDataTypeBase dt)
  {
    return tBlackboardPlugin::RegisterBlackboardType<T>(dt, dt.GetName());
  }

  /*!
   * Registers blackboard data type
   * (actually two: one for buffer and one for method calls)
   *
   * \param dt Data type to create blackboard type for
   * \param name Blackboard buffer type name
   * \return Blackboard buffer type
   */
  template <typename T>
  static rrlib::rtti::tDataTypeBase RegisterBlackboardType(rrlib::rtti::tDataTypeBase dt, const util::tString& name);

  /*!
   * Registers blackboard data type
   * (actually two: one for buffer and one for method calls)
   *
   * \param clazz Type
   * \return Blackboard buffer type
   */
  template <typename T>
  inline static rrlib::rtti::tDataTypeBase RegisterBlackboardType()
  {
    return RegisterBlackboardType<T>(rrlib::rtti::tDataTypeBase::GetDataTypeNameFromRtti(typeid(T).name()));
  }

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tAbstractBlackboardServer.h"

namespace finroc
{
namespace blackboard
{
template <typename T>
rrlib::rtti::tDataTypeBase tBlackboardPlugin::RegisterBlackboardType(rrlib::rtti::tDataTypeBase dt, const util::tString& name)
{
  util::tString bb_name = std::string("Blackboard<") + name + ">";
  rrlib::rtti::tDataTypeBase dtbb = rrlib::rtti::tDataTypeBase::FindType(bb_name);
  if (dtbb == NULL)
  {
    core::tPortInterface* methods = &tAbstractBlackboardServer<T>::GetBlackboardInterface();
    methods->Clear();
    methods->AddMethod(&tAbstractBlackboardServer<T>::cLOCK);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cREAD_LOCK);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cUNLOCK);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cREAD_UNLOCK);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cASYNCH_CHANGE);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cDIRECT_COMMIT);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cIS_SINGLE_BUFFERED);
    methods->AddMethod(&tAbstractBlackboardServer<T>::cKEEP_ALIVE);

    core::tRPCInterfaceType rpct(bb_name, methods);
    dtbb = rpct;

    // add annotation to element type
    tBlackboardTypeInfo* bti = new tBlackboardTypeInfo();
    bti->blackboard_type = dtbb;
    dt.AddAnnotation(bti);

    // add annotation to blackboard type
    tBlackboardTypeInfo* btibb = new tBlackboardTypeInfo();
    btibb->element_type = dt;
    dtbb.AddAnnotation(btibb);
  }

  return dtbb;
}

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardPlugin_h__
