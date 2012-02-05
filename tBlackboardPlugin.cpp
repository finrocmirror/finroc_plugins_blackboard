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
#include "plugins/blackboard/tBlackboardPlugin.h"
#include "rrlib/serialization/tMemoryBuffer.h"
#include "plugins/blackboard/tBlackboardBuffer.h"
#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "plugins/blackboard/tBlackboardTypeInfo.h"

namespace finroc
{
namespace blackboard
{
rrlib::rtti::tDataTypeBase tBlackboardPlugin::cBB_MEM_BUFFER = RegisterBlackboardType(util::tTypedClass<rrlib::serialization::tMemoryBuffer>());
rrlib::rtti::tDataTypeBase tBlackboardPlugin::cBB_BLACKBOARD_BUFFER = RegisterBlackboardType(util::tTypedClass<tBlackboardBuffer>());

void tBlackboardPlugin::Init()
{
  //        taskPool = new ReusablesPoolCR<BlackboardTask>();
  //        AutoDeleter.addStatic(taskPool);
  //        taskPool = new ReusablesPoolCR<BlackboardTask>();
  //        AutoDeleter.addStatic(taskPool);
}

} // namespace finroc
} // namespace blackboard

