/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2011 Max Reichardt,
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
#include "rrlib/serialization/tMemoryBuffer.h"
#include "plugins/blackboard/tBlackboardBuffer.h"
#include "plugins/blackboard/tBlackboardPlugin.h"
#include "plugins/blackboard/tBlackboardServer.h"
#include "plugins/blackboard/tBlackboardClient.h"
#include "plugins/blackboard/tSingleBufferedBlackboardServer.h"

namespace finroc
{
namespace blackboard
{

template class tBlackboardClient<tBlackboardBuffer>;
template class tBlackboardClient<rrlib::serialization::tMemoryBuffer>;
template class tBlackboardServer<tBlackboardBuffer>;
template class tBlackboardServer<rrlib::serialization::tMemoryBuffer>;
template class tSingleBufferedBlackboardServer<tBlackboardBuffer>;
template class tSingleBufferedBlackboardServer<rrlib::serialization::tMemoryBuffer>;

} // namespace finroc
} // namespace blackboard
