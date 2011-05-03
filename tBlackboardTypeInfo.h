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

#ifndef plugins__blackboard__tBlackboardTypeInfo_h__
#define plugins__blackboard__tBlackboardTypeInfo_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "rrlib/serialization/tDataTypeBase.h"
#include "rrlib/serialization/tDataTypeAnnotation.h"

namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * Additional type info for blackboards
 */
class tBlackboardTypeInfo : public rrlib::serialization::tDataTypeAnnotation
{
public:

  /*! Blackboard (method) type */
  rrlib::serialization::tDataTypeBase blackboard_type;

  /*! Blackboard element type */
  rrlib::serialization::tDataTypeBase element_type;

  tBlackboardTypeInfo() :
      blackboard_type(),
      element_type()
  {}
};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardTypeInfo_h__
