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
/*!\file    plugins/blackboard/test/mBlackboardWriterAsync.h
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 * \brief   Contains mBlackboardWriterAsync
 *
 * \b mBlackboardWriterAsync
 *
 */
//----------------------------------------------------------------------
#ifndef _blackboard__mBlackboardWriterAsync_h_
#define _blackboard__mBlackboardWriterAsync_h_

#include "plugins/structure/tModule.h"
#include "plugins/blackboard/tBlackboardClient.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
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
// Class declaration
//----------------------------------------------------------------------
//! Writes entries [15..17] using async change commands.
/*!
 * Note, that these transaction-like changes do not require any locking
 * and do not block.
 */
class mBlackboardWriterAsync : public structure::tModule
{
  static runtime_construction::tStandardCreateModuleAction<mBlackboardWriterAsync> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tBlackboardClient<float> bb_client;

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  mBlackboardWriterAsync(core::tFrameworkElement *parent, const std::string &name = "BlackboardWriterAsync");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  int update_counter;

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
