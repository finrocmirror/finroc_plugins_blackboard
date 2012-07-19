//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    mBlackboardWriterAsync.h
 *
 * \author  Max Reichardt
 *
 * \date    2011-03-30
 *
 * \brief Contains mBlackboardWriterAsync
 *
 * \b mBlackboardWriterAsync
 *
 */
//----------------------------------------------------------------------
#ifndef _blackboard__mBlackboardWriterAsync_h_
#define _blackboard__mBlackboardWriterAsync_h_

#include "core/structure/tModule.h"
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
class mBlackboardWriterAsync : public finroc::core::structure::tModule
{
  static finroc::core::tStandardCreateModuleAction<mBlackboardWriterAsync> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tBlackboardClient<float> bb_client;

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  mBlackboardWriterAsync(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name = "BlackboardWriterAsync");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  int update_counter;

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
