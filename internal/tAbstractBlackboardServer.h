//
// You received this file as part of Finroc
// A Framework for intelligent robot control
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
/*!\file    plugins/blackboard/internal/tAbstractBlackboardServer.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-19
 *
 * \brief   Contains tAbstractBlackboardServer
 *
 * \b tAbstractBlackboardServer
 *
 * Abstract base class of all variants of blackboard servers.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tAbstractBlackboardServer_h__
#define __plugins__blackboard__internal__tAbstractBlackboardServer_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElement.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace blackboard
{
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Blackboard Server base class
/*!
 * Abstract base class of all variants of blackboard servers
 * (non-template base class to reduce size of compiled code)
 */
class tAbstractBlackboardServer : public core::tFrameworkElement
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tAbstractBlackboardServer(core::tFrameworkElement* parent, const std::string& name, tFrameworkElement::tFlags flags = tFlags());

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*!
   * \return Mutex for blackboard operations
   */
  rrlib::thread::tOrderedMutex& BlackboardMutex()
  {
    return blackboard_mutex;
  }

  virtual void PrepareDelete() {} // TODO: mark override in gcc 4.7

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*!
   * Mutex for blackboard operations
   * (needs to be deeper than runtime - (for initial pushes etc.))
   */
  rrlib::thread::tOrderedMutex blackboard_mutex;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
