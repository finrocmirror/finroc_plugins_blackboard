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
/*!\file    plugins/blackboard/definitions.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-31
 *
 * \brief
 *
 * Various definitions for blackboard plugin.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__definitions_h__
#define __plugins__blackboard__definitions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

/*! Desired read port configuration (constructor parameter) */
enum class tReadPorts
{
  NONE,     //!< Do not create any read ports
  INTERNAL, //!< Create read port in internal client
  EXTERNAL, //!< Create read port in internal client and also in group's/module's external interface (e.g. "Output" or "Sensor Output")
};

/*!
 * Auto-connect mode for clients of global blackboards
 *
 * Same categories of blackboard as in MCA2.
 * (Questionable whether that makes sense.
 *  Is, however, the easiest and most efficient way to remain compatible)
 */
enum class tAutoConnectMode
{
  OFF,      //!< Do not connect to any global blackboards
  ALL,      //!< Connect to any global blackboard with the same name
  SHARED,   //!< Connect to any shared global blackboard with the same name
  LOCAL,    //!< Connect to any local global blackboard with the same name
  REMOTE,   //!< Connect to any remote global blackboard with the same name
};


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
