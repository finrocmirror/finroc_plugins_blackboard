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
/*!\file    plugins/blackboard/internal/tLockParameters.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-30
 *
 * \brief   Contains tLockParameters
 *
 * \b tLockParameters
 *
 * Parameter for blackboard write lock.
 * Main purpose of using this class is the ability to detect remote calls automatically.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__blackboard__internal__tLockParameters_h__
#define __plugins__blackboard__internal__tLockParameters_h__

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
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Parameters for write lock
/*!
 * Parameter for blackboard write lock.
 * Main purpose of using this class is the ability to detect remote calls automatically.
 */
class tLockParameters
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tLockParameters() :
    timeout(),
    remote_call(false)
  {}

  /*!
   * \param timeout Lock timeout in ms
   */
  tLockParameters(const rrlib::time::tDuration& timeout) :
    timeout(timeout),
    remote_call(false)
  {}

  /*!
   * \return Lock timeout in ms
   */
  rrlib::time::tDuration GetTimeout() const
  {
    return timeout;
  }

  /*!
   * \return Is this lock call originating from a remote runtime?
   */
  bool IsRemoteCall() const
  {
    return remote_call;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tLockParameters& parameters);

  /*! Lock timeout in ms */
  rrlib::time::tDuration timeout;

  /*! Is this lock call originating from a remote runtime? */
  bool remote_call;
};

inline rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tLockParameters& parameters)
{
  stream << parameters.GetTimeout();
  return stream;
}

inline rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tLockParameters& parameters)
{
  stream >> parameters.timeout;
  parameters.remote_call = true;
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
