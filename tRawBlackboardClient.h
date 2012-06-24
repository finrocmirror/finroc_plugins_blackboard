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

#ifndef plugins__blackboard__tRawBlackboardClient_h__
#define plugins__blackboard__tRawBlackboardClient_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/serialization/tMemoryBuffer.h"
#include "rrlib/rtti/tDataTypeBase.h"

#include "core/port/tPortCreationInfoBase.h"
#include "core/port/tPortFlags.h"
#include "core/tFrameworkElement.h"
#include "core/port/std/tPortBase.h"
#include "core/port/rpc/tInterfaceClientPort.h"

#include "plugins/blackboard/tAbstractBlackboardServer.h"
#include "plugins/blackboard/tAbstractBlackboardServerRaw.h"


namespace finroc
{
namespace blackboard
{
/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard client
 */
class tRawBlackboardClient : public core::tFrameworkElement
{
public:

public:

  enum tLockType { eNONE, eREAD, eWRITE };

  enum tServerBuffers { eUNKNOWN, eMULTI, eSINGLE };

public:
  /*implements ReturnHandler*/

  class tReadPort : public core::tPortBase
  {
  private:

    // Outer class RawBlackboardClient
    tRawBlackboardClient* const outer_class_ptr;

  protected:

    virtual void ConnectionRemoved(core::tAbstractPort* partner)
    {
      outer_class_ptr->server_buffers = tRawBlackboardClient::eUNKNOWN;
    }

    virtual void NewConnection(core::tAbstractPort* partner);

  public:

    tReadPort(tRawBlackboardClient* const outer_class_ptr_, core::tPortCreationInfoBase pci);

    inline tRawBlackboardClient* GetBBClient()
    {
      return outer_class_ptr;
    }

  };

public:

  class tWritePort : public core::tInterfaceClientPort
  {
  private:

    // Outer class RawBlackboardClient
    tRawBlackboardClient* const outer_class_ptr;

  protected:

    virtual void ConnectionRemoved(core::tAbstractPort* partner)
    {
      outer_class_ptr->server_buffers = tRawBlackboardClient::eUNKNOWN;
    }

    virtual void NewConnection(core::tAbstractPort* partner);

  public:

    tWritePort(tRawBlackboardClient* const outer_class_ptr_, rrlib::rtti::tDataTypeBase type);

    inline tRawBlackboardClient* GetBBClient()
    {
      return outer_class_ptr;
    }

  };

  template<typename T>
  friend class tBlackboardClient;

protected:

  //Pointers to methods to use
  template <typename T>
  static int8 CallIsSingleBuffered(tWritePort* port)
  {
    return tAbstractBlackboardServer<T>::cIS_SINGLE_BUFFERED.Call(*port, cNET_TIMEOUT);
  }

  template <typename T>
  static void CallKeepAlive(tWritePort* port, int lockid)
  {
    tAbstractBlackboardServer<T>::cKEEP_ALIVE.Call(*port, lockid);
  }

  int8(*is_single_buffered_func)(tWritePort*);
  void (*keep_alive_func)(tWritePort*, int);

  /*! Interface port for write access */
  std::shared_ptr<tWritePort> write_port;

  /*! Port for reading */
  tReadPort* read_port;

  /*! Is there currently a lock? */
  tRawBlackboardClient::tLockType lock_type;

  /*! ID of current locking operation */
  volatile int cur_lock_id;

  /*! Auto-Connect blackboard client to matching server? */
  bool auto_connect;

  /*! If auto-connect is active: Limit auto-connecting to a specific blackboard category? (-1 is no) */
  int auto_connect_category;

  tRawBlackboardClient::tServerBuffers server_buffers;

public:

  /*! Default network timeout (added to any other timeouts for network calls) */
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6))
  static constexpr rrlib::time::tDuration cNET_TIMEOUT = std::chrono::seconds(1);
#else
  static rrlib::time::tDuration cNET_TIMEOUT;
#endif

protected:

  /*! Check whether we are dealing with a single buffered blackboard server */
  void CheckSingleBuffered();

  virtual void PostChildInit();

  virtual void PrepareDelete();

public:

  /*!
   * \param pci PortCreationInfo (relevant info: name (of blackboard), parent (of client), type (data type of blackboard content),
   * flags (emit data => write port, accept data => read port
   * \param auto_connect Auto-Connect blackboard client to matching server?
   * \param auto_connect_category If auto-connect is active: Limit auto-connecting to a specific blackboard category? (-1 is no)
   */
  template <typename T>
  tRawBlackboardClient(core::tPortCreationInfoBase pci, T* t, bool auto_connect_ = true, int auto_connect_category_ = -1) :
    core::tFrameworkElement(pci.parent, pci.name),
    is_single_buffered_func(CallIsSingleBuffered<T>),
    keep_alive_func(CallKeepAlive<T>),
    write_port(pci.GetFlag(core::tPortFlags::cEMITS_DATA) ? new tWritePort(this, tAbstractBlackboardServerRaw::GetBlackboardTypeInfo(pci.data_type)->blackboard_type) : NULL),
    read_port(pci.GetFlag(core::tPortFlags::cACCEPTS_DATA) ? new tReadPort(this, core::tPortCreationInfoBase("read", this, pci.data_type.GetListType(), core::tPortFlags::cACCEPTS_DATA | (pci.flags & core::tPortFlags::cPUSH_STRATEGY))) : NULL),
    lock_type(eNONE),
    cur_lock_id(-1),
    auto_connect(auto_connect_),
    auto_connect_category(auto_connect_category_),
    server_buffers(eUNKNOWN)
  {
    tAbstractBlackboardServerRaw::CheckType(pci.data_type);
  }

  /*!
   * \return Auto-connect client?
   */
  inline bool AutoConnectClient()
  {
    return auto_connect;
  }

  /*!
   * Check whether this auto-connecting client wants to auto-connect to this server
   *
   * \param server Server that is a candidate for connecting
   * \return True if client is connected
   */
  bool CheckConnect(tAbstractBlackboardServerRaw* server);

  /*!
   * \return Default ProtCreationInfo for Blackboard clients (creates both read write ports)
   */
  inline static core::tPortCreationInfoBase GetDefaultPci()
  {
    static core::tPortCreationInfoBase default_pci(rrlib::rtti::tDataType<rrlib::serialization::tMemoryBuffer>(), core::tPortFlags::cEMITS_DATA | core::tPortFlags::cACCEPTS_DATA);
    return default_pci;
  }

  inline tReadPort* GetReadPort()
  {
    return read_port;
  }

  inline tWritePort* GetWritePort()
  {
    return write_port.get();
  }

  inline bool HasLock()
  {
    return lock_type != eNONE;
  }

  inline bool HasReadLock()
  {
    return lock_type == eREAD;
  }

  inline bool HasWriteLock()
  {
    return lock_type == eWRITE;
  }

  /*!
   * (relevant mainly for auto-connect)
   *
   * \return Is client connected to blackboard server?
   */
  inline bool IsConnected() const
  {
    bool w = (write_port.get() == NULL) ? true : write_port->IsConnected();
    return w && read_port->IsConnected();
  }

  /*!
   * Send keep-alive signal to server (usually done automatically...)
   */
  void SendKeepAlive();

};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tRawBlackboardClient_h__
