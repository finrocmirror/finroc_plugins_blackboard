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
#include "rrlib/finroc_core_utils/tJCBase.h"

#ifndef PLUGINS__BLACKBOARD__TRAWBLACKBOARDCLIENT_H
#define PLUGINS__BLACKBOARD__TRAWBLACKBOARDCLIENT_H

#include "core/port/tPortCreationInfo.h"
#include "plugins/blackboard/tBlackboardBuffer.h"
#include "core/port/tThreadLocalCache.h"
#include "core/tFrameworkElement.h"
#include "core/port/std/tPortBase.h"
#include "core/port/rpc/tInterfaceClientPort.h"

namespace finroc
{
namespace core
{
class tAbstractPort;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
class tAbstractBlackboardServer;

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

    tReadPort(tRawBlackboardClient* const outer_class_ptr_, core::tPortCreationInfo pci);

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

    tWritePort(tRawBlackboardClient* const outer_class_ptr_, core::tDataType* type);

    inline tRawBlackboardClient* GetBBClient()
    {
      return outer_class_ptr;
    }

  };

private:

  tRawBlackboardClient::tServerBuffers server_buffers;

protected:

  /*! valid IDs of methods */
  //public static final byte LOCK = 1, ASYNCH_CHANGE = 2, UNLOCK = 3, READ_PART = 4, DIRECT_COMMIT = 5, DEPRECATED_DIRECT_BUFFER_ACCESS = 6, IS_SINGLE_BUFFERED = 7;

  /*! Interface port for write access */
  tWritePort* write_port;

  /*! Port for reading */
  tReadPort* read_port;

  /*! not null - if buffer is currently locked for writing */
  tBlackboardBuffer* locked;

  /*! not null - if buffer is currently locked for writing */
  const tBlackboardBuffer* read_locked;

  /*! Is there currently a lock? */
  tRawBlackboardClient::tLockType lock_type;

  /*! ID of current locking operation */
  volatile int cur_lock_iD;

  /*! Auto-Connect blackboard client to matching server? */
  bool auto_connect;

  /*! If auto-connect is active: Limit auto-connecting to a specific blackboard category? (-1 is no) */
  int auto_connect_category;

public:

  /*! Default network timeout (added to any other timeouts for network calls) */
  int cNET_TIMEOUT;

private:

  /*! Check whether we are dealing with a single buffered blackboard server */
  void CheckSingleBuffered();

protected:

  virtual void PostChildInit();

  virtual void PrepareDelete();

public:

  /*!
   * \param pci PortCreationInfo (relevant info: description (blackboard name), parent (of client), type (data type of blackboard content),
   * flags (emit data => write port, accept data => read port
   * \param auto_connect Auto-Connect blackboard client to matching server?
   * \param auto_connect_category If auto-connect is active: Limit auto-connecting to a specific blackboard category? (-1 is no)
   */
  tRawBlackboardClient(core::tPortCreationInfo pci, bool auto_connect_ = true, int auto_connect_category_ = -1);

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
  bool CheckConnect(tAbstractBlackboardServer* server);

  /*!
   * Commit asynchronous change to blackboard. Blackboard does
   * not need to be locked for this operation.
   * (if connection is broken, there's no guarantee that this will work or that an exception is thrown otherwise)
   *
   * \param offset Offset in byte in blackboard
   * \param change_buf Contents to write to this position (unlocked buffer retrieved via getUnusedBuffer OR a used buffer with an additional lock)
   */
  void CommitAsynchChange(int offset, const tBlackboardBuffer* change_buf);

  /*!
   * \return Default ProtCreationInfo for Blackboard clients (creates both read write ports)
   */
  inline static core::tPortCreationInfo GetDefaultPci()
  {
    static core::tPortCreationInfo default_pci(tBlackboardBuffer::cBUFFER_TYPE, core::tPortFlags::cEMITS_DATA | core::tPortFlags::cACCEPTS_DATA);
    return default_pci;
  }

  inline tReadPort* GetReadPort()
  {
    return read_port;
  }

  /*!
   * \return unused buffer - may be published/committed directly
   */
  inline tBlackboardBuffer* GetUnusedBuffer()
  {
    return static_cast<tBlackboardBuffer*>(write_port->GetUnusedBuffer(read_port->GetDataType()));
  }

  inline tWritePort* GetWritePort()
  {
    return write_port;
  }

  inline bool HasLock()
  {
    return lock_type != tRawBlackboardClient::eNONE;
  }

  inline bool HasReadLock()
  {
    return lock_type == tRawBlackboardClient::eREAD;
  }

  inline bool HasWriteLock()
  {
    return lock_type == tRawBlackboardClient::eWRITE;
  }

  /*!
   * (relevant mainly for auto-connect)
   *
   * \return Is client connected to blackboard server?
   */
  inline bool IsConnected() const
  {
    bool w = (write_port == NULL) ? true : write_port->IsConnected();
    return w && read_port->IsConnected();
  }

  /*!
   * Directly commit/publish buffer - without lock
   *
   * \param buffer Buffer to publish (unlocked buffer retrieved via getUnusedBuffer OR a used buffer with an additional lock)
   */
  void Publish(tBlackboardBuffer* buffer);

  /*!
   * Often Non-blocking, safe blackboard read operation
   * (will always operate on read copy - for SingleBufferedBlackboardServers readLock can be more efficient, but also more blocking)
   *
   * \param timeout (relevant for SingleBufferedBlackboardClients only) Timeout for lock attempt
   * \return Raw memory buffer containing blackboard contents - locked - don't forget to release read lock
   */
  inline const tBlackboardBuffer* Read(int64 timeout)
  {
    return static_cast<const tBlackboardBuffer*>(read_port->GetLockedUnsafeRaw());
  }

  inline const tBlackboardBuffer* Read()
  {
    return Read(2000);
  }

  /*!
   * same as read(long) with automatic locking of buffer.
   * (needs to be released by calling ThreadLocalCache.getFast().releaseAllLocks())
   */
  inline const tBlackboardBuffer* ReadAutoLocked(int64 timeout)
  {
    const tBlackboardBuffer* bb = Read(timeout);
    core::tThreadLocalCache::GetFast()->AddAutoLock(bb);
    return bb;
  }

  inline const tBlackboardBuffer* ReadAutoLocked()
  {
    return ReadAutoLocked(2000);
  }

  /*!
   * Read Lock on blackboard.
   *
   * Blackboard locked using this method needs to be unlocked via unlock() method!
   *
   * In most cases it will return a read copy (this can be forced).
   * On local single buffered blackboard servers - the same buffer might be used for reading (blocks more, but less copying)
   *
   * \param force_read_copy_to_avoid_blocking Force read copy to avoid blocking? (only relevant for single buffered blackboard servers)
   * \param timeout Timeout for call
   */
  const tBlackboardBuffer* ReadLock(bool force_read_copy_to_avoid_blocking = false, int64 timeout = 60000);

  /*!
   * Read part of blackboard
   *
   * \param offset offset in byte
   * \param length length in byte
   * \param timeout timeout for this synchronous operation
   * \return Lock Locked buffer - or null if operation failed (position 0 in this buffer is position 'offset' in original one)
   *  is unlocked automatically
   */
  tBlackboardBuffer* ReadPart(int offset, int length, int timeout = 60000);

  /*!
   * Send keep-alive signal to server (usually done automatically...)
   */
  void SendKeepAlive();

  /*!
   * Commit changes of previously locked buffer
   */
  void Unlock();

  /*!
   * Lock blackboard in order to read and commit changes
   * (synchronous... therefore deprecated if not absolutely necessary)
   *
   * \param timeout timeout for lock
   * \return Lock Locked buffer - or null if lock failed - this buffer may be modified -
   * call unlock() after modifications are complete - locks of buffer should normally not be modified -
   * except of it should be used in some other port or stored for longer than the unlock() operation
   */
  tBlackboardBuffer* WriteLock(int64 timeout = 60000);

};

} // namespace finroc
} // namespace blackboard

#endif // PLUGINS__BLACKBOARD__TRAWBLACKBOARDCLIENT_H
