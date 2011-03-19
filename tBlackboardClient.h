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

#ifndef plugins__blackboard__tBlackboardClient_h__
#define plugins__blackboard__tBlackboardClient_h__

#include "rrlib/finroc_core_utils/definitions.h"

#include "rrlib/serialization/tDataTypeBase.h"
#include "plugins/blackboard/tRawBlackboardClient.h"

#include "core/port/tPortUtil.h"

namespace finroc
{
namespace core
{
class tFrameworkElement;
} // namespace finroc
} // namespace core

namespace finroc
{
namespace blackboard
{
template<typename T>
class tBlackboardReadAccess;
template<typename T>
class tBlackboardWriteAccess;

/*!
 * \author Max Reichardt
 *
 * This is the base class for a blackboard client
 */
template<typename T>
class tBlackboardClient : public util::tObject
{
private:

public:
  typedef typename tAbstractBlackboardServer<T>::tBBVector tBBVector;
  typedef typename tAbstractBlackboardServer<T>::tBBVectorVar tBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tConstBBVectorVar tConstBBVectorVar;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransaction tChangeTransaction;
  typedef typename tAbstractBlackboardServer<T>::tChangeTransactionVar tChangeTransactionVar;
  typedef typename tAbstractBlackboardServer<T>::tConstChangeTransactionVar tConstChangeTransactionVar;

private:

  /*! Wrapped raw blackboard client */
  tRawBlackboardClient* wrapped;

protected:

  /*! not null - if buffer is currently locked for writing */
  tBBVectorVar locked;

  /*! not null - if buffer is currently locked for writing */
  tConstBBVectorVar read_locked;

public:

  typedef tBlackboardWriteAccess<T> tWriteAccess;
  typedef tBlackboardReadAccess<T> tReadAccess;

  /*! Log domain for this class */
  RRLIB_LOG_CREATE_NAMED_DOMAIN(log_domain, "blackboard");

private:

  /*!
   * Reset variables after unlock
   */
  inline void ResetVariables()
  {
    wrapped->cur_lock_iD = -1;
    wrapped->lock_type = tRawBlackboardClient::eNONE;

    locked.reset();
    read_locked.reset();
  }

protected:

  /*!
   * \return log description
   */
  inline const core::tFrameworkElement& GetLogDescription() const
  {
    return *wrapped;
  }

public:

  /*!
   * \param description Name/Uid of blackboard
   * \param parent Parent of blackboard client
   * \param auto_connect Auto-Connect blackboard client to matching server?
   * \param auto_connect_category If auto-connect is active: Limit auto-connecting to a specific blackboard category? (-1 is no)
   * \param read_port Create read port?
   * \param write_port Create write port?
   * \param type Data Type of blackboard content
   */
  tBlackboardClient(const util::tString& description, core::tFrameworkElement* parent = NULL, bool auto_connect = true, int auto_connect_category = -1, bool read_port = true, bool write_port = true, rrlib::serialization::tDataTypeBase type = rrlib::serialization::tDataType<T>());

  //
  //    /**
  //     * same as read(long) with automatic locking of buffer.
  //     * (needs to be released by calling ThreadLocalCache.getFast().releaseAllLocks())
  //     */
  //    @Inline
  //    @Const public BlackboardBuffer readAutoLocked(long timeout) {
  //        @Const BlackboardBuffer bb = read(timeout);
  //        ThreadLocalCache.getFast().addAutoLock(bb);
  //        return bb;
  //    }
  //
  //    @Inline
  //    @Const public BlackboardBuffer readAutoLocked() {
  //        return readAutoLocked(2000);
  //    }
  //
  //
  /*!
   * Commit asynchronous change to blackboard. Blackboard does
   * not need to be locked for this operation.
   * (if connection is broken, there's no guarantee that this will work or that an exception is thrown otherwise)
   *
   * \param change_buf Contents to write to this position (unlocked buffer retrieved via getUnusedBuffer OR a used buffer with an additional lock)
   * \param index First element to change
   * \param offset Some custom offset in element (optional)
   */
  inline void CommitAsynchChange(tChangeTransactionVar& change_buf, int index, int offset)
  {
    assert(!core::tPortDataManager::GetManager(change_buf)->IsUnused() && "Obtain buffer from getUnusedChangeBuffer()");
    tAbstractBlackboardServer<T>::cASYNCH_CHANGE.Call(*wrapped->GetWritePort(), change_buf, index, offset, true);
  }

  /*!
   * \return unused buffer - may be published/committed directly
   */
  inline tBBVectorVar GetUnusedBuffer()
  {
    return wrapped->GetWritePort()->GetBufferForCall<tBBVector>();
  }

  /*!
   * \return unused change buffer - to be used in commitAsynchChange
   */
  inline tChangeTransactionVar GetUnusedChangeBuffer()
  {
    return wrapped->GetWritePort()->GetBufferForCall<tChangeTransaction>();
  }

  /*!
   * \return Wrapped raw blackboard client
   */
  inline tRawBlackboardClient* GetWrapped()
  {
    return wrapped;
  }

  /*!
   * \return Is client currently holding read or write lock?
   */
  inline bool HasLock()
  {
    return wrapped->HasLock();
  }

  /*!
   * \return Is client currently holding read lock?
   */
  inline bool HasReadLock()
  {
    return wrapped->HasReadLock();
  }

  /*!
   * \return Is client currently holding write lock?
   */
  inline bool HasWriteLock()
  {
    return wrapped->HasWriteLock();
  }

  /*!
   * Initialize blackboard client
   */
  inline void Init()
  {
    wrapped->Init();
  }

  /*!
   * Directly commit/publish buffer - without lock
   *
   * \param buffer Buffer to publish (unlocked buffer retrieved via getUnusedBuffer OR a used buffer with an additional lock)
   */
  void Publish(tBBVectorVar& buffer);

  /*!
   * Often Non-blocking, safe blackboard read operation
   * (will always operate on read copy - for SingleBufferedBlackboardServers readLock can be more efficient, but also more blocking)
   *
   * \param timeout (relevant for SingleBufferedBlackboardClients only) Timeout for lock attempt
   * \return Raw memory buffer containing blackboard contents - locked - don't forget to release read lock
   */
  inline tConstBBVectorVar Read(int64 timeout)
  {
    return core::tPortUtil<tBBVector>::GetValueWithLock(wrapped->GetReadPort());
  }

  inline tConstBBVectorVar Read()
  {
    return Read(2000);
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
  typename tAbstractBlackboardServer<T>::tConstBBVectorVar ReadLock(bool force_read_copy_to_avoid_blocking = false, int timeout = 60000);

  /*!
   * Commit changes of previously locked buffer
   */
  void Unlock();

  //
  //    /**
  //     * Read part of blackboard
  //     *
  //     * \param offset offset in byte
  //     * \param length length in byte
  //     * \param timeout timeout for this synchronous operation
  //     * \return Lock Locked buffer - or null if operation failed (position 0 in this buffer is position 'offset' in original one)
  //     *  is unlocked automatically
  //     */
  //    public BlackboardBuffer readPart(int offset, int length, @CppDefault("60000") int timeout) {
  //        if (timeout <= 0) {
  //            timeout = 60000; // wait one minute for method to complete if no time is specified
  //        }
  //        try {
  //            return AbstractBlackboardServer.READ_PART.call(getWritePort(), offset, length, timeout, timeout + NET_TIMEOUT);
  //        } catch (MethodCallException e) {
  //            return null;
  //        }
  //    }
  //
  /*!
   * Lock blackboard in order to read and commit changes
   * (synchronous/blocking... only use if absolutely necessary)
   *
   * \param timeout timeout for lock
   * \return Lock Locked buffer - or null if lock failed - this buffer may be modified -
   * call unlock() after modifications are complete - locks of buffer should normally not be modified -
   * except of it should be used in some other port or stored for longer than the unlock() operation
   */
  typename tAbstractBlackboardServer<T>::tBBVectorVar WriteLock(int timeout = 60000);

};

} // namespace finroc
} // namespace blackboard

#include "plugins/blackboard/tBlackboardClient.hpp"

#include "plugins/blackboard/tBlackboardReadAccess.h"
#include "plugins/blackboard/tBlackboardWriteAccess.h"

#endif // plugins__blackboard__tBlackboardClient_h__
