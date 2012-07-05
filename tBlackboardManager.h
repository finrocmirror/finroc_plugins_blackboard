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

#ifndef plugins__blackboard__tBlackboardManager_h__
#define plugins__blackboard__tBlackboardManager_h__

#include "rrlib/finroc_core_utils/definitions.h"
#include "rrlib/rtti/tDataTypeBase.h"
#include "rrlib/finroc_core_utils/container/tSimpleListWithMutex.h"

#include "core/tRuntimeEnvironment.h"
#include "core/tLockOrderLevels.h"
#include "core/tFrameworkElement.h"
#include "core/tRuntimeListener.h"
#include "core/thread/tCoreLoopThreadBase.h"

namespace finroc
{
namespace blackboard
{
class tRawBlackboardClient;
class tAbstractBlackboardServerRaw;

/*!
 * \author Max Reichardt
 *
 * Blackboard Manager (has similar tasks as blackboard handler in MCA2)
 *
 * is also framework element that groups blackboard servers
 */
class tBlackboardManager : public core::tFrameworkElement, public core::tRuntimeListener
{
  /*!
   * Class that stores infos for one category of blackboards
   */
  class tBlackboardCategory : public core::tFrameworkElement
  {
  private:

    // Outer class BlackboardManager
    tBlackboardManager* const outer_class_ptr;

  public:

    /*! Default flags for AbstractBlackboardServers in this category */
    const int default_flags;

    /*! List of blackboards in category */
    util::tSafeConcurrentlyIterableList<tAbstractBlackboardServerRaw*, rrlib::thread::tNoMutex> blackboards;

    /*!
     * \param category_name Name of category
     * \param cDefault flags for AbstractBlackboardServers in this category
     */
    tBlackboardCategory(tBlackboardManager* const outer_class_ptr_, const util::tString& category_name, int default_flags_);

    /*!
     * Add Blackboard to this category
     *
     * \param blackboard Blackboard to add
     */
    void Add(tAbstractBlackboardServerRaw* blackboard);

    /*!
     * Check whether blackboard client wants to connect to any contained blackboards
     *
     * \param client Blackboard client
     */
    void CheckConnect(tRawBlackboardClient* client);

    /*!
     * Remove blackboard from this category
     *
     * \param blackboard Blackboard to remove
     */
    void Remove(tAbstractBlackboardServerRaw* blackboard);

  };

private:
  class tLockCheckerThread; // inner class forward declaration
  friend class tAbstractBlackboardServerRaw;
private:

  /*! Singleton instance */
  static tBlackboardManager* volatile instance;

  /*! Blackboard categories */
  ::finroc::util::tArrayWrapper<tBlackboardCategory*> categories;

  /*! Temporary StringBuilder */
  std::string temp_buffer;

  /*! all blackboard clients */
  util::tSafeConcurrentlyIterableList<tRawBlackboardClient*, rrlib::thread::tNoMutex> bb_clients;

  /*! Clients that wish to autoconnect */
  util::tSimpleListWithMutex<tRawBlackboardClient*> auto_connect_clients;

public:

  /*!
   * Categories of blackboards - same as in MCA2 - questionable if that makes sense
   * Is, however, the easiest & most efficient way to remain compatible
   */
  static const int cALL = -1, cSHARED = 0, cLOCAL = 1, cREMOTE = 2, cDIMENSION = 3;

  /*! Name of Blackboard Manager Framework Element */
  static util::tString cNAME;

  /*! Cached Value: Name with slashes */
  static util::tString cSLASHED_NAME;

  /*! Name of read and write ports */
  static util::tString cREAD_PORT_NAME, cWRITE_PORT_NAME;

  /*! The same with prepended slahes */
  static util::tString cREAD_POSTFIX, cWRITE_POSTFIX;

private:

  tBlackboardManager();

  /*!
   * Check whether any of the autoConnectClients wishes to connect
   *
   * \param server new server to check with
   */
  void CheckAutoConnect(tAbstractBlackboardServerRaw* server);

  /*!
   * Synchronized helper method
   */
  static void CreateBlackboardManager();

protected:

  virtual void PrepareDelete();

public:

  /*!
   * Add client to blackboard manager
   *
   * \param client Blackboard client
   * \param auto_connect Auto-connect client
   */
  void AddClient(tRawBlackboardClient* client, bool auto_connect);

  /*!
   * Get blackboard matching the specified features
   *
   * \param name Blackboard name
   * \param category Blackboard Category (-1 all categories)
   * \param type Data type of blackboard (null = all types)
   * \return Blackboard - or null if no blackboard could be found
   */
  tAbstractBlackboardServerRaw* GetBlackboard(const util::tString& name, int category, rrlib::rtti::tDataTypeBase type);

  /*!
   * Get blackboard matching the specified features
   *
   * \param name Blackboard name
   * \param start_cat category index to start looking (inclusive)
   * \param end_cat end category index (inclusive)
   * \param type Data type of blackboard (null = all types)
   * \return Blackboard - or null if no blackboard could be found
   */
  tAbstractBlackboardServerRaw* GetBlackboard(const util::tString& name, int start_cat, int end_cat, rrlib::rtti::tDataTypeBase type);

  /*!
   * Retrieve blackboard with specified index
   *
   * \param index Index
   * \param category Index in which category? (-1 all)
   * \return Blackboard - or null, if it does not exist (can happen, because lists are not filled continuously when blackboards are deleted)
   */
  tAbstractBlackboardServerRaw* GetBlackboard(size_t index, int category = -1);

  /*!
   * Retrieve blackboard with specified index
   *
   * \param index Index
   * \param start_cat category index to start looking (inclusive)
   * \param end_cat end category index (inclusive)
   * \return Blackboard - or null, if it does not exist (can happen, because lists are not filled continuously when blackboards are deleted)
   */
  tAbstractBlackboardServerRaw* GetBlackboard(size_t index, int start_cat, int end_cat);

  /*!
   * \param qname qualified link
   * \return Blackboard name - empty string if no blackboard
   */
  util::tString GetBlackboardNameFromQualifiedLink(const util::tString& qname);

  /*!
   * \param category_index Index (see constants at beginning of class)
   * \return Category object
   */
  inline tBlackboardCategory* GetCategory(int category_index)
  {
    return categories[category_index];
  }

  /*!
   * \return Singleton instance - may be null after blackboard manager has been deleted
   */
  static tBlackboardManager* GetInstance();

  /*!
   * Retrieve number of blackboards (may include empty entries, if blackboards have been deleted)
   *
   * \param category Index in which category? (-1 all)
   */
  size_t GetNumberOfBlackboards(int category = -1);

  /*!
   * Retrieve number of blackboards (may include empty entries, if blackboards have been deleted)
   *
   * \param start_cat category index to start looking (inclusive)
   * \param end_cat end category index (inclusive)
   */
  size_t GetNumberOfBlackboards(int start_cat, int end_cat);

  /*!
   * \param client Remove Blackboard client
   */
  void RemoveClient(tRawBlackboardClient* client);

  virtual void RuntimeChange(int8 change_type, core::tFrameworkElement& element);

  virtual void RuntimeEdgeChange(int8 change_type, core::tAbstractPort& source, core::tAbstractPort& target)
  {
    // do nothing
  }

private:
  /*!
   * \author Max Reichardt
   *
   * Thread checks for outdated locks in BlackboardServers
   * Thread sends alive signals for blackboard clients that hold lock.
   */
  class tLockCheckerThread : public core::tCoreLoopThreadBase
  {
  private:

    // Outer class BlackboardManager
    tBlackboardManager* const outer_class_ptr;

  public:

    tLockCheckerThread(tBlackboardManager* const outer_class_ptr_);

    virtual void MainLoopCallback();

  };

};

} // namespace finroc
} // namespace blackboard

#endif // plugins__blackboard__tBlackboardManager_h__
