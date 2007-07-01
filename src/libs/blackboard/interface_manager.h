 
/***************************************************************************
 *  interface_manager.h - BlackBoard interface manager
 *
 *  Generated: Mon Oct 09 19:05:46 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __BLACKBOARD_INTERFACE_MANAGER_H_
#define __BLACKBOARD_INTERFACE_MANAGER_H_

#include <interface/mediators/interface_mediator.h>
#include <interface/interface.h>
#include <core/exceptions/software.h>
#include <typeinfo>
#include <map>
#include <list>

class BlackBoardMemoryManager;
class BlackBoardMessageManager;
class Mutex;
class Module;

class BlackBoardInterfaceManager : public InterfaceMediator
{
 friend class BlackBoardMessageManager;
 public:

  BlackBoardInterfaceManager(bool bb_master = false);
  virtual ~BlackBoardInterfaceManager();

  Interface *  open_for_reading(const char *interface_type, const char *identifier);
  Interface *  open_for_writing(const char *interface_type, const char *identifier);
  void         close(Interface *interface);

  std::list<Interface *> *  open_all_of_type_for_reading(const char *interface_type);
  //  template <class InterfaceType>
  //    std::list<InterfaceType *>  openAllOfTypeForReading(const char *interface_type);

  virtual bool exists_writer(const Interface *interface) const;
  virtual void notify_of_data_change(const Interface *interface);

  template <class InterfaceType>
    InterfaceType * open_for_reading(const char *identifier);

  template <class InterfaceType>
    InterfaceType * open_for_writing(const char *identifier);

  const BlackBoardMemoryManager *  memory_manager() const;

 private:
  Interface *  new_interface_instance(const char *type, const char *identifier);
  void         delete_interface_instance(Interface *interface);

  void *       find_interface_in_memory(const char *type, const char *identifier);
  unsigned int next_mem_serial();
  unsigned int next_instance_serial();
  void         create_interface(const char *type, const char *identifier,
				Interface* &interface, void* &ptr);

  char *       strip_class_type(const char *type);

  Interface *  writer_for_mem_serial(unsigned int mem_serial);

 private:
  bool                          bb_master;

  unsigned int                  instance_serial;

  BlackBoardMemoryManager      *memmgr;
  BlackBoardMessageManager     *msgmgr;
  Mutex                        *mutex;
  Module                       *iface_module;

  std::map< unsigned int, Interface * >  writer_interfaces;
  std::map< unsigned int, RefCountRWLock * >  rwlocks;
};


/** Get interface of given type.
 * This will open a new interface for reading just like the non-template version of
 * openForReading(). But with the template method you will get a correctly typed object
 * that you can use. An TypeMismatchException is thrown if the string representation
 * of the type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception TypeMismatchException thrown if type in interface_type and the actual class
 * type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoardInterfaceManager::open_for_reading(const char *identifier)
{
  char *type_name = strip_class_type(typeid(InterfaceType).name());
  InterfaceType *interface = dynamic_cast<InterfaceType *>(open_for_reading(type_name, identifier));
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (R) types do not match");
  } else {
    return interface;
  }
}



/** Get writer interface of given type.
 * This will open a new interface for writing just like the non-template version of
 * openForWriting(). But with the template method you will get a correctly typed object
 * that you can use. An TypeMismatchException is thrown if the string representation
 * of the type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 * @exception TypeMismatchException thrown if type in interface_type and the actual class
 * type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoardInterfaceManager::open_for_writing(const char *identifier)
{
  char *type_name = strip_class_type(typeid(InterfaceType).name());
  InterfaceType *interface;
  try {
    interface = dynamic_cast<InterfaceType *>(open_for_writing(type_name, identifier));
  } catch (Exception &e) {
    // just caught to properly free memory
    delete[] type_name;
    throw;
  }
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (W) types do not match");
  } else {
    return interface;
  }
}

#endif
