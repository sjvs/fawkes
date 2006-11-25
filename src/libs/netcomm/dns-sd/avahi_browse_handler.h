
/***************************************************************************
 *  avahi_browse_handler.h - Avahi browse handler
 *
 *  Created: Wed Nov 08 13:16:47 2006
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

#ifndef __NETCOMM_DNSSD_AVAHI_BROWSE_HANDLER_H_
#define __NETCOMM_DNSSD_AVAHI_BROWSE_HANDLER_H_

#include <avahi-common/address.h>
#include <avahi-client/lookup.h>

#include <string>
#include <list>

/** @class AvahiBrowseHandler netcomm/dns-sd/avahi_browse_handler.h
 * Interface for class that process browse results.
 * Implement this class if you want to browse for services on the network.
 * Then register your handler and it will be informed of services that
 * join or leave the network. This is also required for an active search.
 *
 * It is recommended that you read about mDNS, DNS-SD and Avahi.
 *
 * @author Tim Niemueller
 */
class AvahiBrowseHandler
{
 friend class AvahiBrowser;
 public:
  /** Virtual destructor */
  virtual ~AvahiBrowseHandler() {};

  /** All results have been retrieved.
   * If you read the DNS-SD specs you will see that there is no explicit
   * "not existent" or "end of records" message - it cannot be. But after
   * some time it is assumed that there are no more records. If that is
   * the case this method is called.
   */
  virtual void all_for_now()                                  = 0;

  /** Cache exhausted. */
  virtual void cache_exhausted()                              = 0;

  /** Failed to browse for a given service.
   * @param name name of the service
   * @param type type of the service
   * @param domain domain of the service
   */
  virtual void failed(const char *name,
		      const char *type,
		      const char *domain)                     = 0;

  /** A service has been announced on the network.
   * @param name name of the service
   * @param type type of the service
   * @param domain domain of the service
   * @param host_name name of the host that provides the service
   * @param address Address of the host, read Avahi documentation
   * @param port port of the service
   * @param txt list of txt records.
   * @param flags extra flags, see Avahi documentation
   */
  virtual void service_added(const char *name,
			     const char *type,
			     const char *domain,
			     const char *host_name,
			     const AvahiAddress *address,
			     uint16_t port,
			     std::list<std::string> &txt,
			     AvahiLookupResultFlags flags
			     )                                = 0;

  /** A service has been removed from the network.
   * @param name name of the service
   * @param type type of the service
   * @param domain domain of the service
   */
  virtual void service_removed(const char *name,
			       const char *type,
			       const char *domain)            = 0;

};

#endif
