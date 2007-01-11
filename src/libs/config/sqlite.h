
/***************************************************************************
 *  sqlite.h - Fawkes configuration stored in a SQLite database
 *
 *  Created: Wed Dec 06 17:20:41 2006
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

#ifndef __CONFIG_SQLITE_H_
#define __CONFIG_SQLITE_H_

#include <config/config.h>

#include <map>
#include <list>
#include <string>

class Mutex;

class SQLiteConfiguration : public Configuration
{
 public:
  SQLiteConfiguration(const char *conf_path = NULL);
  virtual ~SQLiteConfiguration();

  virtual void          copy(Configuration *copyconf);

  virtual void          add_change_handler(ConfigurationChangeHandler *h);
  virtual void          rem_change_handler(ConfigurationChangeHandler *h);

  virtual void          load(const char *filename, const char *defaults_filename,
			     const char *tag = NULL);

  virtual void          tag(const char *tag);
  virtual std::list<std::string> tags();

  virtual bool          exists(const char *comp, const char *path);
  virtual bool          is_float(const char *comp, const char *path);
  virtual bool          is_uint(const char *comp, const char *path);
  virtual bool          is_int(const char *comp, const char *path);
  virtual bool          is_bool(const char *comp, const char *path);
  virtual bool          is_string(const char *comp, const char *path);

  virtual std::string     get_type(const char *comp, const char *path);
  virtual float           get_float(const char *comp, const char *path);
  virtual unsigned int    get_uint(const char *comp, const char *path);
  virtual int             get_int(const char *comp, const char *path);
  virtual bool            get_bool(const char *comp, const char *path);
  virtual std::string     get_string(const char *comp, const char *path);
  virtual ValueIterator * get_value(const char *comp, const char *path);

  virtual void          set_float(const char *comp, const char *path,
				  float f);
  virtual void          set_uint(const char *comp, const char *path,
				 unsigned int uint);
  virtual void          set_int(const char *comp, const char *path,
				int i);
  virtual void          set_bool(const char *comp, const char *path,
				 bool b);
  virtual void          set_string(const char *comp, const char *path,
				   std::string s);
  virtual void          set_string(const char *comp, const char *path,
				   const char *s, unsigned int s_length);

  virtual void          erase(const char *comp, const char *path);

 private:
  typedef struct sqlite3_stmt sqlite3_stmt;

 public:
 class SQLiteValueIterator : public Configuration::ValueIterator
  {
    friend class SQLiteConfiguration;
   protected:
    SQLiteValueIterator(sqlite3_stmt *stmt);
   public:
    virtual ~SQLiteValueIterator();
    virtual bool          next();
    virtual bool          valid();
    
    virtual const char *  component();
    virtual const char *  path();
    virtual const char *  type();
    
    virtual bool          is_float();
    virtual bool          is_uint();
    virtual bool          is_int();
    virtual bool          is_bool();
    virtual bool          is_string();

    virtual float         get_float();
    virtual unsigned int  get_uint();
    virtual int           get_int();
    virtual bool          get_bool();
    virtual std::string   get_string();

   private:
    sqlite3_stmt *stmt;
  };

  ValueIterator * iterator();
  ValueIterator * search(const char *component, const char *path);

  void lock();
  bool tryLock();
  void unlock();

 private:
  void            init();
  std::string     get_type(const char *table, const char *comp, const char *path);
  bool            exists(const char *sql, const char *comp, const char *path);
  sqlite3_stmt *  get_value(const char *type, const char *comp, const char *path);
  sqlite3_stmt *  prepare_update_value(const char *sql,
				       const char *comp, const char *path);
  sqlite3_stmt *  prepare_insert_value(const char *sql, const char *type,
				       const char *comp, const char *path);
  void            execute_insert_or_update(sqlite3_stmt *stmt);


 private:
  typedef struct sqlite3 sqlite3;
  sqlite3 *db;
  /* to do:
   * - config-iterator
   */
  const char *conf_path;
  bool opened;
  Mutex *mutex;

  std::map<std::string, std::list<ConfigurationChangeHandler *> >  change_handlers;
  std::list<ConfigurationChangeHandler *>::iterator                cit;

};

#endif
