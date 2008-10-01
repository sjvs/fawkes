
/***************************************************************************
 *  config_add_dialog.cpp - Add config entries
 *
 *  Created: Thu Sep 25 17:31:40 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <tools/config_editor/config_add_dialog.h>
#include <gui_utils/utils.h>

using namespace fawkes;

/** @class ConfigAddDialog tools/config_editor/config_add_dialog.h
 * Dialog to add a config entry
 *
 * @author Daniel Beck
 */

/** @var ConfigAddDialog::m_ent_path
 * The Gtk::Entry that contains the path of the new entry.
 */

/** @var ConfigAddDialog::m_ent_value
 * The Gtk::Entry that contains the value of the new entry.
 */

/** @var ConfigAddDialog::m_cmb_type
 * The Gtk::ComboBox to select the type of the new entry.
 */

/** @var ConfigAddDialog::m_chb_is_default
 * The Gtk::CheckButton to set the default flag
 */

/** Constructor.
 * @param cobject pointer to base object type
 * @param ref_xml Glade XML file
 */
ConfigAddDialog::ConfigAddDialog( BaseObjectType* cobject,
				  const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml )
  : Gtk::Dialog(cobject)
{
  m_ent_path = dynamic_cast<Gtk::Entry*>( get_widget(ref_xml, "entPathAdd") );
  m_ent_value = dynamic_cast<Gtk::Entry*>( get_widget(ref_xml, "entValueAdd") );
  m_cmb_type = dynamic_cast<Gtk::ComboBox*>( get_widget(ref_xml, "cmbTypeAdd") );
  m_chb_is_default = dynamic_cast<Gtk::CheckButton*>( get_widget(ref_xml, "chbIsDefaultAdd") );
}

/** Destructor. */
ConfigAddDialog::~ConfigAddDialog()
{
}

/** Initialize the dialog.
 * @param path the config path of the selected row
 */
void
ConfigAddDialog::init(const Glib::ustring& path)
{
  m_ent_path->set_text(path);
  m_cmb_type->set_active(-1);
}

/** Get the path of the new entry.
 * @return the path of the new entry
 */
Glib::ustring
ConfigAddDialog::get_path() const
{
  return m_ent_path->get_text();
}

/** Get the type of the new entry.
 * @return the type of the new entry
 */
Glib::ustring
ConfigAddDialog::get_type() const
{
  Gtk::TreeIter iter = m_cmb_type->get_active();
  Gtk::TreeRow row = *iter;
  Glib::ustring type;  
  
  row.get_value(0, type);

  return type;
}

/** Get the value of the new entry.
 * @return the value of the new entry
 */
Glib::ustring
ConfigAddDialog::get_value() const
{
  return m_ent_value->get_text();
}

/** Get the default flag of the new entry
 * @return if true add to default config database
 */
bool
ConfigAddDialog::get_is_default() const
{
  return m_chb_is_default->get_active();
}