/*
 *  This file is part of Rabbits
 *  Copyright (C) 2017 Clément Chigot
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _SWITCH_H
#define _SWITCH_H

#include <rabbits/component/component.h>
#include <rabbits/component/port/out.h>

class Switch : public Component {
 public:
	SC_HAS_PROCESS(Switch); 
	Switch(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
	virtual ~Switch(); 

	OutPort<bool> p_out; 

 private:
	void thread();
	bool _kbhit(); 
	char m_key;

};

#endif
