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

#ifndef _LED_H
#define _LED_H

#include <rabbits/component/component.h>
#include <rabbits/component/port/in.h>

class Led : public Component {
 public:
	SC_HAS_PROCESS(Led); 
	Led(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
	virtual ~Led(); 

	InPort<bool> p_in; 

 private:
	void blink();

};

#endif
