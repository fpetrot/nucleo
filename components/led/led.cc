/*
 *  This file is part of Rabbits
 *  Copyright (C) 2017 Frédéric Pétrot and ...
 *
 *  Based on the Raspberry PI Braodcom GPIO implementation
 *  Copyright (C) 2015  Clement Deschamps and Luc Michel
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

#include <cstdio>
#include <cstdlib>

#include "led.h"

#include "rabbits/logger.h"

using namespace sc_core;

Led::Led(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
	: Component(name, params, c)
	, p_in("pin") 
{
	SC_METHOD(blink);
	sensitive << p_in.sc_p; 
}

Led::~Led(){}

void Led::blink(){
	if(p_in.sc_p.read()){
		std::cout << "Led light on" << std::endl;
	} else {
		std::cout << "Led light off" << std::endl; 
	}
}
