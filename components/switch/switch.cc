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
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>

#include "switch.h"

#include "rabbits/logger.h"

using namespace sc_core;

Switch::Switch(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
	: Component(name, params, c)
	 , p_out("pout") 
{
	m_key = params["key"].as<uint8_t>();
	SC_THREAD(thread);
}

Switch::~Switch(){}

/* Function to prevent infinite wait 
 * NOTE : may not work under windows or others distributions
 */ 
bool Switch::_kbhit() {
  static const int STDIN = 0;
  static bool initialized = false;

  if (! initialized) {
	// Use termios to turn off line buffering
	termios term;
	tcgetattr(STDIN, &term);
	term.c_lflag &= ~ICANON;
	tcsetattr(STDIN, TCSANOW, &term);
	setbuf(stdin, NULL);
	initialized = true;
  }

  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return bytesWaiting;
}

void Switch::thread(){
	while(1){
		while (!_kbhit()) { // wait until a key has been pressed
		    wait(1, SC_MS); 
		}
		if(getchar() == m_key){
			p_out.sc_p = !p_out.sc_p;
		}
	}
}
