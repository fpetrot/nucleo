/*
 *  This file is a tester for Nucleo Usart component
 *  Copyright (C) 2017 Joris Collomb
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

#ifndef _usartTester_H
#define _usartTester_H

#include <rabbits/component/slave.h>
#include <rabbits/component/port/out.h>
#include "usartPort.h"


class usartTester : public Component
{
public:
SC_HAS_PROCESS (usartTester);
usartTester(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
virtual ~usartTester();

private:

void read_thread();

void send_thread();

void send_frame(bool , uint16_t ,char );

public:
UsartPort p_uart;


};

#endif
