/*
 *  This file is part of Rabbits
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

template <typename data_type>
void read_thread();

void read_thread8bit();
void read_thread9bit();

void send_thread8bit();
void send_thread9bit();


public:
UsartPort p_uart;


};

#endif
