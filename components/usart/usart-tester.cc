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

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>

#include <rabbits/logger.h>

#include "usart-tester.h"

using namespace sc_core;


usartTester::usartTester(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
    : Component(name, c)
    ,p_uart("usart")
{
    SC_THREAD(read_thread);

    SC_THREAD(send_thread);
}

usartTester::~usartTester(){}


void usartTester::read_thread(){
  usart_data data_recv;

  while(1) {
    p_uart.recv(data_recv);  //rx->recv(data)
    MLOG_F(SIM, DBG, "rcv_thread: got a %d bits/0x%01x Stop  (0x%02x)\n",data_recv.length?9:8 ,data_recv.stopBit, data_recv.data);
  }
}


//SENDER THREAD FOR TEST PURPOSE
void usartTester::send_thread(){
  wait(1,SC_MS);
  send_frame(1,0x9,2);
  wait(12,SC_NS);
  send_frame(0,0x10,0);
  wait(12,SC_NS);
  send_frame(1,0x11,1);
  wait(12,SC_NS);
}

void usartTester::send_frame( bool length, uint16_t data, char stopBit){
  usart_data frame;
    frame.length = length;
    frame.data = data;
    frame.stopBit =stopBit;
    MLOG_F(SIM, DBG, "%s: send %d bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.length?9:8, frame.stopBit, frame.data);
    p_uart.send(frame);
}
