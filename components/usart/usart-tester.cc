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
    SC_THREAD(read_thread8bit);
    SC_THREAD(read_thread9bit);

    SC_THREAD(send_thread8bit);
    SC_THREAD(send_thread9bit);
}

usartTester::~usartTester()
{
}


void usartTester::read_thread8bit()
{
  std::vector<data8bit> data8;

  while(1) {
    p_uart.recv(data8);  //rx->recv(data)
    for (auto c : data8) {
      MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/0x%01x Stop  (0x%02x)\n",c.stopBit, c.data);
    }
  }
}

void usartTester::read_thread9bit()
{
  std::vector<data9bit> data9;
  while(1) {
    p_uart.recv(data9);  //rx->recv(data)
    for (auto c : data9) {
      MLOG_F(SIM, DBG, "rcv_thread: got a 9 bits/0x%01x Stop  (0x%02x)\n",c.stopBit, c.data);
    }
  }
}


//SENDER THREAD FOR TEST PURPOSE
void usartTester::send_thread8bit(){
  std::vector<data8bit> data_v;
  data8bit frame;

  wait(1,SC_MS);

  wait(32,SC_NS);
  frame.data = 10;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 8 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);

  wait(32,SC_NS);
  frame.data = 20;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 8 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);


  wait(32,SC_NS);
  frame.data = 30;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 8 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);

}

//SENDER THREAD FOR TEST PURPOSE
void usartTester::send_thread9bit(){
  std::vector<data9bit> data_v;
  data9bit frame;

  wait(5,SC_MS);

  wait(32,SC_NS);
  frame.data = 1;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 9 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);

  wait(32,SC_NS);
  frame.data = 2;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 9 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);


  wait(32,SC_NS);
  frame.data = 3;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 9 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);

  wait(200,SC_MS);

  frame.data = 4;
  frame.stopBit = 0b10;
  data_v.push_back(frame);
  MLOG_F(SIM, DBG, "%s: send 9 bits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,frame.stopBit, frame.data);
  p_uart.send(data_v);
}
