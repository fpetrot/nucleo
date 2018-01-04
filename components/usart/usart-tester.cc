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
      switch (c.stopBit){
        // 00 :1    bit stop
        // 01 :0.5  bit stop
        // 10 :2    bit stop
        // 11 :1,5  bit stop
        case 0:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/1 Stop  (0x%02x)\n", c.data);
        break;
        case 1:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/0.5 Stop  (0x%02x)\n", c.data);
        break;
        case 2:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/2 Stop  (0x%02x)\n", c.data);
        break;
        case 3:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/1.5 Stop  (0x%02x)\n", c.data);
        break;
        default:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/ERROR Stop  (0x%02x)\n", c.data);
      }
    }
  }
}

void usartTester::read_thread9bit()
{
  std::vector<data9bit> data9;
  while(1) {
    p_uart.recv(data9);  //rx->recv(data)
    for (auto c : data9) {
      switch (c.stopBit){
        // 00 :1    bit stop
        // 01 :0.5  bit stop
        // 10 :2    bit stop
        // 11 :1,5  bit stop
        case 0:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/1 Stop  (0x%02x)\n", c.data);
        break;
        case 1:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/0.5 Stop  (0x%02x)\n", c.data);
        break;
        case 2:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/2 Stop  (0x%02x)\n", c.data);
        break;
        case 3:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/1.5 Stop  (0x%02x)\n", c.data);
        break;
        default:
          MLOG_F(SIM, DBG, "rcv_thread: got a 8 bits/ERROR Stop  (0x%02x)\n", c.data);
      }
    }
  }
}
            //
            // if( c == 0x15){ //si TE: transmision enable à true, alors send contenue TDR
            //   MLOG_F(SIM, DBG, "follow %s prepare to send 0x%lx\n", __FUNCTION__, (unsigned long) 0x16);
            //   std::vector<uint8_t> data8;
            //   data8.push_back(uint8_t(0x16)); //on envoie le contenue du transmission data register
            //   p_uart.send(data8);
            //
            //   wait(12, SC_NS);
            //
            //   std::vector<uint8_t> dara;
            //   dara.push_back(uint8_t(0x16)); //on envoie le contenue du transmission data register
            //   p_uart.send(dara);
