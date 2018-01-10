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
    ,p_uart_rx("usart-rx")
    ,p_uart_tx("usart-tx")
{
    SC_THREAD(read_thread);

    SC_THREAD(send_thread);

    state.sampling_time_tester=2312;
    state.M_tester=0;
    state.OVER8_tester=0;
    state.USART_DR_SR_tester=0x0;
    state.USART_TDR_SR_tester=0x0;
    state.USART_DR_tester=0x0;
    state.stop_bit_tester=0b10;
    state.PCE_tester=0;
    state.PS_tester=1;
}

usartTester::~usartTester(){}


////////////////////////////////////////////////////////////////////////////////
void usartTester::read_thread()
{
  unsigned int sample, bit_count;

  wait(10,SC_NS); //Little wait, to let the line time to init

  ////////////////////////sampling data
  while(1) {
    while(p_uart_rx.sc_p){        //the line is idle, waiting for start bit
      wait(state.sampling_time_tester,SC_NS);
    }
    bool value = p_uart_rx.sc_p;
    MLOG_F(SIM, DBG, "%s:USART-TESTER: start bit detected! %d\n",__FUNCTION__, value);
    //Start bit detection!
    for (bit_count=0; bit_count <= (state.M_tester?9:8); bit_count++){
      for (sample=1; sample <= (state.OVER8_tester?8:16); sample++){
        if (sample==(state.OVER8_tester?5:9)){  //sampling time
          //adding input bit in the shifting register
          if(bit_count==0){ //if still during start bit
            if(p_uart_rx.sc_p){
              bool value = p_uart_rx.sc_p;
              //TODO: gestion of false start bit detection
              MLOG_F(SIM, DBG, "%s: ERROR: start bit detection Error:%d sample:%d\n",__FUNCTION__, value, sample);
            }
          }else{
            state.USART_DR_SR_tester = ((state.USART_DR_SR_tester >> 1) | (p_uart_rx.sc_p << (state.M_tester?9:8)-1));
            MLOG_F(SIM, DBG, "%s: USART-TESTER-RX:%d: %d\n",__FUNCTION__,bit_count,p_uart_rx.sc_p.read());
          }
        }
        wait(state.sampling_time_tester,SC_NS);
      }
    }//data is received, and in shift register

    //Verification of stop bits
    //514/841 DocID025350 Rev 4
    switch(state.stop_bit_tester){
      case 0b00 ://1 stop bit
      wait((state.sampling_time_tester * (state.OVER8_tester?5:9)),SC_NS);
      if(!p_uart_rx.sc_p){
        //TODO: gestion of error stop bit sampling
        MLOG_F(SIM, DBG, "%s: USART-TESTER: Stop bit detection Error\n",__FUNCTION__);
      }
      break;
      /////////////////////
      case 0b01 ://0,5 stop bit
      //No sampling with 0,5 stop bit, so no error checking
      break;
      /////////////////////
      case 0b10 : //2 stop bit
      wait((state.sampling_time_tester * (state.OVER8_tester?5:9)),SC_NS);  //only the first stop bit is check
      if(!p_uart_rx.sc_p){
        //TODO: gestion of error stop bit sampling
        MLOG_F(SIM, DBG, "%s: USART-TESTER: Stop bit detection Error\n",__FUNCTION__);
      }
      break;
      /////////////////////
      case 0b11 ://1,5 stop bit
      wait((state.sampling_time_tester * (state.OVER8_tester?8:17)),SC_NS);
      if(!p_uart_rx.sc_p){
        //TODO: gestion of error stop bit sampling
        MLOG_F(SIM, DBG, "%s: USART-TESTER: Stop bit detection Error\n",__FUNCTION__);
      }
      break;
      /////////////////////
    }

    //update USART_RDR_SR, the data are in the shift register
    if(state.PCE_tester){  //parity control enable
      int count=0 , i;
      for (i=0; i<(state.M_tester?8:7) ; i++){
        count+=(state.USART_DR_SR_tester>>i)&1; //counting set bits in data
      }
      if (count%2 + (state.PS_tester)!=state.USART_DR_SR_tester>>(state.M_tester?9:8)){  //MSB is parity bit, PS define Odd or Even
        MLOG_F(SIM, DBG, "%s: ERROR: parity check, PE set\n",__FUNCTION__);
      }else{
        MLOG_F(SIM, DBG, "%s: parity check OK\n",__FUNCTION__);
      }
      state.USART_DR_tester = state.USART_DR_SR_tester & (state.M_tester?255:127); //masking of MSB parity bit, mask depend on length
    }else state.USART_DR_tester = state.USART_DR_SR_tester;
    MLOG_F(SIM, DBG, "%s: USART_DR update complete (0x%x)\n",__FUNCTION__,state.USART_DR_tester);
  }
}


////////////////////////////////////////////////////////////////////////////////
void usartTester::send_thread()
{
  // p_uart_tx.sc_p = true;  //1 on output tx port
  //
  // wait(1000,SC_MS);
  // MLOG_F(SIM, DBG, "%s:starting sending test frame\n",__FUNCTION__);
  //
  // unsigned int bit_count;
  // //////////////////SENDING IDLE FRAME
  // p_uart_tx.sc_p = true;  //1 on output tx port
  //
  // //calculation of idle frme time, depend on oversampling method, number of data bit and stop bit
  // uint32_t time_of_idle_frame;
  // float nb_stop;
  // switch(state.stop_bit_tester){
  //   case 0b00 ://1 stop bit
  //   nb_stop = 1;
  //   break;
  //   case 0b01 ://2 stop bit
  //   nb_stop = 0.5;
  //   break;
  //   case 0b10 : //2 stop bit
  //   nb_stop = 2;
  //   break;
  //   case 0b11 ://1,5 stop bit
  //   nb_stop = 1.5;
  //   break;
  // }
  // time_of_idle_frame = uint32_t(state.sampling_time_tester * (8*(2-state.OVER8_tester)) * ((state.M_tester?9:8) + nb_stop));
  //
  // MLOG_F(SIM, DBG, "%s:sending idle frame\n",__FUNCTION__);
  // wait(time_of_idle_frame,SC_NS);
  // MLOG_F(SIM, DBG, "%s:idle frame sent\n",__FUNCTION__);
  //
  // //////////////////SENDING DATA FRAME
  // state.USART_TDR_SR_tester = 0x15; //Copy of data register in shift registers
  //
  // //Sending data in USART_DR_SR
  // for (bit_count=0; bit_count <= (state.M_tester?9:8); bit_count++){
  //
  //   if(bit_count==0){//start bit
  //     p_uart_tx.sc_p = false;
  //   }
  //   else{
  //     MLOG_F(SIM, DBG,"%s: USART-TESTER-TX:%d\n", __FUNCTION__,state.USART_DR_SR_tester & 1);
  //     p_uart_tx.sc_p = state.USART_DR_SR_tester & 1; //send the LSB bit of SR
  //     state.USART_DR_SR_tester = state.USART_DR_SR_tester >> 1; //shifting the shift register
  //   }
  //   wait(state.sampling_time_tester * (8*(2-state.OVER8_tester)),SC_NS); //wait for the next bit to send
  // }
  //
  // //sending stop bit
  // p_uart_tx.sc_p = true;
  //
  // wait(uint32_t(state.sampling_time_tester * (8*(2-(float)state.OVER8_tester))*nb_stop),SC_NS);

}
