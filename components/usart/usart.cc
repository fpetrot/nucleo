/*
*  This file is part of Nucleo platforms, Usart component
*  Copyright (C) 2017 Joris Collomb from SLE ENSIMAG
*  Contact Joris.Collomb@gmail.com in case of trouble
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

#include "usart.h"

using namespace sc_core;


////////////////////////////////////////////////////////////////////////////////
usart::usart(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
: Slave(name, params, c)
, p_irq("irq")
, p_uart_rx("usart-rx")
, p_uart_tx("usart-tx")
, p_uart_sclk("usart-sclk")
, p_uart_nCTS("usart-nCTS")
, p_uart_nRTS("usart-nRTS")
{
  fclk = params["fclk"].as<uint32_t>();
  //OVERWISE SEND REQUEST TO RCC TO HAVE THE APB FREQUENCY. BUT HAS WE ARE SLAVE...

  usart_init_register();

  SC_THREAD(read_thread);
  SC_THREAD(send_thread);
  SC_THREAD(SCLK_thread);
  SC_THREAD(irq_update_thread);

  SC_METHOD(nCTS_update_method);
  sensitive << p_uart_nCTS.sc_p;
}

usart::~usart()
{
}


////////////////////////////////////////////////////////////////////////////////
void usart::usart_init_register(void)
{
  memset(&state, 0, sizeof(state));
  state.USART_SR      = USART_SR_RST_VALUE;
  state.USART_DR      = USART_DR_RST_VALUE;
  state.USART_DR_TSR  = USART_DR_RST_VALUE;
  state.USART_DR_RSR  = USART_DR_RST_VALUE;
  state.USART_BRR     = USART_BRR_RST_VALUE;
  state.USART_CR1     = USART_CR1_RST_VALUE;
  state.USART_CR2     = USART_CR2_RST_VALUE;
  state.USART_CR3     = USART_CR3_RST_VALUE;
  state.USART_GTPR    = USART_GTPR_RST_VALUE;
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////READ THREAD///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
*We use the RX_PORT macro to read the input port,
because we can be in half duplex or full duplex communication
with the macro we don't care, it's as well TX or RX port according the configuration
TODO the RX_PORT macro don't seem to work. Check it!
*/
void usart::read_thread()
{
  uint32_t sample, bit_count;
  bool seeking_for_addr = false;
  float nb_stop;          //need nb_stop to calculate nb_sample_idle

  wait(NS_BEFORE_SAMPLING,SC_NS); //Little wait, to let the line time to init

  ////Reception mode activate
  while(1) {
    if(!RE){
      wait(RE_posedge); //not in reception mode, wait RE posedge
      continue; //retest of RE
    }

    ///////////////////////////MUTE MODE/////////////////////////////
    if(RWU){  //if mute mode, idle frame detection to wake the receiver
      MLOG_F(SIM, INF, "%s: ENTER MUTE MODE\n",__FUNCTION__);
      int nb_sample_high;   //number of high sample detected in a row
      int nb_sample_idle;   //number of sample where the port need to be high to detect an idle frames
      //how many stop assuming the configuration?
      int stop_b_reg_v = (state.USART_CR2 >> STOP0_POS) & 0b11;
      switch(stop_b_reg_v){
        case 0b00 ://1 stop bit
        nb_stop = 1;
        break;
        case 0b01 ://0,5 stop bit
        nb_stop = 0.5;
        break;
        case 0b10 : //2 stop bit
        nb_stop = 2;
        break;
        case 0b11 ://1,5 stop bit
        nb_stop = 1.5;
        break;
      }
      nb_sample_idle = (1 + (M?9:8) + nb_stop);   //nb sample expected for an idle frame
      for(nb_sample_high = 0 ; nb_sample_high < nb_sample_idle ; nb_sample_high++){
        if(!RX_PORT){ //line not high?
        // if(!p_uart_rx.sc_p){ //line not high?
          nb_sample_high = 0;  //not an idle frame, reset the counter and continue looking for idle frame to wake
          wait(state.sampling_time,SC_NS);  //wait for next sample
        }
      }
      MLOG_F(SIM, DBG, "%s: IDLE FRAME DETECTED\n",__FUNCTION__);
      if(WAKE){ //if wake=1 addres mark detectection mode, next data will be an address
        seeking_for_addr = true;
      }else{
        state.USART_CR1 &= ~(1<<RWU_POS);   //reset RWU flag
        MLOG_F(SIM, INF, "%s: EXIT MUTE MODE (RWU reset)\n",__FUNCTION__);
      }
    }

    /////////////////////START BIT DETECTION//////////////////////
    int nb_idle_sample = 0;
    while(RX_PORT){        //the line is idle, waiting for start bit
      nb_idle_sample++;
      wait(state.sampling_time,SC_NS);
      if(nb_idle_sample == (OVER8?8:16) * ((M?9:8) + 1 + nb_stop )) { //length of a frame
        state.USART_SR |= (1<<IDLE_POS);    //still low? set IDLE flag
      }
    }

    //Start bit detected!
    if (WAKE) MLOG_F(SIM, DBG, "%s: Frame detected, testing the address of the node\n",__FUNCTION__);
    state.USART_SR &= ~(1<<IDLE_POS);    //reset IDLE flag

    //////////////////////SAMPLING DATA///////////////////////////
    for (bit_count=0; (bit_count <= (M?9:8))  && (!RWU || seeking_for_addr); bit_count++){     //for each bit,        [RWU break because it need to MUTE the receiver, but not when we are looking for an address]
      uint8_t samples_bit = 0;
      for (sample=1; (sample <= (OVER8?8:16)) && (!RWU || seeking_for_addr); sample++){       //at each sampling time,[RWU break because it need to MUTE the receiver, but not when we are looking for an address]

        if (((sample == (OVER8?4:8))  && !ONEBIT) ||
             (sample == (OVER8?5:9))              ||
            ((sample == (OVER8?6:10)) && !ONEBIT)) {   //the bit are sample 3 times if ONEBIT is low, otherwise just once
          samples_bit=(samples_bit << 1) | RX_PORT;  //add the sample bit
          // printf("sampling: \n RX_PORT:%d\n p_uart_tx:%d\n p_uart_rx:%d\n",RX_PORT,p_uart_tx.sc_p.read(),p_uart_rx.sc_p.read());
        }
        if(sample == (OVER8?7:11)){ //finish sampling, test the sample value
          if( !ONEBIT &&   //if ONEBIT low, need to determine if there is noisy trouble
            !((samples_bit & 0b111) == 0b111 ||
              (samples_bit & 0b111) == 0b000 )){   //Ob111 or 0b000 are a not noisy sample
            MLOG_F(SIM, DBG, "%s:USART: noise/desynch detected during sampling : [%d;%d;%d]\n",__FUNCTION__,(samples_bit >> 0) & 0b1,  (samples_bit >> 1) & 0b1,  (samples_bit >> 2) & 0b1);
            state.USART_SR |= 1<<NF_POS;  //set noise detected flag
            int nb_1;
            for(int i = 0 ; i<3 ; i++){   //we count the number of high sample,
              nb_1 += ((samples_bit >> i ) & 0b1);
            }
            if(nb_1>2)samples_bit=1;    //if >2 the bit is read as high
          }
          //adding input bit in the shifting register
          if(bit_count==0){
            if(samples_bit & 0b1){    //we use the LSB sample bit, there are all the same
              MLOG_F(SIM, DBG, "%s: USART-RX: Start bit detection Error\n",__FUNCTION__);
            }
          }else{
            MLOG_F(SIM, DBG, "%s: USART-RX:%d :%d\n",__FUNCTION__,bit_count,(samples_bit & 0b1));
              state.USART_DR_RSR = ((state.USART_DR_RSR >> 1) | ((samples_bit & 0b1) << ((M?9:8)-1)));
          }
        }
        wait(state.sampling_time,SC_NS);
      }
    }//data is received, and in shift register

    if(RWU && !seeking_for_addr)continue; //if entering mute mode, return to the while(1) statement for idle frame detection to wake the receiver
    //'->it need to be test after each sc_core::wait because the configuration may change..
    //but not we are looking for an address


    ////////////////////PARITY CHECKING///////////////////////////
    if(PCE){  //parity control enable
      int count=0 , i;
      for (i=0; i<((M?8:7)) ; i++){
        count+=(state.USART_DR_RSR>>i)&1; //counting set bits in data
      }
      if (((count+ (PS))%2 )!=state.USART_DR_RSR>>(M?9:8)){  //MSB is parity bit, PS define Odd or Even
        MLOG_F(SIM, DBG, "%s: ERROR: parity check, PE set\n",__FUNCTION__);
        state.USART_SR|=(1<<PE_POS); //Set PE parity error bit in status register
      }else{
        MLOG_F(SIM, DBG, "%s: parity check OK\n",__FUNCTION__);
      }
    }

    //////////////////////UPDATE DATA REGISTER///////////////////
    if(RXNE){ //read data register not empty, need to set overrrun flag (ORE)
      state.USART_SR |= 1<<ORE_POS;   //it will be cleared by sw sequence (read SR read DR )
      MLOG_F(SIM, INF, "%s: USART_DR update not complete: OVERRUN ERROR (SR:%d: overwritten at next frame)\n",__FUNCTION__,state.USART_DR_RSR);

    }else{
      // state.USART_SR |= 1<<TXE_POS;  //FIXME TODO: validate or erase. It's a trick.
                                        //The HAL provided by ARM send a dummy frame before each reception.
                                        //But never set send mode for this.
                                        //So it never stop waiting for TXE raise if HAL is configure in RX mode only...
                                        // If you set TX_RX mode this trick is useless
      state.USART_DR = state.USART_DR_RSR;
      MLOG_F(SIM, INF, "%s: USART_DR update complete (0x%x(%d))\n",__FUNCTION__,state.USART_DR,state.USART_DR);
    }

    //update RXNE in USART_SR:
    state.USART_SR |= 1<<RXNE_POS;

    if (RTSE){
      p_uart_nRTS.sc_p = true;  //set nRTS output if hardware flow control is enable. Will be reset by a read to DR
      wait(nRTS_event);         //data has been read, can continue
      p_uart_nRTS.sc_p = false;
    }



    /////////////////SAMPLING STOP BIT//////////////////////////////
    //514/841 DocID025350 Rev 4
    int stop_b_reg_v = (state.USART_CR2 >> STOP0_POS) && 0b11;
    switch(stop_b_reg_v){
      case 0 ://1 stop bit

      wait(state.sampling_time * (OVER8?4:8),SC_NS);  //we will sample the 4/5/6 or 8/9/10 depend on OVER8
      for(int i=0;i<3;i++){   //the stop bit are sample 3 times
        stop_sampling=(stop_sampling << 1) | RX_PORT;  //add the sample stop bit
        wait(state.sampling_time,SC_NS);
      }
      if(stop_sampling != 0b111){   //0b111 is the non-error sample
        MLOG_F(SIM, DBG, "%s:USART: Stop bit detection Error\n",__FUNCTION__);
        state.USART_SR |= 1<<FE_POS;  //set framing error: we may have detect a fram error
        if(stop_sampling != 0b000){ //(513/841)
          state.USART_SR |= 1<<NF_POS;  //it's a noise problem: set noise detected flag
        }
      }
      break;
      /////////////////////
      case 1 ://0,5 stop bit
      //No sampling with 0,5 stop bit, so no error checking

      //Smartcard NACK signal sending
      if(SCEN && NACK && PE){
        wait(state.sampling_time * (4*(2-(OVER8))),SC_NS); //half baud cycle
        p_uart_tx.sc_p = false;  //line pull low by receiver (it's us here) during stop bit in case of parity error
        wait(state.sampling_time * (8*(2-(OVER8))),SC_NS);  //full baud cycle
        p_uart_tx.sc_p = true;  //line pull low by receiver (it's us here) during stop bit in case of parity error
        //FIXME as sc_p is connect to a non-SC_MANY_WRITERS signal, this will stop the simulation.
        //So for now: smartcard mode with NACK enable in transmission mode in not supported
      }
      break;
      /////////////////////
      case 2 : //2 stop bit

      wait(state.sampling_time * (OVER8?4:8),SC_NS);  //we will sample the 4/5/6 or 8/9/10 depend on OVER8
      for(int i=0;i<3;i++){   //the stop bit are sample 3 times
        stop_sampling=(stop_sampling << 1) | RX_PORT;  //add the sample stop bit
        wait(state.sampling_time,SC_NS);
      }
      if(stop_sampling != 0b111){   //0b111 is the non-error sample
        MLOG_F(SIM, DBG, "%s:USART: Stop bit detection Error\n",__FUNCTION__);
        state.USART_SR |= 1<<FE_POS;  //set framing error: we may have detect a fram error
        if(stop_sampling != 0b000){ //(513/841)
          state.USART_SR |= 1<<NF_POS;  //it's a noise problem: set noise detected flag
        }
      }
      break;
      /////////////////////
      case 3 ://1,5 stop bit

      //Smartcard NACK signal sending
      if(SCEN && NACK && PE){
        wait(state.sampling_time * (4*(2-(OVER8))),SC_NS); //half baud cycle
        p_uart_tx.sc_p = false;  //line pull low by receiver (it's us here) during stop bit in case of parity error
        wait(state.sampling_time * (8*(2-(OVER8))),SC_NS);  //full baud cycle
        p_uart_tx.sc_p = true;  //line pull low by receiver (it's us here) during stop bit in case of parity error
        //FIXME as sc_p is connect to a non-SC_MANY_WRITERS signal, this will stop the simulation.
        //So for now: smartcard mode with NACK enable in transmission mode in not supported

      }else{  //no NACK signal to send, so standard stop bit sampling
        wait(state.sampling_time * (OVER8?8:16),SC_NS);  //we will sample the 8/9/10 or 16/17/18 depend on OVER8
        for(int i=0;i<3;i++){   //the stop bit are sample 3 times
          stop_sampling=(stop_sampling << 1) | RX_PORT;  //add the sample stop bit
          wait(state.sampling_time,SC_NS);
        }
        if(stop_sampling != 0b111){   //0b111 is the non-error sample
          MLOG_F(SIM, DBG, "%s:USART: Stop bit detection Error\n",__FUNCTION__);
          state.USART_SR |= 1<<FE_POS;  //set framing error: we may have detect a fram error
          if(stop_sampling != 0b000){ //(513/841)
            state.USART_SR |= 1<<NF_POS;  //it's a noise problem: set noise detected flag
          }
        }
      }
      break;
      /////////////////////
    }

    /////////////////ADDRESS MARK DETECTION WAKE UP/////////////////////////////
    if(WAKE && seeking_for_addr){ //the data is an addr,
      //the 4 LSb of CR2 are the addr of our node
      if (state.USART_DR_RSR == (state.USART_CR2 & 0b1111)){  //if it match we need to wake up the receiver
        state.USART_CR1 &= ~(1<<RWU_POS);   //reset RWU flag -> wake up
        MLOG_F(SIM, INF, "%s: EXIT MUTE MODE (RWU reset)\n",__FUNCTION__);
      }
    }
    seeking_for_addr = false;
    ////////////////

    if(RWU)continue; //if entering mute mode, return to the while(1) statement for idle frame detection to wake the receiver
    //'->it need to be test after each sc_core::wait because the configuration may change..
    irq_update.notify();
  }
}


////////////////////////////////////////////////////////////////////////////////
//////////////////////SEND THREAD///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void usart::send_thread(){

  unsigned int bit_count;

  while(1){
    uint32_t time_of_idle_frame;
    float nb_stop;

    p_uart_tx.sc_p = true;  //steady state output tx port
    data_composed_sclk = CPOL;   //reset SCLK, to steady value
    SCLK_update.notify();

    //wake up on TE posedge
    wait(TE_posedge);

    //how many stop assuming the configuration?
    int stop_b_reg_v = (state.USART_CR2 >> STOP0_POS) & 0b11;
    switch(stop_b_reg_v){
      case 0b00 ://1 stop bit
      nb_stop = 1;
      break;
      case 0b01 ://0,5 stop bit
      nb_stop = 0.5;
      break;
      case 0b10 : //2 stop bit
      nb_stop = 2;
      break;
      case 0b11 ://1,5 stop bit
      nb_stop = 1.5;
      break;
    }

    wait((state.sampling_time * (8*(2-(OVER8)))),SC_NS); //1 bit time delay berfore transmission start

    if(SCEN){  //posedge on TE must send an idle frame, but not in smartcard mode (SCEN=1), as idle frame not defined in ISO
    }else{//calculation of idle frme time, depend on oversampling method, number of data bit and stop bit
      time_of_idle_frame = uint32_t(state.sampling_time * (8*(2-(OVER8))) * (((M)?9:8) + nb_stop));
      MLOG_F(SIM, DBG, "%s: request idle frame\nM:%d (%d bit)\nOVER8:%d (%d sample per bit)\nb_stop %x\nsampling_time:%d\n",__FUNCTION__,M,(M?9:8),OVER8,(OVER8?8:16),((state.USART_CR2 >> STOP0_POS) & 0b11),state.sampling_time);
      wait(time_of_idle_frame,SC_NS);
      //idle frame: frame full of 1, the line is already high
      MLOG_F(SIM, DBG, "%s: idle frame sent\n",__FUNCTION__);
    }

    //NOW DATA MANAGEMENT

    while(TE){  //while we stay in transmission
      data_composed_sclk = CPOL;   //reset SCLK, to steady value
      SCLK_update.notify();
      while(TXE && !SBK){ //Waiting for new data (not if break frame requested SBK=1)
        MLOG_F(SIM, DBG, "%s: wait for new data (TXE:%d)\n",__FUNCTION__,TXE);
        wait(TXE_event);  //waiting for new data to send, TXE event and TXE clear by writing in USART_DR
      }
      if(!TE){
        MLOG_F(SIM, DBG, "%s: TE reset, stop sending\n",__FUNCTION__);
        break; //TXE event but TE no more set, need to stop sending thread. Return to wait TE_posedge
      }

      if(CTSE && p_uart_nCTS.sc_p){ //Receiver not ready and hardware flow control enable! need to hold before sending
        MLOG_F(SIM, DBG, "%s: nCTS High, receiver not ready\n",__FUNCTION__);
          wait(nCTS_event); //event notify when p_uart_nCTS.sc_p is low
          continue; //return to TE test,
      }

      MLOG_F(SIM, INF, "%s: New %s to send (0x%x) \n",__FUNCTION__,(SBK?"break":"data"),(SBK?0:state.USART_DR));

      if(SBK){ //need to send a break frame!
        int nb_bit_break = (LINEN? 13 : (M? 11:10));  //if LINEN break is 13 bit long, otherwise depending on M bits
        p_uart_tx.sc_p = false;
        wait(state.sampling_time * (8*(2-(OVER8))) * nb_bit_break,SC_NS); //wait other half of the time for the next bit to send
        p_uart_tx.sc_p = true;  //stop bit of the break frame
        state.USART_CR2 &= ~(1<<SBK_POS);   //reset SBK bit during stop bit sending
        wait(state.sampling_time * (8*(2-(OVER8))) * nb_stop,SC_NS); //wait other half of the time for the next bit to send
        continue; //return to test of TE
      }

      state.USART_DR_TSR = state.USART_DR; //Copy of data register in shift registers
      state.USART_DR = 0x0;

      if (SCEN){    //in smartcad mode, delayed by a guaranteed 1/2 baud clock before shifting
        wait(state.sampling_time * (4*(2-(OVER8))),SC_NS);
      }

      state.USART_SR |= 1<<TXE_POS; //Transmit data register is empty, it can be over-write without data loss
      MLOG_F(SIM, DBG, "%s: data copy in shift register (TXE:%d)\n",__FUNCTION__,TXE);
      irq_update.notify();  //update irq, if TXIE and TE, should raise an irq

      /////////////////SENDING DATA in USART_DR_TSR/////////////////////////
      uint8_t parity_count;
      for (bit_count=0; bit_count <= (M?9:8); bit_count++){

        if(bit_count==0){//start bit
          p_uart_tx.sc_p = false;
        }
        else{
          if(PCE && (state.USART_DR_TSR & 1)) parity_count++;

          if(PCE && bit_count == (M?9:8)){  //MSB is parity bit when PCE is set
            p_uart_tx.sc_p = (parity_count + PS )%2;    //send parity bit, PS?Odd:Even
          }else p_uart_tx.sc_p = state.USART_DR_TSR & 1; //send data: the LSB bit of SR

          MLOG_F(SIM, DBG,"%s: USART-TX:%d\n", __FUNCTION__,state.USART_DR_TSR & 1);
          state.USART_DR_TSR = state.USART_DR_TSR >> 1; //shifting the shift register
        }

        // we may need to set SCLK
        if(CLKEN){//SCLK if needed
          if ((bit_count != 0) &&                   // not on bit 0, it's the start bit
             !(PCE   && (bit_count==(M?9:8)))) {    //and not on the parity bit (MSB of data) if PCE=1.
            if((!LBCL && (bit_count==(M?9:8)))){    //don't toggle if LBCL=0 and transimission of MSB
              data_composed_sclk = CPOL;              //SCLK reset to steady state
            }else data_composed_sclk = ((!CPHA && (bit_count == 1))? CPOL: !data_composed_sclk) ;  //CPHA? toggle imediatly, otherwise, toggle after half baud period
            SCLK_update.notify();
          }
        }

        wait(state.sampling_time * (4*(2-(OVER8))),SC_NS); //wait half of the time for the next bit to send

        //because we may need to set SCLK
        if(CLKEN){//SCLK if needed
          if ((bit_count != 0) &&                   // not on bit 0, it's the start bit
             !(PCE   && (bit_count==(M?9:8)))) {    //and not on the parity bit (MSB of data) if PCE=1.
            if((!LBCL && (bit_count==(M?9:8)))){    //don't toggle if LBCL=0 and transimission of MSB
              data_composed_sclk = CPOL;              //SCLK reset to steady state
            }else data_composed_sclk = !data_composed_sclk; //set SCLK at the midle of the data bit, see fig 179  531/841 DocID025350 Rev 4
            SCLK_update.notify();
          }
        }

        wait(state.sampling_time * (4*(2-(OVER8))),SC_NS); //wait other half of the time for the next bit to send

      }//-----> Data has been sent

      //sclk return to steady value
      data_composed_sclk = CPOL;   //reset SCLK before stop bits
      SCLK_update.notify();

      ////////////////////////STOP BITS//////////////////////
      p_uart_tx.sc_p = true;

      if(SCEN){   //Need to sample NACK signal from the receiver
        wait(state.sampling_time * (OVER8?8:16),SC_NS);
        for(int i=0;i<3;i++){   //the stop bit are sample 3 times (534/841)
          stop_sampling=(stop_sampling << 1) | p_uart_tx.sc_p;  //add the sample stop bit
          wait(state.sampling_time,SC_NS);
        }
        if(stop_sampling != 0b111){   //0b111 is the non-error sample
          state.USART_SR |= 1<<FE_POS;  //set framing error: we may have detect a NACK signal
          if(stop_sampling != 0b000){ //(513/841)
            state.USART_SR |= 1<<NF_POS;  //oh no, it's a noise problem: set noise detected flag
          }
        }
      }

      wait(state.sampling_time * (8*(2-(OVER8)))*nb_stop,SC_NS);

      if(TXE){
        state.USART_SR |= 1<<TC_POS;  //sending frame complete, if TXE is still set (no new data) set TC
        MLOG_F(SIM, DBG, "%s: set TC because no new data (TXE:%d)\n",__FUNCTION__,TXE);
      }
      irq_update.notify();
    }
    MLOG_F(SIM, DBG, "%s: Quit sending mode\n",__FUNCTION__);
  }

}



////////////////////////////////////////////////////////////////////////////////
//////////////////////////SCLK THREAD////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void usart::SCLK_thread(){
  while(1){

    ///////////////////////PRECONDITION/////////////////////////////////////////

    if (!SCEN && CLKEN){  //SCLK needed in normal mode, just need to apply the value composed by the sending thread to the outport
      p_uart_sclk.sc_p = data_composed_sclk;
      wait(SCLK_update); //wait for change on SCLK status, or config modification
      // wait(state.sampling_time?state.sampling_time:1000,SC_NS); //if sampling time already calculate, otherwise arbitrary 1000ns
      continue;
    }

    if(!(SCEN && CLKEN)){  //we are not in smartcard mode with clock?
      //we don't need this thread, wait for one of those control bit
      wait(SCLK_update);
      continue;
    }

    if ((state.USART_GTPR & 0b11111) == 0){
      MLOG_F(SIM, ERR, "%s: GTPR:PSC is 0x0, do not program this value\n",__FUNCTION__);
      wait(state.sampling_time, SC_NS);
      continue;
    }

    /////////////////////SCLK HANDLING//////////////////////////////////////////

    wait(((state.USART_GTPR & 0b11111)*2)/((fclk*2)/1000000),SC_NS); //first half period wait of doubly divided system clock

    if(SCEN && CLKEN){  //clock for smartcard mode: toggle sclk out port
      p_uart_sclk.sc_p = !p_uart_sclk.sc_p;
    }

    wait(((state.USART_GTPR & 0b11111)*2)/((fclk*2)/1000000),SC_NS); //second half period wait of doubly divided system clock

    if(SCEN && CLKEN){  //clock for smartcard mode: toggle sclk out port
      p_uart_sclk.sc_p = !p_uart_sclk.sc_p;
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
//////////////////////////IrDa THREAD///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
Thread to drive IrDA port with the usart tx
//TODO delete IrDA facility, it seem to be manage by hardware outside the usart,
//so nothing to do with IrDA
*/
// void usart::IrDa_thread(){
//   while(1){
//
//     ///////////////////////PRECONDITION/////////////////////////////////////////
//     if(!IREN){  //we are not in IrDA?
//       //we don't need this thread, wait for one of those control bit
//       wait(IrDA_update);
//       continue;
//     }
//
//     if ((state.USART_GTPR & 0b11111111) == 0){
//       MLOG_F(SIM, ERR, "%s: GTPR:PSC is 0x0, do not program this value\n",__FUNCTION__);
//       wait(state.sampling_time, SC_NS);
//       continue;
//     }
//
  //     //////////////////////IrDA HANDLING/////////////////////////////////////////
//     //two case, Low Power or normal 3/16 period
//
//     if (!IRLP){ //not in low power, IrDa input and output should modulate 0' as 3/16 of bit period
//
//
//
//     }
//
//     if (IRLP){ //not in low power, IrDa input and output should modulate 0' as pulse with freq diveded by PSC
//
//     }
//   }
// }



////////////////////////////////////////////////////////////////////////////////
//////////////////////////nCTS METHOD////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void usart::nCTS_update_method()
{
  if(!p_uart_nCTS.sc_p){
    nCTS_event.notify();
  }
}
////////////////////////////////////////////////////////////////////////////////
//////////////////////////IRQ THREAD////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void usart::irq_update_thread()
{
  unsigned long flags;

  while (1) {

    wait(irq_update);


    // MLOG_F(SIM, DBG, "%s : before update %d\n", __FUNCTION__, p_irq.sc_p.read());

    //cf page 541 de DocID025350 Rev 4
    flags = (((TC    && TCIE)   ||
              (TXE   && TXIE)   ||
              (CTS   && CTSIE)) ||

            ( (IDLE  && IDLEIE) ||
              (ORE   && RXNEIE) ||
              (RXNE  && RXNEIE) ||
              (PE    && PEIE)   ||
              (LBD   && LBDIE)  ||
              (( FE || NF || ORE) && EIE && DMAR )));
    // printf("TC:%d\nTXE:%d\nCTS:%d\nIDLE:%d\nORE:%d\nRXNE:%d\nPE:%d\nLBD:%d\n( FE || NF || ORE):%d\n",TC,TXE,CTS,IDLE,ORE,RXNE,PE,LBD,(FE||NF||ORE));

    MLOG_F(SIM, DBG, "%s - %s\n", __FUNCTION__, ((flags != 0) ? "1" : "0"));

    p_irq.sc_p = (flags != 0);
  }
}


//////////////////////READING WRITING REGISTER HANDLING/////////////////////////

void usart::bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr)
{
  uint32_t value = *((uint32_t *) data + 0);

  bErr = false;

  MLOG_F(SIM, DBG, "%s: to 0x%lx - value 0x%lx\n", __FUNCTION__, (unsigned long) ofs,
  (unsigned long) value);

  switch (ofs) {

    case USART_SR_OFS   :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_SR_MSK_Res) != (USART_SR_RST_VALUE & USART_SR_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_SR_MSK_RC0) ^ (state.USART_SR & USART_SR_MSK_RC0)) & (state.USART_SR & USART_SR_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_SR_MSK_R) != (state.USART_SR & USART_SR_MSK_R) ))
    //Read only bit must stay at previous value
    {
      bErr = true;
      break;
    }


    state.USART_SR = value;

    break;
    case USART_DR_OFS   :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_DR_MSK_Res) != (USART_DR_RST_VALUE & USART_DR_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_DR_MSK_RC0) ^ (state.USART_DR & USART_DR_MSK_RC0)) & (state.USART_DR & USART_DR_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_DR_MSK_R) != (state.USART_DR & USART_DR_MSK_R) ))
    //Read only bit must stay at previous value
    {
      bErr = true;
      break;
    }

    if (lastReadSR){  //detection of [read SR, write DR] software sequence
      state.USART_SR &=(  ~(1<<TC_POS) &  //reset TC
                          ~(1<<PE_POS));  //reset PE
    }

    state.USART_DR = value;
    //writing in DR clear TXE bit
    state.USART_SR &= ~1<<TXE_POS;
    MLOG_F(SIM, DBG, "%s: DR change: reset TXE (%d)\n",__FUNCTION__,TXE);
    irq_update.notify();
    TXE_event.notify(); //notifying sending thread that new data is available

    break;

    case USART_BRR_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_BRR_MSK_Res) != (USART_BRR_RST_VALUE & USART_BRR_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_BRR_MSK_RC0) ^ (state.USART_BRR & USART_BRR_MSK_RC0)) & (state.USART_BRR & USART_BRR_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_BRR_MSK_R) != (state.USART_BRR & USART_BRR_MSK_R) ))
    //Read only bit must stay at previous value
    {
      bErr = true;
      break;
    }

    if ((OVER8)){ //DocID025350 Rev 4 545/841
      if ((value>>4)&1){
        MLOG_F(SIM, DBG, "%s: ERROR: OVER8 is set, DIV_Fraction[3] must kept clear \n value:0x%x\n",__FUNCTION__,value);
        value&=(~(1<<4)); //force  DIV_Fraction[3] clear
      }
    }
    state.USART_BRR = value;
    state.USARTDIV = DIV_MANTISSA + ((float)DIV_FRACTION / ((OVER8)?8:16));  //caclul of USARTDIV value once for all
    state.sampling_time = uint32_t((float)((state.USARTDIV*1000000000))/fclk); //sampling time calculation, in ns

    MLOG_F(SIM, DBG, "%s: new USARTDIV value:%f\nnew oversampling time:%u\n",__FUNCTION__,state.USARTDIV,state.sampling_time);


    break;

    case USART_CR1_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_CR1_MSK_Res) != (USART_CR1_RST_VALUE & USART_CR1_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_CR1_MSK_RC0) ^ (state.USART_CR1 & USART_CR1_MSK_RC0)) & (state.USART_CR1 & USART_CR1_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_CR1_MSK_R) != (state.USART_CR1 & USART_CR1_MSK_R) ))
    //Read only bit must stay at previous value
    {bErr = true;
      break;
    }
    //not sure if needed in RE mode
    if ((((value >> RE_POS &1) && !RE) && UE) ||
        (((value >> UE_POS &1) && !UE) && RE)) { //posedge on RE bit when UE set or posedge on UE when RE is set
          RE_posedge.notify();  //send idle frame to init transmissions
      // }
    }

    if ((((value >> TE_POS &1) && !TE) && UE) ||    //posedge on TE bit when UE is set
        (((value >> UE_POS &1) && !UE) && TE) ||    //posedge on UE bit when TE is set
        (((value >> TE_POS &1) && !TE) && ((value >> UE_POS &1) && !UE))) //posedge on UE and TE
        {
          state.USART_SR |= 1<<TXE_POS; //set TXE when enabling transmission mode, overwhise may send an unknow frame
          TE_posedge.notify();  //send idle frame to init transmissions
      // }
    }

    state.USART_CR1 = value;  //update CR1

    break;

    case USART_CR2_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_CR2_MSK_Res) != (USART_CR2_RST_VALUE & USART_CR2_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_CR2_MSK_RC0) ^ (state.USART_CR2 & USART_CR2_MSK_RC0)) & (state.USART_CR2 & USART_CR2_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_CR2_MSK_R) != (state.USART_CR2 & USART_CR2_MSK_R) ))
    //Read only bit must stay at previous value
    {
      bErr = true;
      break;
    }

    if ((value >> CLKEN_POS) & 0b1){  //we need to wake up the thread that handle SCLK in these sitution
      SCLK_update.notify();
    }

    state.USART_CR2 = value;
    break;

    case USART_CR3_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_CR3_MSK_Res) != (USART_CR3_RST_VALUE & USART_CR3_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_CR3_MSK_RC0) ^ (state.USART_CR3 & USART_CR3_MSK_RC0)) & (state.USART_CR3 & USART_CR3_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_CR3_MSK_R) != (state.USART_CR3 & USART_CR3_MSK_R) ))
    //Read only bit must stay at previous value
    {
      bErr = true;
      break;
    }

    if ((value >> SCEN_POS) & 0b1) {  //we need to wake up the thread that handle SCLK in these sitution
      SCLK_update.notify();
    }

    // if (  ((value >> IREN_POS) & 0b1) |
    //       ((value >> IRLP_POS) & 0b1)){
    //   IrDA_update.notify();
    // }
    //TODO delete IrDA facility, it seem to be manage by hardware outside the usart,
    //so nothing to do with IrDA.. Check it, this is unclear in the documentation of the board

    state.USART_CR3 = value;
    break;

    case USART_GTPR_OFS :///////////////////////////////////////////////////
    ///////////////VERIFICATION OF VALUE
    if  (((value & USART_GTPR_MSK_Res) != (USART_GTPR_RST_VALUE & USART_GTPR_MSK_Res))|
    //reserved bit must stay at reset value
    ((((value & USART_GTPR_MSK_RC0) ^ (state.USART_GTPR & USART_GTPR_MSK_RC0)) & (state.USART_GTPR & USART_GTPR_MSK_RC0)) != 0 )|
    //Read_clear bit must stay at previous value or be clear
    ((value & USART_GTPR_MSK_R) != (state.USART_GTPR & USART_GTPR_MSK_R) ))
    //Read only bit must stay at previous value
    {
      bErr = true;
      break;
    }
    state.USART_GTPR = value;
    break;



    default:
    MLOG_F(SIM, ERR, "%s - Error: ofs=0x%X, data=0x%X-%X!\n",
    __PRETTY_FUNCTION__, (unsigned int) ofs,
    (unsigned int) *((uint32_t *) data + 0),
    (unsigned int) *((uint32_t *) data + 1));
    bErr = true;
  }

  lastReadSR = false;

}


////////////////////////////////////////////////////////////////////////////////
void usart::bus_cb_read(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr)
{
  uint32_t *pdata = (uint32_t *) data;

  bErr = false;

  switch (ofs) {

    case USART_SR_OFS   :
    lastReadSR = true;    //bool to detection of read SR write DR software sequence
    *pdata = state.USART_SR;
    break;
    case USART_DR_OFS   :
    state.USART_SR = state.USART_SR & ~(1<<RXNE_POS);  //A read to DR reset RXNE flag
    nRTS_event.notify();  //data has been read, notify the event for the read thread
    if(lastReadSR){    //detection of [read SR, read DR] software sequence
      state.USART_SR &=(~(1<<IDLE_POS)  &   //reset IDLE
                        ~(1<<ORE_POS)   &   //reset ORE
                        ~(1<<NF_POS)    &   //reset NF
                        ~(1<<FE_POS)    &   //reset FE
                        ~(1<<PE_POS)  );    //reset PE
    }
    *pdata = state.USART_DR;
    break;
    case USART_BRR_OFS  :
    *pdata = state.USART_BRR;
    break;
    case USART_CR1_OFS  :
    *pdata = state.USART_CR1;
    break;
    case USART_CR2_OFS  :
    *pdata = state.USART_CR2;
    break;
    case USART_CR3_OFS  :
    *pdata = state.USART_CR3;
    break;
    case USART_GTPR_OFS :
    *pdata = state.USART_GTPR;
    break;

    default:
    MLOG_F(SIM, ERR, "%s - Error: ofs=0x%X!\n", __PRETTY_FUNCTION__,
    (unsigned int) ofs);
    bErr = true;
  }

  if(ofs != USART_SR_OFS) lastReadSR = false; //reset the boolean use to detect software sequence

  MLOG_F(SIM, DBG, "%s: to 0x%lx value %x\n", __FUNCTION__, (unsigned long) ofs, *pdata);

}
