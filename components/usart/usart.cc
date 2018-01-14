/*
*  This file is part of Nucleo platforms, Usart component
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

#include "usart.h"

using namespace sc_core;

////////////////////////////////////////////////////////////////////////////////
void usart::read_thread()
{
  wait(NS_BEFORE_SAMPLING,SC_NS); //Little wait, to let the line time to init
  uint32_t sample, bit_count;

  ////////////////////////DATA SAMPLING
  while(1) {
    if(!RE){
      wait(read_mode); //not in reception mode, wait RE posedge
    }
    while(p_uart_rx.sc_p){        //the line is idle, waiting for start bit
      wait(state.sampling_time,SC_NS);
    }
    //Start bit detection!
    for (bit_count=0; bit_count <= (M?9:8); bit_count++){
      for (sample=1; sample <= (OVER8?8:16); sample++){
        if (sample==(OVER8?5:9)){
          //adding input bit in the shifting register
          if(bit_count==0){
            if(p_uart_rx.sc_p){
              //TODO: gestion of false start bit detection
              MLOG_F(SIM, DBG, "%s: USART-RX: Start bit detection Error\n",__FUNCTION__);
            }
          }else{
            MLOG_F(SIM, DBG, "%s: USART-RX:%d :%d\n",__FUNCTION__,bit_count,p_uart_rx.sc_p.read());
              state.USART_DR_SR = ((state.USART_DR_SR >> 1) | (p_uart_rx.sc_p << (M?9:8)-1));
          }
        }
        wait(state.sampling_time,SC_NS);
      }
    }//data is received, and in shift register

    //STOP BIT SAMPLING TO VALIDATE DATA
    //514/841 DocID025350 Rev 4
    int stop_b_reg_v = (state.USART_CR2 >> STOP0_POS) && 0b11;
    switch(stop_b_reg_v){
      case 0 ://1 stop bit
      wait((state.sampling_time * (OVER8?5:9)),SC_NS);
      if(!p_uart_rx.sc_p){
        //TODO: gestion of error stop bit sampling
        MLOG_F(SIM, DBG, "%s:USART: Stop bit detection Error (in:%d config %d)\n",__FUNCTION__,p_uart_rx.sc_p.read(),stop_b_reg_v);
      }
      break;
      /////////////////////
      case 1 ://0,5 stop bit
      //No sampling with 0,5 stop bit, so no error checking
      break;
      /////////////////////
      case 2 : //2 stop bit
      wait((state.sampling_time * (OVER8?5:9)),SC_NS);  //only the first stop bit is check
      if(!p_uart_rx.sc_p){
        //TODO: gestion of error stop bit sampling
        MLOG_F(SIM, DBG, "%s: Stop bit detection Error (in:%d expected %d)\n",__FUNCTION__,p_uart_rx.sc_p.read(),stop_b_reg_v);
      }
      break;
      /////////////////////
      case 3 ://1,5 stop bit
      wait((state.sampling_time * (OVER8?8:17)),SC_NS);
      if(!p_uart_rx.sc_p){
        //TODO: gestion of error stop bit sampling
        MLOG_F(SIM, DBG, "%s: Stop bit detection Error (in:%d expected %d)\n",__FUNCTION__,p_uart_rx.sc_p.read(),stop_b_reg_v);
      }
      break;
      /////////////////////
    }

    state.USART_SR_read=false;  //unvalidate the SR register. Use to check software sequence for reseting flag. Exemple 544/841 DocID025350 Rev 4

    //update USART_RDR_SR, the data are in the shift register

    //PARITY CHECKING
    if(PCE){  //parity control enable
      int count=0 , i;
      for (i=0; i<((M?8:7)) ; i++){
        count+=(state.USART_DR_SR>>i)&1; //counting set bits in data
      }
      if ((count%2 + (PS))!=state.USART_DR_SR>>(M?9:8)){  //MSB is parity bit, PS define Odd or Even
        MLOG_F(SIM, DBG, "%s: ERROR: parity check, PE set\n",__FUNCTION__);
        state.USART_SR|=(1<<PE_POS); //Set PE parity error bit in status register
      }else{
        MLOG_F(SIM, DBG, "%s: parity check OK\n",__FUNCTION__);
      }
      state.USART_DR = state.USART_DR_SR & (M?255:127); //masking of MSB parity bit, mask depend on length
    }else state.USART_DR = state.USART_DR_SR;
    MLOG_F(SIM, DBG, "%s: USART_DR update complete (%x)\n",__FUNCTION__,state.USART_DR);
    //update RXNE in USART_SR:
    state.USART_SR |= 1<<RXNE_POS;
    irq_update.notify();
  }
}


////////////////////////////////////////////////////////////////////////////////
void usart::send_thread()
{
  unsigned int bit_count;
  p_uart_tx.sc_p = true;  //init output tx port

  while(1){
    uint32_t time_of_idle_frame;
    float nb_stop;

    //wake up on TE posedge
    wait(request_idle);

    MLOG_F(SIM, DBG, "%s: request idle frame\nM:%d (%d bit)\nOVER8:%d (%d sample per bit)\nb_stop %x\nsampling_time:%d\n",__FUNCTION__,M,M?9:8,OVER8,OVER8?8:16,((state.USART_CR2 >> STOP0_POS) && 0b11),state.sampling_time);
    //idle frame: frame full of 1
    wait((state.sampling_time * (8*(2-(OVER8)))),SC_NS); //1 bit time delay berfore transmission start

    p_uart_tx.sc_p = true;  //1 on output tx port

    //calculation of idle frme time, depend on oversampling method, number of data bit and stop bit
    int stop_b_reg_v = (state.USART_CR2 >> STOP0_POS) && 0b11;
    switch(stop_b_reg_v){
      case 0 ://1 stop bit
      nb_stop = 1;
      break;
      case 1 ://2 stop bit
      nb_stop = 0.5;
      break;
      case 2 : //2 stop bit
      nb_stop = 2;
      break;
      case 3 ://1,5 stop bit
      nb_stop = 1.5;
      break;
    }
    time_of_idle_frame = uint32_t(state.sampling_time * (8*(2-(OVER8))) * (((M)?9:8) + nb_stop));

    wait(time_of_idle_frame,SC_NS);

    MLOG_F(SIM, DBG, "%s: idle frame sent\n",__FUNCTION__);

    //IDLE FRAME SENT, NOW DATA MANAGEMENT

    while(TE){  //while we stay in transmission mode
      while(TXE){ //at first rising of TE, TXE is low but no data as been provided to the USART, so first send:unconditional wait of TXE event
        MLOG_F(SIM, DBG, "%s: wait for new data (TXE:%d)\n",__FUNCTION__,TXE);
        wait(TXE_event);  //waiting for new data to send, TXE event and TXE clear by writing in USART_DR
      }
      if(!TE){
        break; //TXE event bu TE no more set, need to stop sending thread. Return to wait request_idle
        MLOG_F(SIM, DBG, "%s: TE reset, stop sending\n",__FUNCTION__);
      }
      MLOG_F(SIM, DBG, "%s: New data to send (0x%x) \n",__FUNCTION__,state.USART_DR);
      state.USART_SR &= (~1)<<TC_POS;  //sending frame starting, clear transmission complete flag

      state.USART_DR_SR = state.USART_DR; //Copy of data register in shift registers
      state.USART_DR = 0x0;

      state.USART_SR |= 1<<TXE_POS; //Transmit data register is empty, it can be over-write without data loss
      MLOG_F(SIM, DBG, "%s: data copy in shift register (TXE:%d)\n",__FUNCTION__,TXE);
      irq_update.notify();  //update irq, if TXIE and TE, should raise an irq

      //Sending data in USART_DR_SR
      for (bit_count=0; bit_count <= (M?9:8); bit_count++){

        if(bit_count==0){//start bit
          p_uart_tx.sc_p = false;
        }
        else{
          p_uart_tx.sc_p = state.USART_DR_SR & 1; //send the LSB bit of SR
          MLOG_F(SIM, DBG,"%s: USART-TX:%d\n", __FUNCTION__,state.USART_DR_SR & 1);
          state.USART_DR_SR = state.USART_DR_SR >> 1; //shifting the shift register
        }

        wait(state.sampling_time * (8*(2-(OVER8))),SC_NS); //wait for the next bit to send
      }

      //sending stop bit
      p_uart_tx.sc_p = true;
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
void usart::usart_init_register(void)
{
  memset(&state, 0, sizeof(state));
  state.USART_SR    = USART_SR_RST_VALUE;
  state.USART_DR    = USART_DR_RST_VALUE;
  state.USART_DR_SR = USART_DR_RST_VALUE;
  state.USART_BRR   = USART_BRR_RST_VALUE;
  state.USART_CR1   = USART_CR1_RST_VALUE;
  state.USART_CR2   = USART_CR2_RST_VALUE;
  state.USART_CR3   = USART_CR3_RST_VALUE;
  state.USART_GTPR  = USART_GTPR_RST_VALUE;
}


////////////////////////////////////////////////////////////////////////////////
usart::usart(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
: Slave(name, params, c)
, p_irq("irq")
, p_uart_rx("usart-rx")
, p_uart_tx("usart-tx")
{
  usart_init_register();

  SC_THREAD(read_thread)
  SC_THREAD(send_thread);
  SC_THREAD(irq_update_thread);
}

usart::~usart()
{
}

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

    MLOG_F(SIM, DBG, "%s - %s\n", __FUNCTION__, (flags != 0) ? "1" : "0");

    p_irq.sc_p = (flags != 0);
  }
}

void usart::bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr)
{
  uint32_t value = *((uint32_t *) data + 0);

  bErr = false;

  MLOG_F(SIM, DBG, "%s: to 0x%lx - value 0x%lx\n", __FUNCTION__, (unsigned long) ofs,
  (unsigned long) value);

  switch (ofs) {

    case USART_SR_OFS   :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_SR_MSK_Res) != (USART_SR_RST_VALUE & USART_SR_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_SR_MSK_RC0) ^ (state.USART_SR & USART_SR_MSK_RC0)) & (state.USART_SR & USART_SR_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_SR_MSK_R) != (state.USART_SR & USART_SR_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
    {
      bErr = true;
      break;
    }

    state.USART_SR = value;

    break;
    case USART_DR_OFS   :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_DR_MSK_Res) != (USART_DR_RST_VALUE & USART_DR_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_DR_MSK_RC0) ^ (state.USART_DR & USART_DR_MSK_RC0)) & (state.USART_DR & USART_DR_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_DR_MSK_R) != (state.USART_DR & USART_DR_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
    {
      bErr = true;
      break;
    }

    state.USART_DR = value;
    //writing in DR clear TXE bit
    state.USART_SR &= ~1<<TXE_POS;
    MLOG_F(SIM, DBG, "%s: DR change: reset TXE (%d)\n",__FUNCTION__,TXE);
    irq_update.notify();
    TXE_event.notify(); //notifying sending thread that new data is available

    break;

    case USART_BRR_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_BRR_MSK_Res) != (USART_BRR_RST_VALUE & USART_BRR_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_BRR_MSK_RC0) ^ (state.USART_BRR & USART_BRR_MSK_RC0)) & (state.USART_BRR & USART_BRR_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_BRR_MSK_R) != (state.USART_BRR & USART_BRR_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
    {
      bErr = true;
      break;
    }

    if ((OVER8)){ //DocID025350 Rev 4 545/841
      if ((value>>4)&1){
        MLOG_F(SIM, DBG, "%s: ERROR: OVER8 is set, DIV_Fraction[3] must kept clear \n",__FUNCTION__);
        value&=(~(1<<4)); //force  DIV_Fraction[3] clear
      }
    }
    state.USART_BRR = value;
    state.USARTDIV = DIV_MANTISSA + ((float)DIV_FRACTION / ((OVER8)?8:16));  //caclul of USARTDIV value once for all
    state.sampling_time = uint32_t((float)((state.USARTDIV*1000000000))/FCLK); //sampling time calculation, in ms

    MLOG_F(SIM, DBG, "%s: new USARTDIV value:%f\nnew oversampling time:%u\n",__FUNCTION__,state.USARTDIV,state.sampling_time);


    break;

    case USART_CR1_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_CR1_MSK_Res) != (USART_CR1_RST_VALUE & USART_CR1_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_CR1_MSK_RC0) ^ (state.USART_CR1 & USART_CR1_MSK_RC0)) & (state.USART_CR1 & USART_CR1_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_CR1_MSK_R) != (state.USART_CR1 & USART_CR1_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
    {
      // printf("res: %d \n",((value & USART_CR1_MSK_Res) != USART_CR1_RST_VALUE));
      //      //les bits reservés doivent rester aux valeurs de reset
      // printf("RC0: %d \n",    ((((value & USART_CR1_MSK_RC0) ^ (state.USART_CR1 & USART_CR1_MSK_RC0)) & (state.USART_CR1 & USART_CR1_MSK_RC0)) != 0 ));
      //     //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
      // printf("Readonly: %d \n",    ((value & USART_CR1_MSK_R) != (state.USART_CR1 & USART_CR1_MSK_R) ));
      bErr = true;
      break;
    }

    if((value >> RE_POS &1) && !RE){ //posedge on TE bit
      read_mode.notify();  //send idle frame to init transmissions
    }

    if((value >> TE_POS &1) && !TE){ //posedge on TE bit
      state.USART_SR |= 1<<TXE_POS; //set TXE when enabling transmission mode, overwhise may send an unknow frame
      request_idle.notify();  //send idle frame to init transmissions
    }

    state.USART_CR1 = value;  //update CR1




    break;

    case USART_CR2_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_CR2_MSK_Res) != (USART_CR2_RST_VALUE & USART_CR2_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_CR2_MSK_RC0) ^ (state.USART_CR2 & USART_CR2_MSK_RC0)) & (state.USART_CR2 & USART_CR2_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_CR2_MSK_R) != (state.USART_CR2 & USART_CR2_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
    {
      bErr = true;
      break;
    }
    state.USART_CR2 = value;
    break;

    case USART_CR3_OFS  :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_CR3_MSK_Res) != (USART_CR3_RST_VALUE & USART_CR3_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_CR3_MSK_RC0) ^ (state.USART_CR3 & USART_CR3_MSK_RC0)) & (state.USART_CR3 & USART_CR3_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_CR3_MSK_R) != (state.USART_CR3 & USART_CR3_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
    {
      bErr = true;
      break;
    }
    state.USART_CR3 = value;
    break;

    case USART_GTPR_OFS :///////////////////////////////////////////////////
    ///////////////VERIFICATION DES DROITS D'UTILISATION DES REGISTRES
    if  (((value & USART_GTPR_MSK_Res) != (USART_GTPR_RST_VALUE & USART_GTPR_MSK_Res))|
    //les bits reservés doivent rester aux valeurs de reset
    ((((value & USART_GTPR_MSK_RC0) ^ (state.USART_GTPR & USART_GTPR_MSK_RC0)) & (state.USART_GTPR & USART_GTPR_MSK_RC0)) != 0 )|
    //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
    ((value & USART_GTPR_MSK_R) != (state.USART_GTPR & USART_GTPR_MSK_R) ))
    //les bits read only doivent rester aux valeurs precedente
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
}


////////////////////////////////////////////////////////////////////////////////
void usart::bus_cb_read(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr)
{
  uint32_t *pdata = (uint32_t *) data;

  bErr = false;

  switch (ofs) {

    case USART_SR_OFS   :
    *pdata = state.USART_SR;
    break;
    case USART_DR_OFS   :
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

  MLOG_F(SIM, DBG, "%s: to 0x%lx value %x\n", __FUNCTION__, (unsigned long) ofs, *pdata);

}
