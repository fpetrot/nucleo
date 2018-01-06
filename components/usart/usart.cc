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

#include "usart.h"

using namespace sc_core;

void usart::read_thread()
{
    usart_data data_recv;

    while(1) {
      p_uart.recv(data_recv);  //rx->recv(data)
      MLOG_F(SIM, DBG, "%s: got a %d (M:%d) bits/0x%01x Stop  (0x%02x)\n",__FUNCTION__,data_recv.length ?9:8,M ,data_recv.stopBit, data_recv.data);
      if (((M) ^ data_recv.length)){ //nb de bit recu =/= nb de bit attendu
        MLOG_F(SIM, DBG, "%s: got a %d bits but expect %d\n",__FUNCTION__, data_recv.length ?9:8 , M ?9:8);
      }

      //update USART_RDR_SR, the data are in the shift register
      if(PCE){  //parity control enable
        int count=0 , i;
        for (i=0; i<(M?8:7) ; i++){
          count+=(data_recv.data>>i)&1; //counting set bits in data
        }
        if ((count%2)!=data_recv.data>>(M?9:8)){  //MSB is parity bit
          MLOG_F(SIM, DBG, "%s: ERROR: parity check\n",__FUNCTION__);

        }else  state.USART_RDR_SR = data_recv.data & M?255:127; //masking of MSB
      }else state.USART_RDR_SR = data_recv.data;
      //update RXNE in USART_SR:
      state.USART_SR |= 1<<RXNE_POS;
      irq_update.notify();
    }

}
void usart::usart_init_register(void)
{
    memset(&state, 0, sizeof(state));
    state.USART_SR    = USART_SR_RST_VALUE;
    state.USART_TDR   = USART_DR_RST_VALUE;
    state.USART_RDR   = USART_DR_RST_VALUE;
    state.USART_BRR   = USART_BRR_RST_VALUE;
    state.USART_CR1   = USART_CR1_RST_VALUE;
    state.USART_CR2   = USART_CR2_RST_VALUE;
    state.USART_CR3   = USART_CR3_RST_VALUE;
    state.USART_GTPR  = USART_GTPR_RST_VALUE;
}

usart::usart(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
    : Slave(name, params, c)
    , p_irq("irq")
    , p_uart("usart")
{
    usart_init_register();

    SC_THREAD(read_thread);
    SC_THREAD(irq_update_thread);
}

usart::~usart()
{
}

void usart::irq_update_thread()
{
  unsigned long flags;

  while (1) {

      wait(irq_update);

      //cf page 541 de DocID025350 Rev 4
      flags =(( (TC    && TCIE)   ||
                (TXE   && TXIE)   ||
                (CTS   && CTSIE)) ||

              ( (IDLE  && IDLEIE) ||
                (ORE   && RXNEIE) ||
                (RXNE  && RXNEIE) ||
                (PE    && PEIE)   ||
                (LBD   && LBDIE)  ||
                (( FE || NF || ORE) && EIE && DMAR )));

      MLOG_F(SIM, DBG, "%s - %s\n", __FUNCTION__, (flags != 0) ? "1" : "0");

      p_irq.sc_p = (flags != 0);
  }
}

void usart::bus_cb_write(uint64_t ofs, uint8_t *data,
                                  unsigned int len, bool &bErr)
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
            ((((value & USART_DR_MSK_RC0) ^ (state.USART_TDR & USART_DR_MSK_RC0)) & (state.USART_TDR & USART_DR_MSK_RC0)) != 0 )|
            //les bits read clear doivent rester aux valeurs de precedente ou remmetre a 0 un bit du registre
            ((value & USART_DR_MSK_R) != (state.USART_TDR & USART_DR_MSK_R) ))
            //les bits read only doivent rester aux valeurs precedente
        {
          bErr = true;
          break;
        }

        state.USART_TDR = value;
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

        if (OVER8){ //DocID025350 Rev 4 545/841
          if ((value>>4)&1){
            MLOG_F(SIM, DBG, "%s: ERROR: OVER8 is set, DIV_Fraction[3] must kept clear \n",__FUNCTION__);
            value&=(~(1<<4)); //force  DIV_Fraction[3] clear
          }
        }

          else
        state.USART_BRR = value;
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

        state.USART_CR1 = value;  //update CR1

        if(TE){ //si TE: transmision enable à true, alors send contenue TDR
          usart_data data_to_send;
          data_to_send.length=M;
          data_to_send.data=state.USART_TDR;
          data_to_send.stopBit=(state.USART_CR2 >> STOP0_POS) & 0b11;
          MLOG_F(SIM, DBG, "%s: send %dbits/0x%01x Stop  (0x%02x)\n", __FUNCTION__,data_to_send.length?9:8, data_to_send.stopBit, data_to_send.data);
          p_uart.send(data_to_send);
        }



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

void usart::bus_cb_read(uint64_t ofs, uint8_t *data,
                                 unsigned int len, bool &bErr)
{
    uint32_t *pdata = (uint32_t *) data;

    bErr = false;

    switch (ofs) {

      case USART_SR_OFS   :
        *pdata = state.USART_SR;
      break;
      case USART_DR_OFS   :
        *pdata = state.USART_RDR;
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
