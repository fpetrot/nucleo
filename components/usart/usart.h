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

#ifndef _usart_H
#define _usart_H

#include <rabbits/component/slave.h>
#include <rabbits/component/port/out.h>
#include "usartPort.h"

#define READ_BUF_SIZE           1

#define AMBA_CID 0xB105F00D
#define PID 0x00041011

//offset des registres
#define USART_SR_OFS                0x00
#define USART_DR_OFS                0x04
#define USART_BRR_OFS               0x08
#define USART_CR1_OFS               0x0C
#define USART_CR2_OFS               0x10
#define USART_CR3_OFS               0x14
#define USART_GTPR_OFS              0x18


//mask bit reserved (Res: reserved)
#define USART_SR_MSK_Res            0xFFFFFC00
#define USART_DR_MSK_Res            0xFFFFFF00
#define USART_BRR_MSK_Res           0xFFFF0000
#define USART_CR1_MSK_Res           0xFFFF4000
#define USART_CR2_MSK_Res           0xFFFF8090
#define USART_CR3_MSK_Res           0xFFFFF000
#define USART_GTPR_MSK_Res          0xFFFF0000

//mask bit clearable  (RC0: read clear to 0)
#define USART_SR_MSK_RC0            0x00000360
#define USART_DR_MSK_RC0            0x00000000
#define USART_BRR_MSK_RC0           0x00000000
#define USART_CR1_MSK_RC0           0x00000000
#define USART_CR2_MSK_RC0           0x00000000
#define USART_CR3_MSK_RC0           0x00000000
#define USART_GTPR_MSK_RC0          0x00000000

//mask bit read-only (R: read only)
#define USART_SR_MSK_R              0x0000009F
#define USART_DR_MSK_R              0x00000000
#define USART_BRR_MSK_R             0x00000000
#define USART_CR1_MSK_R             0x00000000
#define USART_CR2_MSK_R             0x00000000
#define USART_CR3_MSK_R             0x00000000
#define USART_GTPR_MSK_R            0x00000000

//mask bit read/write (RW: read write)
#define USART_SR_MSK_RW             0x00000000
#define USART_DR_MSK_RW             0x000000FF
#define USART_BRR_MSK_RW            0x0000FFFF
#define USART_CR1_MSK_RW            0x0000AFFF
#define USART_CR2_MSK_RW            0x00007F7F
#define USART_CR3_MSK_RW            0x00000FFF
#define USART_GTPR_MSK_RW           0x0000FFFF

//////////////////////////////////////////////
//Reset Value of the registers
#define USART_SR_RST_VALUE          0x00C00000
#define USART_DR_RST_VALUE          0x00000000
#define USART_BRR_RST_VALUE         0x00000000
#define USART_CR1_RST_VALUE         0x00000000
#define USART_CR2_RST_VALUE         0x00000000
#define USART_CR3_RST_VALUE         0x00000000
#define USART_GTPR_RST_VALUE        0x00000000


///////////////////////////////////////////USART_SR bit access
// bit num USART_SR
#define CTS_POS                 9
#define LBD_POS                 8
#define TXE_POS                 7
#define TC_POS                  6
#define RXNE_POS                5
#define IDLE_POS                4
#define ORE_POS                 3
#define NF_POS                  2
#define FE_POS                  1
#define PE_POS                  0
//for reading bit USART_SR
#define CTS                     state.USART_SR >> CTS_POS   & 1
#define LBD                     state.USART_SR >> LBD_POS   & 1
#define TXE                     state.USART_SR >> TXE_POS   & 1
#define TC                      state.USART_SR >> TC_POS    & 1
#define RXNE                    state.USART_SR >> RXNE_POS  & 1
#define IDLE                    state.USART_SR >> IDLE_POS  & 1
#define ORE                     state.USART_SR >> ORE_POS   & 1
#define NF                      state.USART_SR >> NF_POS    & 1
#define FE                      state.USART_SR >> FE_POS    & 1
#define PE                      state.USART_SR >> PE_POS    & 1

///////////////////////////////////////////USART_CR1 bit access
// bit num USART_CR1
#define OVER8_POS               15
#define UE_POS                  13
#define M_POS                   12
#define WAKE_POS                11
#define PCE_POS                 10
#define PS_POS                  9
#define PEIE_POS                8
#define TXIE_POS                7
#define TCIE_POS                6
#define RXNEIE_POS              5
#define IDLEIE_POS              4
#define TE_POS                  3
#define RE_POS                  2
#define RWU_POS                 1
#define SBK_POS                 0
//for reading bit USART_CR1
#define OVER8                   state.USART_CR1 >> OVER8_POS  & 1
#define UE                      state.USART_CR1 >> UE_POS     & 1
#define M                       state.USART_CR1 >> M_POS      & 1
#define WAKE                    state.USART_CR1 >> WAKE_POS   & 1
#define PCE                     state.USART_CR1 >> PCE_POS    & 1
#define PS                      state.USART_CR1 >> PS_POS     & 1
#define PEIE                    state.USART_CR1 >> PEIE_POS   & 1
#define TXIE                    state.USART_CR1 >> TXIE_POS   & 1
#define TCIE                    state.USART_CR1 >> TCIE_POS   & 1
#define RXNEIE                  state.USART_CR1 >> RXNEIE_POS & 1
#define IDLEIE                  state.USART_CR1 >> IDLEIE_POS & 1
#define TE                      state.USART_CR1 >> TE_POS     & 1
#define RE                      state.USART_CR1 >> RE_POS     & 1
#define RWU                     state.USART_CR1 >> RWU_POS    & 1
#define SBK                     state.USART_CR1 >> SBK_POS    & 1

///////////////////////////////////////////USART_CR2 bit access
// bit num USART_CR2
#define LINEN_POS               14
#define STOP1_POS               13
#define STOP0_POS               12
#define CLKEN_POS               11
#define CPOL_POS                10
#define CPHA_POS                9
#define LBCL_POS                8
#define LBDIE_POS               6
#define LBLD_POS                5
#define ADD3_POS                3
#define ADD2_POS                2
#define ADD1_POS                1
#define ADD0_POS                0
//for reading bit USART_CR2
#define LINEN                   state.USART_CR2 >> LINEN_POS  & 1
#define STOP1                   state.USART_CR2 >> STOP1_POS  & 1
#define STOP0                   state.USART_CR2 >> STOP0_POS  & 1
#define CLKEN                   state.USART_CR2 >> CLKEN_POS  & 1
#define CPOL                    state.USART_CR2 >> CPOL_POS   & 1
#define CPHA                    state.USART_CR2 >> CPHA_POS   & 1
#define LBCL                    state.USART_CR2 >> LBCL_POS   & 1
#define LBDIE                   state.USART_CR2 >> LBDIE_POS  & 1
#define LBLD                    state.USART_CR2 >> LBLD_POS   & 1
#define ADD3                    state.USART_CR2 >> ADD3_POS   & 1
#define ADD2                    state.USART_CR2 >> ADD2_POS   & 1
#define ADD1                    state.USART_CR2 >> ADD1_POS   & 1
#define ADD0                    state.USART_CR2 >> ADD0_POS   & 1

///////////////////////////////////////////USART_CR3 bit access
// bit num USART_CR3
#define ONEBIT_POS              11
#define CTSIE_POS               10
#define CTSE_POS                9
#define RTSE_POS                8
#define DMAT_POS                7
#define DMAR_POS                6
#define SCEN_POS                5
#define NACK_POS                4
#define HDSEL_POS               3
#define IRLP_POS                2
#define IREN_POS                1
#define EIE_POS                 0
//for reading bit USART_CR3
#define ONEBIT                  state.USART_CR3 >> ONEBIT_POS & 1
#define CTSIE                   state.USART_CR3 >> CTSIE_POS  & 1
#define CTSE                    state.USART_CR3 >> CTSE_POS   & 1
#define RTSE                    state.USART_CR3 >> RTSE_POS   & 1
#define DMAT                    state.USART_CR3 >> DMAT_POS   & 1
#define DMAR                    state.USART_CR3 >> DMAR_POS   & 1
#define SCEN                    state.USART_CR3 >> SCEN_POS   & 1
#define NACK                    state.USART_CR3 >> NACK_POS   & 1
#define HDSEL                   state.USART_CR3 >> HDSEL_POS  & 1
#define IRLP                    state.USART_CR3 >> IRLP_POS   & 1
#define IREN                    state.USART_CR3 >> IREN_POS   & 1
#define EIE                     state.USART_CR3 >> EIE_POS    & 1


#define GT           ((state.USART_GTPR >> 8 ) & 0b11111111)       // [15:8]
#define PCS          ((state.USART_GTPR      ) & 0b11111111)       // [7:0]
#define DR           ((state.USART_DR        ) & 0b11111111)       // [7:0]
#define DIV_MANTISSA ((state.USART_BRR >> 4  ) & 0b111111111111)   // [15:4]
#define DIV_FRACTION ((state.USART_BRR       ) & 0b1111)           // [3:0]

// #define UART_MASK_TX(reg)       ((reg >> 5) & 1)
// #define UART_MASK_RX(reg)       ((reg >> 4) & 1)

struct tty_state
{
        uint32_t USART_SR;
        //uint32_t USART_DR;
        uint32_t USART_TDR; //transmit data register
        uint32_t USART_RDR; //receive data register

        uint32_t USART_BRR;
        uint32_t USART_CR1;
        uint32_t USART_CR2;
        uint32_t USART_CR3;
        uint32_t USART_GTPR;

        uint8_t read_buf[READ_BUF_SIZE];
        int read_pos;
        int read_count;


};

class usart : public Slave<>
{
public:
SC_HAS_PROCESS (usart);
usart(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
virtual ~usart();

private:
void bus_cb_read(uint64_t addr, uint8_t *data, unsigned int len,
                 bool &bErr);
void bus_cb_write(uint64_t addr, uint8_t *data, unsigned int len,
                  bool &bErr);

sc_core::sc_event irq_update;
void read_thread();
void irq_update_thread();

void usart_init_register(void);

public:
OutPort<bool> p_irq;
UsartPort p_uart;

private:
sc_core::sc_event evRead;

tty_state state;
};

#endif
