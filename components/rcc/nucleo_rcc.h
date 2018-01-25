/*
 *  This file is part of Rabbits
 *  Copyright (C) 2017  Clement Chigot
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

#ifndef _NUCLEO_RCC_H_
#define _NUCLEO_RCC_H_

#include "rabbits/component/slave.h"

/* RCC REGISTERS */ 
#define NUCLEO_RCC_CR 0x00
#define NUCLEO_RCC_PLLCFGR 0x04
#define NUCLEO_RCC_CFGR 0x08
#define NUCLEO_RCC_CIR 0x0C
#define NUCLEO_RCC_AHB1RSTR 0x10
#define NUCLEO_RCC_AHB2RSTR 0x14
#define NUCLEO_RCC_APB1RSTR 0x20
#define NUCLEO_RCC_APB2RSTR 0x24
#define NUCLEO_RCC_AHB1ENR 0x30
#define NUCLEO_RCC_AHB2ENR 0x34
#define NUCLEO_RCC_APB1ENR 0x40
#define NUCLEO_RCC_APB2ENR 0x44
#define NUCLEO_RCC_AHB1LPENR 0x50
#define NUCLEO_RCC_AHB2LPENR 0x54
#define NUCLEO_RCC_APB1LPENR 0x60
#define NUCLEO_RCC_APB2LPENR 0x64
#define NUCLEO_RCC_BDCR 0x70
#define NUCLEO_RCC_CSR 0x74
#define NUCLEO_RCC_SSCGR 0x80
#define NUCLEO_RCC_PLLI2SCFGR 0x84
#define NUCLEO_RCC_DCKCFGR 0x8C

/* RCC Registers Readonly masks */
#define NUCLEO_RCC_CR_RW_MASK 0x050D00F9
#define NUCLEO_RCC_PLLCFGR_RW_MASK 0x0F437FFF
#define NUCLEO_RCC_CFGR_RW_MASK 0xFFFFFCF3

/* RCC_CR Bits  */
#define NUCLEO_RCC_CR_HSION 0x1
#define NUCLEO_RCC_CR_HSIRDY 0x2
#define NUCLEO_RCC_CR_HSEON 0x10000
#define NUCLEO_RCC_CR_HSERDY 0x20000
#define NUCLEO_RCC_CR_PLLON 0x1000000
#define NUCLEO_RCC_CR_PLLRDY 0x2000000
#define NUCLEO_RCC_CR_PLLI2SON 0x4000000
#define NUCLEO_RCC_CR_PLLI2SRDY 0x8000000


/* RCC_PLLCFGR Bits */
#define NUCLEO_RCC_PLLCFGR_PLLSRC 0x400000

/* RCC_CFGR Bits */
#define NUCLEO_RCC_CFGR_SW 0x3
#define NUCLEO_RCC_CFGR_SWS 0xC




class NucleoRcc: public Slave<>{
 public:
	SC_HAS_PROCESS(NucleoRcc); 
	NucleoRcc(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
	virtual ~NucleoRcc();


 private:
	void bus_cb_read_32(uint64_t ofs, uint32_t *data, bool &bErr);
    void bus_cb_write_32(uint64_t ofs, uint32_t *data, bool &bErr);


	/* Registers with reset value */
	uint32_t rcc_cr_reg = 0x0077083;
	uint32_t rcc_pllcfgr_reg = 0x24003010;
	uint32_t rcc_cfgr_reg = 0x0;
	uint32_t rcc_cir_reg = 0x0;
	uint32_t rcc_ahb1rstr_reg = 0x0; 
	uint32_t rcc_ahb2rstr_reg = 0x0; 
	uint32_t rcc_apb1rstr_reg = 0x0; 
	uint32_t rcc_apb2rstr_reg = 0x0; 
	uint32_t rcc_ahb1enr_reg = 0x0; 
	uint32_t rcc_ahb2enr_reg = 0x0; 
	uint32_t rcc_apb1enr_reg = 0x0; 
	uint32_t rcc_apb2enr_reg = 0x0; 
	uint32_t rcc_ahb1lpenr_reg = 0x0061900F; 
	uint32_t rcc_ahb2lpenr_reg = 0x00000080; 
	uint32_t rcc_apb1lpenr_reg = 0x10E2C80F; 
	uint32_t rcc_apb2lpenr_reg = 0x00077930; 
	uint32_t rcc_bdcr_reg = 0x0;
	uint32_t rcc_csr_reg = 0X0E000000;
	uint32_t rcc_sscgr_reg = 0x0;
	uint32_t rcc_plli2scfgr_reg = 0x24003000;
	uint32_t rcc_dckcfgr_reg = 0x0; 

};

#endif //_NUCLEO_RCC_H_
