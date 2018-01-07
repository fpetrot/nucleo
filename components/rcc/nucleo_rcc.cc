/*
 *  This file is part of Rabbits
 *  Copyright (C) 2017 Clément Chigot
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
#include "nucleo_rcc.h"
#include "rabbits/logger.h"


using namespace sc_core;

NucleoRcc::NucleoRcc(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
	: Slave(name, params, c)
{
}

NucleoRcc::~NucleoRcc(){}

void NucleoRcc::bus_cb_read_32(uint64_t ofs, uint32_t *data, bool &bErr){
	
    MLOG_F(SIM, DBG, "%s to 0x%lx\n", __FUNCTION__, (unsigned long) ofs);
	
	switch(ofs){
	case RCC_CR:
		*data = rcc_cr_reg;
		break;
	case RCC_PLLCFGR:
		*data = rcc_pllcfgr_reg;
		break; 
	case RCC_CFGR:
		*data = rcc_cfgr_reg;
		break;
	case RCC_CIR:
		*data = rcc_cir_reg;
		break;
	case RCC_BDCR:
		*data = rcc_bdcr_reg;
		break;
	case RCC_CSR:
		*data = rcc_csr_reg;
		break; 
	case RCC_SSCGR:
		*data = rcc_sscgr_reg;
		break;
	case RCC_PLLI2SCFGR:
		*data = rcc_plli2scfgr_reg;
		break;
	case RCC_DCKCFGR:
		*data = rcc_dckcfgr_reg;
		break;
	case RCC_AHB1RSTR:
		*data = rcc_ahb1rstr_reg;
		break; 
	case RCC_AHB2RSTR:
		*data = rcc_ahb2rstr_reg;
		break;
	case RCC_APB1RSTR:
		*data = rcc_apb1rstr_reg;
		break;
	case RCC_APB2RSTR:
		*data = rcc_apb2rstr_reg;
		break;
	case RCC_AHB1ENR:
		*data = rcc_ahb1enr_reg;
		break;
	case RCC_AHB2ENR:
		*data = rcc_ahb2enr_reg;
		break;
	case RCC_APB1ENR:
		*data = rcc_apb1enr_reg;
		break;
	case RCC_APB2ENR:
		*data = rcc_apb2enr_reg;
		break;
	case RCC_AHB1LPENR:
		*data = rcc_ahb1lpenr_reg;
		break;
	case RCC_AHB2LPENR:
		*data = rcc_ahb2lpenr_reg;
		break;
	case RCC_APB1LPENR:
		*data = rcc_apb1lpenr_reg;
		break;
	case RCC_APB2LPENR:
		*data = rcc_apb2lpenr_reg; 
		break; 
		
    default:
        MLOG_F(SIM, ERR, "%s - Error: ofs=0x%X!\n", __PRETTY_FUNCTION__,
			   (unsigned int) ofs);
        bErr = true;
		
	}
}

void NucleoRcc::bus_cb_write_32(uint64_t ofs, uint32_t *data, bool &bErr){
	MLOG_F(SIM, DBG, "%s 0x%x to 0x%lx\n", __FUNCTION__,(uint32_t) *data, (unsigned long) ofs);
	
	switch(ofs){
	case RCC_CR:
		rcc_cr_reg &= ~RCC_CR_RW_MASK;
		rcc_cr_reg |= *data & RCC_CR_RW_MASK;
		// Set clock ready directly when set ON
		if( rcc_cr_reg & RCC_CR_HSION)
			rcc_cr_reg |= RCC_CR_HSIRDY;
		if( rcc_cr_reg & RCC_CR_HSEON)
			rcc_cr_reg |= RCC_CR_HSERDY;
		if( rcc_cr_reg & RCC_CR_PLLON)
			rcc_cr_reg |= RCC_CR_PLLRDY;
		if( rcc_cr_reg & RCC_CR_PLLI2SON)
			rcc_cr_reg |= RCC_CR_PLLI2SRDY;
		break;

	case RCC_PLLCFGR:
		rcc_pllcfgr_reg &= ~RCC_PLLCFGR_RW_MASK;
		rcc_pllcfgr_reg |= *data & RCC_PLLCFGR_RW_MASK;
		// Use PLLSRC to set CR_PLLXXX bits
		if( rcc_pllcfgr_reg & RCC_PLLCFGR_PLLSRC)
			rcc_cr_reg |= RCC_CR_PLLON | RCC_CR_PLLRDY; 
		break; 
		
	case RCC_CFGR:{
		uint8_t oldclock = rcc_cfgr_reg & RCC_CFGR_SW;
		rcc_cfgr_reg &= ~RCC_CFGR_RW_MASK;
		rcc_cfgr_reg |= *data & RCC_CFGR_RW_MASK;

		// System clock must be changed 
		if(oldclock != (rcc_cfgr_reg & RCC_CFGR_SW)){ 
			rcc_cfgr_reg &= ~RCC_CFGR_SWS ; // reset system clock status
			rcc_cfgr_reg |= (rcc_cfgr_reg & RCC_CFGR_SW)<<2; // set new system clock status
		}
	
		break;
	}
	case RCC_CIR:
		rcc_cir_reg = *data;
		break;
	case RCC_BDCR:
		rcc_bdcr_reg = *data;
		break;
	case RCC_CSR:
		rcc_csr_reg = *data;
		break; 
	case RCC_SSCGR:
		rcc_sscgr_reg = *data;
		break;
	case RCC_PLLI2SCFGR:
		rcc_plli2scfgr_reg = *data;
		break;
	case RCC_DCKCFGR:
		rcc_dckcfgr_reg = *data;
		break;

	case RCC_AHB1RSTR: 
	case RCC_AHB2RSTR:
	case RCC_APB1RSTR: 
	case RCC_APB2RSTR:
	case RCC_AHB1ENR: 
	case RCC_AHB2ENR:
	case RCC_APB1ENR: 
	case RCC_APB2ENR:
	case RCC_AHB1LPENR: 
	case RCC_AHB2LPENR:
	case RCC_APB1LPENR: 
	case RCC_APB2LPENR:
		/* Not usefull */ 
		break; 

    default:
        MLOG_F(SIM, ERR, "%s - Error: ofs=0x%X!\n", __PRETTY_FUNCTION__,
			   (unsigned int) ofs);
        bErr = true;
		
	}

}
