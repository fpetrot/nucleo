/*
 *  This file is part of Rabbits
 *  Copyright (C) 2015  Clement Chigot
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
#include <inttypes.h>

#include "nucleo_syscfg.h"

#include <rabbits/logger.h>

using namespace sc_core;


NucleoSyscfg::NucleoSyscfg(sc_module_name name, const Parameters &params, ConfigManager &c)
	: Slave(name, params, c)
	, p_out("to-exti")
{

}

NucleoSyscfg::~NucleoSyscfg(){}

void NucleoSyscfg::bus_cb_read(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr){  
	uint32_t *val = (uint32_t*) data;
	switch(ofs){
	case NUCLEO_SYSCFG_MEMRMP:
		*val = m_memrmp_reg;
		break; 
	case NUCLEO_SYSCFG_PMC:
		*val = m_pmc_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR1:
		*val = m_exticr1_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR2:
		*val = m_exticr2_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR3:
		*val = m_exticr3_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR4:
		*val = m_exticr4_reg;
		break; 
	case NUCLEO_SYSCFG_CMPCR:
		*val = m_cmpcr_reg;
		break; 
	default:
		MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
			   (unsigned int) ofs);
		bErr = true;
		return;
	}
	
	MLOG_F(SIM, DBG, "Read ofs: 0x%x - val: 0x%x\n", ofs, *val);
}

void NucleoSyscfg::bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr){
	uint32_t val =*((uint32_t*) data + 0);
	MLOG_F(SIM, DBG, "write to ofs: 0x%x - val: 0x%x\n", ofs, val);

	switch(ofs){
	case NUCLEO_SYSCFG_MEMRMP:
		m_memrmp_reg = val;
		break; 
	case NUCLEO_SYSCFG_PMC:
		m_pmc_reg = val;
		break; 
	case NUCLEO_SYSCFG_EXTICR1:
		m_exticr1_reg = val & NUCLEO_SYSCFG_EXTIx_MASK;
		p_out.sc_p = m_exticr1_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR2:
		m_exticr2_reg = val & NUCLEO_SYSCFG_EXTIx_MASK;
		p_out.sc_p = 0x10000 | m_exticr2_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR3:
		m_exticr3_reg = val & NUCLEO_SYSCFG_EXTIx_MASK;
		p_out.sc_p = 0x20000 | m_exticr3_reg;
		break; 
	case NUCLEO_SYSCFG_EXTICR4:
		m_exticr4_reg = val & NUCLEO_SYSCFG_EXTIx_MASK;
		p_out.sc_p = 0x30000 | m_exticr4_reg;
		break; 
	case NUCLEO_SYSCFG_CMPCR:
		m_cmpcr_reg = val;
		break; 
	default:
		MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
			   (unsigned int) ofs);
		bErr = true;
		return;
	}
}
