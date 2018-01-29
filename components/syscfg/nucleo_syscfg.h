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


#ifndef _NUCLEO_SYSCFG_H
#define _NUCLEO_SYSCFG_H

#include "rabbits/component/slave.h"
#include <rabbits/component/port/out.h>

/* SYSCFG REGISTERS */
#define NUCLEO_SYSCFG_MEMRMP			0x00
#define NUCLEO_SYSCFG_PMC				0x04
#define NUCLEO_SYSCFG_EXTICR1			0x08
#define NUCLEO_SYSCFG_EXTICR2			0x0C
#define NUCLEO_SYSCFG_EXTICR3			0x10
#define NUCLEO_SYSCFG_EXTICR4			0x14
#define NUCLEO_SYSCFG_CMPCR				0x20

/* EXTI BITS */
#define NUCLEO_SYSCFG_EXTICR1_EXTI0		0x000F
#define NUCLEO_SYSCFG_EXTICR1_EXTI1		0x00F0
#define NUCLEO_SYSCFG_EXTICR1_EXTI2		0x0F00
#define NUCLEO_SYSCFG_EXTICR1_EXTI3		0xF000

#define NUCLEO_SYSCFG_EXTICR2_EXTI4		0x000F
#define NUCLEO_SYSCFG_EXTICR2_EXTI5		0x00F0
#define NUCLEO_SYSCFG_EXTICR2_EXTI6		0x0F00
#define NUCLEO_SYSCFG_EXTICR2_EXTI7		0xF000

#define NUCLEO_SYSCFG_EXTICR3_EXTI8		0x000F
#define NUCLEO_SYSCFG_EXTICR3_EXTI9		0x00F0
#define NUCLEO_SYSCFG_EXTICR3_EXTI10	0x0F00
#define NUCLEO_SYSCFG_EXTICR3_EXTI11	0xF000

#define NUCLEO_SYSCFG_EXTICR4_EXTI12	0x000F
#define NUCLEO_SYSCFG_EXTICR4_EXTI13	0x00F0
#define NUCLEO_SYSCFG_EXTICR4_EXTI14	0x0F00
#define NUCLEO_SYSCFG_EXTICR4_EXTI15	0xF000

// MASK
#define NUCLEO_SYSCFG_EXTIx_MASK 0xFFFF


class NucleoSyscfg : public Slave<>{
 public:
	NucleoSyscfg(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c); 
	virtual ~NucleoSyscfg();

	
	OutPort<uint32_t> p_out; 

 private:
	void bus_cb_read(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr);
	void bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr);

	uint32_t m_memrmp_reg = 0;
	uint32_t m_pmc_reg = 0;
	uint32_t m_exticr1_reg = 0;
	uint32_t m_exticr2_reg = 0;
	uint32_t m_exticr3_reg = 0;
	uint32_t m_exticr4_reg = 0;
	uint32_t m_cmpcr_reg = 0;


}; 

#endif
