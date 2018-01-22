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

#ifndef _NUCLEO_EXTI_H
#define _NUCLEO_EXIT_H

#include "rabbits/component/slave.h"
#include <rabbits/component/port/out.h>
#include <rabbits/component/port/in.h>
#include <rabbits/component/port/vector.h>

/* EXTI REGISTERS */
#define NUCLEO_EXTI_IMR 0x00
#define NUCLEO_EXTI_EMR 0x04
#define NUCLEO_EXTI_RTSR 0x08
#define NUCLEO_EXTI_FTSR 0x0C
#define NUCLEO_EXTI_SWIER 0x10
#define NUCLEO_EXTI_PR 0x14

/* Mask */
#define NUCLEO_EXTI_REG_MASK 0x0057FFFF


#define NUCLEO_EXTI_IRQ_NUM 16

class NucleoExti : public Slave<>{
 public:
	NucleoExti(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c); 
	virtual ~NucleoExti();

    VectorPort< InOutPort<bool> > p_gpios;
	VectorPort< OutPort<bool> > p_irq;
	

 private:
	void bus_cb_read(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr);
	void bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr);

	void it_thread(); 
	void end_of_elaboration();

	uint32_t m_imr_reg = 0;
	uint32_t m_emr_reg = 0;
	uint32_t m_rtsr_reg = 0;
	uint32_t m_ftsr_reg = 0;
	uint32_t m_swier_reg = 0;
	uint32_t m_pr_reg = 0;

	sc_core::sc_event_or_list m_ev_gpios; 
};

#endif
