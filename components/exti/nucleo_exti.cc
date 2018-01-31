
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

#include "nucleo_exti.h"
#include "../gpio/nucleo_gpio.h"

#include <rabbits/logger.h>

using namespace sc_core;

#define GPIOS_AUTOCONNECT(gpios)										\
	for(auto &p: gpios) {												\
		p.set_autoconnect_to(0);										\
	}																	\

NucleoExti::NucleoExti(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
	: Slave(name, params, c)
	, p_irq("irq", NUCLEO_EXTI_IRQ_NUM)
	, p_cfg("from-syscfg")
	, p_gpios_a("gpios-a", NUCLEO_GPIO_COUNT)	
	, p_gpios_b("gpios-b", NUCLEO_GPIO_COUNT)
	, p_gpios_c("gpios-c", NUCLEO_GPIO_COUNT)
	, p_gpios_d("gpios-d", NUCLEO_GPIO_COUNT)
	, p_gpios_e("gpios-e", NUCLEO_GPIO_COUNT)
	, p_gpios_h("gpios-h", NUCLEO_GPIO_COUNT)
{

	// Connect to 0 to avoid systemc errors
	GPIOS_AUTOCONNECT(p_gpios_a);
	GPIOS_AUTOCONNECT(p_gpios_b);
	GPIOS_AUTOCONNECT(p_gpios_c);
	GPIOS_AUTOCONNECT(p_gpios_d);
	GPIOS_AUTOCONNECT(p_gpios_e);
	GPIOS_AUTOCONNECT(p_gpios_h);
	for(auto &p: p_irq) {
		p.set_autoconnect_to(0);
	}

	SC_THREAD(irq_detection_thread);
	SC_THREAD(irq_update_thread);
	SC_THREAD(cfg_update_thread);
}

NucleoExti::~NucleoExti(){}


#define GPIOS_EVENT_SET(gpios)					\
	for (auto &p : gpios) {						\
		m_ev_gpios |= p.sc_p.default_event();	\
	}											\

void NucleoExti::end_of_elaboration()
{
	GPIOS_EVENT_SET(p_gpios_a);
	GPIOS_EVENT_SET(p_gpios_b);
	GPIOS_EVENT_SET(p_gpios_c);
	GPIOS_EVENT_SET(p_gpios_d);
	GPIOS_EVENT_SET(p_gpios_e);
	GPIOS_EVENT_SET(p_gpios_h);

	// Default gpios configuration
	for(int i = 0; i < NUCLEO_EXTI_IRQ_NUM; i++){
		m_gpios_selected[i] = &p_gpios_a[i]; 
	}

}

void NucleoExti::bus_cb_read(uint64_t ofs, uint8_t *data,  unsigned int len,bool &bErr){
	uint32_t *val = (uint32_t*) data;
	switch(ofs){
	case NUCLEO_EXTI_IMR:
		*val = m_imr_reg;
		break;
	case NUCLEO_EXTI_EMR:
		*val = m_emr_reg;
		break; 
	case NUCLEO_EXTI_RTSR:
		*val = m_rtsr_reg;
		break;
	case NUCLEO_EXTI_FTSR:
		*val = m_ftsr_reg;
		break; 
	case NUCLEO_EXTI_SWIER:
		*val = m_swier_reg;
		break;
	case NUCLEO_EXTI_PR:
		*val = m_pr_reg;
		break; 
	default:
		MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
			   (unsigned int) ofs);
		bErr = true;
		return;
	}
	
	MLOG_F(SIM, DBG, "Read ofs: 0x%x - val: 0x%x\n", ofs, *val);
}

void NucleoExti::bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr){	
	uint32_t val =*((uint32_t*) data + 0);
	MLOG_F(SIM, DBG, "write to ofs: 0x%x - val: 0x%x\n", ofs, val);

	switch(ofs){
	case NUCLEO_EXTI_IMR:
		m_imr_reg = val & NUCLEO_EXTI_REG_MASK;
		break;
	case NUCLEO_EXTI_EMR:
		m_emr_reg = val & NUCLEO_EXTI_REG_MASK;
		break; 
	case NUCLEO_EXTI_RTSR:
		m_rtsr_reg = val & NUCLEO_EXTI_REG_MASK;
		break;
	case NUCLEO_EXTI_FTSR:
		m_ftsr_reg = val & NUCLEO_EXTI_REG_MASK;
		break; 
	case NUCLEO_EXTI_SWIER:
		m_swier_reg = val & NUCLEO_EXTI_REG_MASK;
		break;
	case NUCLEO_EXTI_PR:
	    m_pr_reg &= ~(val & NUCLEO_EXTI_REG_MASK); // Clear when write '1'
		for(int i = 0; i < NUCLEO_EXTI_IRQ_NUM; i++){
			if(m_irq_status[i] && !m_pr_reg){
				m_irq_status[i] = false; 
			}
		}
		m_ev_irq_update.notify(); 
	
		break;
	default:
		MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
			   (unsigned int) ofs);
		bErr = true;
		return;
	}
}

void NucleoExti::irq_detection_thread(){
	uint16_t irq_pending; // each bit is set to '1' if this irq is pending
	
	while(1) {
		wait(m_ev_gpios);
		irq_pending = 0;

		
		// IRQ fetching 
		int i = 0;
		for(auto &p : m_gpios_selected){
			sc_in<bool> &sc_p = p->sc_p;

#if 0
			// Usefull to check if an input has been received
			if(sc_p->posedge())
				std::cout << "posedge " << i << std::endl;
			if(sc_p->negedge())
				std::cout << "negedge " << i << std::endl;
#endif

			if(sc_p->posedge() // rising edge detected
			   && (m_rtsr_reg & ( 1 << i )) && (m_imr_reg & ( 1 << i ))) // and irq needed
				irq_pending |= ( 1 << i ); 
			if(sc_p->negedge() // falling edge detected
			   && (m_ftsr_reg & ( 1 << i )) && (m_imr_reg & ( 1 << i ))) // and irq needed
				irq_pending |= ( 1 << i );

			i++;
		}

		// IRQ generation
		if(irq_pending){
			for(int i = 0; i < NUCLEO_EXTI_IRQ_NUM; i++){
				if(irq_pending & (1 << i)){
					m_pr_reg |= (1 << i); 
					m_irq_status[i] = true; 
				}
			}
			m_ev_irq_update.notify(); 
		}
	}
}

void NucleoExti::irq_update_thread(){
	while(1) {
		wait(m_ev_irq_update);
		/* NOTE :
		 * We need to separate irqs because irq 5-9 go to the same nvic port, same for irq10-15
		 * Cannot be done only with YAML description because QemuInPorts are SC_ONE_WRITERS  
		 */
		// IRQ 0 - 4
		for(int i = 0; i < 5; i++){
		    p_irq[i].sc_p = m_irq_status[i];
		}

		// IRQ 5 - 9
		bool irq5 = m_irq_status[5];
		for(int i = 6; i < 10; i++){
			irq5 |= m_irq_status[i];
		}
		p_irq[5].sc_p = irq5;

		// IRQ 10 - 15
		bool irq10 = m_irq_status[10];
		for(int i = 11; i < 16; i++){
			irq10 |= m_irq_status[i];
		}
		p_irq[10].sc_p = irq10;

	}
}


void NucleoExti::cfg_update_thread(){
	while(1) {
		wait(p_cfg.sc_p.default_event());
		uint32_t newcfg = p_cfg.sc_p.read();
		int gpiosIndex = (newcfg >> 0x10) * 4;

		// update Gpios connected for irq detection 
		for(int i = gpiosIndex; i < gpiosIndex + 4; i++){
			int newgpio = (newcfg >> ((i % 4) * 4)) & 0xF;
			switch(newgpio){
			case 0x0:
				m_gpios_selected[i] = &p_gpios_a[i];
				break; 
			case 0x1:
				m_gpios_selected[i] = &p_gpios_b[i];
				break; 
			case 0x2:
				m_gpios_selected[i] = &p_gpios_c[i];
				break; 
			case 0x3:
				m_gpios_selected[i] = &p_gpios_d[i];
				break; 
			case 0x4:
				m_gpios_selected[i] = &p_gpios_e[i];
				break; 
			case 0x7:
				m_gpios_selected[i] = &p_gpios_h[i];
				break;
			default:
				MLOG_F(SIM, ERR, "Internal Error: wrong new configuration\n", name(), __FUNCTION__);
				break; 
			}
		}


	}
}
