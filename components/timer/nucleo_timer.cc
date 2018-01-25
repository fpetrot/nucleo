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

#include "nucleo_timer.h"

#include <rabbits/logger.h>

using namespace sc_core;

NucleoTimer::NucleoTimer(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c)
	: Slave(name, params, c)
	, irq("irq")
{
	timer_size = params["timer-size"].as<uint8_t>();
	SC_THREAD(irq_update_thread);
	SC_THREAD(counter_thread);
}

NucleoTimer::~NucleoTimer()
{
}


void NucleoTimer::bus_cb_read(uint64_t ofs, uint8_t *data,  unsigned int len,bool &bErr)
{
	uint32_t *val = (uint32_t*) data;

	switch(ofs){
	case NUCLEO_TIMx_CR1:
		*val = m_cr1_reg;
		break;
	case NUCLEO_TIMx_CR2:
		*val = m_cr2_reg;
		break;
	case NUCLEO_TIMx_SMCR:
		*val = m_smcr_reg;
		break;
	case NUCLEO_TIMx_DIER:
		*val = m_dier_reg;
		break; 
	case NUCLEO_TIMx_SR:
		*val = m_sr_reg;
		break;
	case NUCLEO_TIMx_EGR:
		*val = m_egr_reg;
		break;
	case NUCLEO_TIMx_CCMR1:
		*val = m_ccmr1_reg;
		break;
	case NUCLEO_TIMx_CCMR2:
		*val = m_ccmr2_reg;
		break;
	case NUCLEO_TIMx_CCER:
		*val = m_ccer_reg;
		break;
	case NUCLEO_TIMx_CNT:
		*val = m_cnt_reg;
		break; 
	case NUCLEO_TIMx_PSC:
		*val = m_psc_reg;
		break;
	case NUCLEO_TIMx_ARR:
		*val = m_arr_reg;
		break;
	case NUCLEO_TIMx_CCR1:
		*val = m_ccr1_reg;
		break; 
	case NUCLEO_TIMx_CCR2:
		*val = m_ccr2_reg;
		break;
	case NUCLEO_TIMx_CCR3:
		*val = m_ccr3_reg;
		break;
	case NUCLEO_TIMx_CCR4:
		*val = m_ccr4_reg;
		break;
	case NUCLEO_TIMx_DCR:
		*val = m_dcr_reg;
		break;
	case NUCLEO_TIMx_OR:
		*val = m_or_reg;
		break;
	default:
		MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
			   (unsigned int) ofs);
		bErr = true;
		return;
	}

	MLOG_F(SIM, DBG, "Read ofs: 0x%x - val: 0x%x\n", ofs, *val);
}


/* Manual Interrupt Compare macros */
#define INTERRUPT_CCRx(x)							\
	if(m_egr_reg & NUCLEO_TIMx_EGR_CC##x##G){		\
		m_sr_reg |= NUCLEO_TIMx_SR_CC##x##IF;		\
		if(m_dier_reg & NUCLEO_TIMx_DIER_CC##x##IE)	\
			ev_irq_update.notify();					\
		m_egr_reg &= ~(NUCLEO_TIMx_EGR_CC##x##G);	\
	}

/* Update reg with timer-size */
#define UPDATE_WITH_SIZE(reg)							\
	reg = (timer_size == 16 ? (uint16_t)value : value);

void NucleoTimer::bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr)
{
	uint32_t value =*((uint32_t*) data + 0);
	MLOG_F(SIM, DBG, "write to ofs: 0x%x - val: 0x%x\n", ofs, value);
	switch(ofs){
	case NUCLEO_TIMx_CR1:
		m_cr1_reg = value & NUCLEO_TIMx_CR1_MASK;
	    m_timer_on = m_cr1_reg & NUCLEO_TIMx_CR1_CEN;
		if((m_cr1_reg & NUCLEO_TIMx_CR1_CMS) != 0){			
			MLOG_F(SIM, DBG, "Center-aligned mode not implemented.\n");
			m_cr1_reg &= ~(NUCLEO_TIMx_CR1_CMS); 
		}
		if(m_timer_on){ 
			ev_wake.notify();
		}
		break;
	case NUCLEO_TIMx_CR2:
		m_cr2_reg = value & NUCLEO_TIMx_CR2_MASK;
		break;
	case NUCLEO_TIMx_SMCR:
		m_smcr_reg = value & NUCLEO_TIMx_SMCR_MASK;
		break;
	case NUCLEO_TIMx_DIER: 
		m_dier_reg = value & NUCLEO_TIMx_DIER_MASK;
		break; 
	case NUCLEO_TIMx_SR:
		m_sr_reg &= value; // Only reset available
		if(!(m_sr_reg & m_dier_reg)  ) {
			irq_status = false;
			ev_irq_update.notify(); 
		}
		break;
	case NUCLEO_TIMx_EGR:
		m_egr_reg = value & NUCLEO_TIMx_EGR_MASK;

		/* UG Bit */
		if(m_egr_reg && NUCLEO_TIMx_EGR_UG){
			update_event();
			m_cnt_reg = m_cr1_reg & NUCLEO_TIMx_CR1_DIR ? m_shadow_arr : 0;
			m_sr_reg |= NUCLEO_TIMx_SR_UIF;
			if(m_dier_reg & NUCLEO_TIMx_DIER_UIE)
				ev_irq_update.notify(); 
	
			ev_stop_wait.notify(); // Use to reset prescaler waiting
			m_egr_reg &= ~(NUCLEO_TIMx_EGR_UG); // clear EGR bit
		}

		/* Compare interrupts */
		INTERRUPT_CCRx(1);
		INTERRUPT_CCRx(2);
		INTERRUPT_CCRx(3);
		INTERRUPT_CCRx(4);

		break;
	case NUCLEO_TIMx_CCMR1:
		m_ccmr1_reg = value & NUCLEO_TIMx_CCMR1_MASK;
		break;
	case NUCLEO_TIMx_CCMR2:
		m_ccmr2_reg = value & NUCLEO_TIMx_CCMR2_MASK;
		break;
	case NUCLEO_TIMx_CCER:
		m_ccer_reg = value & NUCLEO_TIMx_CCER_MASK;
		break;
	case NUCLEO_TIMx_CNT:
		UPDATE_WITH_SIZE(m_cnt_reg);
		break; 
	case NUCLEO_TIMx_PSC:
		m_psc_reg = value;
		break;
	case NUCLEO_TIMx_ARR:
		// Arr size check
		UPDATE_WITH_SIZE(m_arr_reg);
		if(m_arr_reg == 0){ // counter blocked
			m_timer_on = false; 
		}
		if(!(m_cr1_reg & NUCLEO_TIMx_CR1_ARPE)) // shadow_arr and arr must be the same
			m_shadow_arr = m_arr_reg; 
		break;
	case NUCLEO_TIMx_CCR1:
		UPDATE_WITH_SIZE(m_ccr1_reg);
		if(!(m_ccmr1_reg & NUCLEO_TIMx_CCMR1_OC1PE))
			m_shadow_ccr1 = m_ccr1_reg; 
		break; 
	case NUCLEO_TIMx_CCR2:
		UPDATE_WITH_SIZE(m_ccr2_reg);
		if(!(m_ccmr1_reg & NUCLEO_TIMx_CCMR1_OC2PE))
			m_shadow_ccr2 = m_ccr2_reg; 
		break;
	case NUCLEO_TIMx_CCR3:
		UPDATE_WITH_SIZE(m_ccr3_reg);
		if(!(m_ccmr2_reg & NUCLEO_TIMx_CCMR2_OC3PE))
			m_shadow_ccr3 = m_ccr3_reg; 
		break;
	case NUCLEO_TIMx_CCR4:
		UPDATE_WITH_SIZE(m_ccr4_reg);
		if(!(m_ccmr2_reg & NUCLEO_TIMx_CCMR2_OC4PE))
			m_shadow_ccr4 = m_ccr4_reg; 
		break;
	case NUCLEO_TIMx_DCR:
		m_dcr_reg = value;
		break;
	case NUCLEO_TIMx_OR:
		m_or_reg = value;
		break;
	default:
		MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
			   (unsigned int) ofs);
		bErr = true;
		return;
	}
}

void NucleoTimer::irq_update_thread()
{
	while(1) {
		wait(ev_irq_update);
		irq.sc_p = irq_status; 
	}
}


/* Interrupt macro */ 
#define UEV_ARR()								\
	if(!(m_cr1_reg & NUCLEO_TIMx_CR1_UDIS)){	\
		update_event();							\
		m_sr_reg |= NUCLEO_TIMx_SR_UIF;			\
		if(m_dier_reg & NUCLEO_TIMx_DIER_UIE)	\
			irq_needed = true;					\
	}

#define UEV_COMPARE_CCRx(x)							\
	if(m_cnt_reg == m_shadow_ccr##x){				\
		m_sr_reg |= NUCLEO_TIMx_SR_CC##x##IF;		\
		if(m_dier_reg & NUCLEO_TIMx_DIER_CC##x##IE)	\
			irq_needed = true;						\
	}


void NucleoTimer::counter_thread()
{
	while(1) {
		if(m_timer_on) {
			bool irq_needed = false; // bool to notify ev_irq_update only once each tick
			uint32_t wait_time = m_ns_period * (m_shadow_psc+1);

			/* Counter Behaviour */
			if(m_cr1_reg & NUCLEO_TIMx_CR1_DIR){
				m_cnt_reg--;

				if(m_cnt_reg == 0) { // Counter done
					UEV_ARR() 
						m_cnt_reg = m_shadow_arr;
				}
			} else {
				m_cnt_reg++; 
				if(m_cnt_reg == m_shadow_arr) { // Counter done
				    UEV_ARR()
						m_cnt_reg = 0;
				}
			}

			/* Compare Behaviour */
			UEV_COMPARE_CCRx(1);
			UEV_COMPARE_CCRx(2);
			UEV_COMPARE_CCRx(3);
			UEV_COMPARE_CCRx(4);
			
			if(irq_needed){
				irq_status = true; 
				ev_irq_update.notify();
				
			}

			wait(wait_time, SC_NS, ev_stop_wait); //ev_update_event used to stop waiting if needed
			
		} else {
			wait(ev_wake);
		}
	}
}


void NucleoTimer::update_event()
{
	/* Update shadow registers */
	m_shadow_psc = m_psc_reg;
	if(m_cr1_reg & NUCLEO_TIMx_CR1_ARPE)
		m_shadow_arr = m_arr_reg;
	if(m_ccmr1_reg & NUCLEO_TIMx_CCMR1_OC1PE)
		m_shadow_ccr1 = m_ccr1_reg;
	if(m_ccmr1_reg & NUCLEO_TIMx_CCMR1_OC2PE)
		m_shadow_ccr2 = m_ccr2_reg;
	if(m_ccmr2_reg & NUCLEO_TIMx_CCMR2_OC3PE)
		m_shadow_ccr3 = m_ccr3_reg;
	if(m_ccmr2_reg & NUCLEO_TIMx_CCMR2_OC4PE)
		m_shadow_ccr4 = m_ccr4_reg;
}
 
