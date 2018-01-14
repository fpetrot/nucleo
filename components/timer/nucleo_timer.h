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

#ifndef _NUCLEO_TIMER_H
#define _NUCLEO_TIMER_H

#include "rabbits/component/slave.h"
#include <rabbits/component/port/out.h>


#define TIMx_CR1		0x00
#define TIMx_CR2		0x04
#define TIMx_SMCR		0x08
#define TIMx_DIER		0x0C
#define TIMx_SR			0x10
#define TIMx_EGR		0x14
#define TIMx_CCMR1		0x18
#define TIMx_CCMR2		0x1C
#define TIMx_CCER		0x20
#define TIMx_CNT		0x24
#define TIMx_PSC		0x28
#define TIMx_ARR		0x2C
#define TIMx_CCR1		0x34
#define TIMx_CCR2		0x38
#define TIMx_CCR3		0x3C
#define TIMx_CCR4		0x40
#define TIMx_DCR		0x48
#define TIMx_OR			0x50

#define TIMx_CR1_CEN	0x0001
#define TIMx_CR1_UDIS	0x0002
#define TIMx_CR1_URS	0x0004
#define TIMx_CR1_OPM	0x0008
#define TIMx_CR1_DIR	0x0010
#define TIMx_CR1_ARPE	0x0080 

#define TIMx_DIER_UIE	0x0001
#define TIMx_DIER_CC1IE 0x0002
#define TIMx_DIER_CC2IE 0x0004
#define TIMx_DIER_CC3IE 0x0008
#define TIMx_DIER_CC4IE 0x0010

#define TIMx_SR_UIF		0x0001
#define TIMx_SR_CC1IF	0x0002
#define TIMx_SR_CC2IF	0x0004
#define TIMx_SR_CC3IF	0x0008
#define TIMx_SR_CC4IF	0x0010

#define TIMx_EGR_UG		0x0001
#define TIMx_EGR_CC1G	0x0002
#define TIMx_EGR_CC2G	0x0004
#define TIMx_EGR_CC3G	0x0008
#define TIMx_EGR_CC4G	0x0010

#define TIMx_CCMR1_CC1S 0x0003
#define TIMx_CCMR1_OC1FE 0x0004
#define TIMx_CCMR1_OC1PE 0x0008
#define TIMx_CCMR1_OC1M 0x0070
#define TIMx_CCMR1_OC1CE 0x0080
#define TIMx_CCMR1_CC2S 0x0300
#define TIMx_CCMR1_OC2FE 0x0400
#define TIMx_CCMR1_OC2PE 0x0800
#define TIMx_CCMR1_OC2M 0x7000
#define TIMx_CCMR1_OC2CE 0x8000

#define TIMx_CCMR2_CC3S 0x0003
#define TIMx_CCMR2_OC3FE 0x0004
#define TIMx_CCMR2_OC3PE 0x0008
#define TIMx_CCMR2_OC3M 0x0070
#define TIMx_CCMR2_OC3CE 0x0080
#define TIMx_CCMR2_CC4S 0x0300
#define TIMx_CCMR2_OC4FE 0x0400
#define TIMx_CCMR2_OC4PE 0x0800
#define TIMx_CCMR2_OC4M 0x7000
#define TIMx_CCMR2_OC4CE 0x8000


class NucleoTimer : public Slave<>
{
 public: 
  SC_HAS_PROCESS(NucleoTimer);
  NucleoTimer(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c); 
  virtual ~NucleoTimer();

 private:
  void bus_cb_read(uint64_t addr, uint8_t *data, unsigned int len, bool &bErr);
  void bus_cb_write(uint64_t addr, uint8_t *data, unsigned int len, bool &bErr);

  void irq_update_thread();
  void counter_thread();
  void update_event();

  /* Registers with reset value */
  uint32_t m_cr1_reg = 0x0;
  uint32_t m_cr2_reg = 0x0;
  uint32_t m_smcr_reg = 0x0;
  uint32_t m_dier_reg = 0x0;
  uint32_t m_sr_reg = 0x0;
  uint32_t m_egr_reg = 0x0;
  uint32_t m_ccmr1_reg = 0x0;
  uint32_t m_ccmr2_reg = 0x0;
  uint32_t m_ccer_reg = 0x0;
  uint32_t m_cnt_reg = 0x0;
  uint16_t m_psc_reg = 0x0;
  uint32_t m_arr_reg = 0x0;
  uint32_t m_ccr1_reg = 0x0;
  uint32_t m_ccr2_reg = 0x0;
  uint32_t m_ccr3_reg = 0x0;
  uint32_t m_ccr4_reg = 0x0;
  uint32_t m_dcr_reg = 0x0;
  uint32_t m_or_reg = 0x0;

  sc_core::sc_event ev_irq_update;
  sc_core::sc_event ev_stop_wait; 
  sc_core::sc_event ev_wake;  

  uint8_t arrbits; 
  uint32_t m_ns_period = 10;

  /* Shadow registers
   * NOTE : Timer behaviour is completely based on those registers. There are updated either during an update event or when their main registers is modified and their preload bit is disable ( ARPE or OCxPE )
   */
  uint32_t m_shadow_arr;
  uint16_t m_shadow_psc;
  uint32_t m_shadow_ccr1;
  uint32_t m_shadow_ccr2;
  uint32_t m_shadow_ccr3;
  uint32_t m_shadow_ccr4;

  bool m_timer_on = false;

 
 public:
  //Port
  OutPort<bool> irq; 
}; 

#endif
