/*
 *  This file is part of Rabbits
 *  Copyright (C) 2017 Frédéric Pétrot and ...
 *
 *  Based on the Raspberry PI Braodcom GPIO implementation
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

#ifndef _NUCLEO_GPIO_H_
#define _NUCLEO_GPIO_H_

#include "rabbits/component/slave.h"
#include <rabbits/component/port/out.h>
#include <rabbits/component/port/inout.h>
#include <rabbits/component/port/vector.h>

#define GPIOx_MODER      0x00
#define GPIOx_OTYPER     0x04
#define GPIOx_OSPEEDER   0x08
#define GPIOx_PUPDR      0x0C
#define GPIOx_IDR        0x10
#define GPIOx_ODR        0x14
#define GPIOx_BSRR       0x18
#define GPIOx_LCKR       0x1C
#define GPIOx_AFRL       0x20
#define GPIOx_AFRH       0x24

const int NUCLEO_GPIO_COUNT = 50;

class GPFSELn {
private:
    uint8_t m_fsel[NUCLEO_GPIO_COUNT];

public:
    void set(uint8_t reg, uint32_t value) {
        for(int i=0; i<10; i++) {
            uint32_t index = 10*reg + i;
            if(index < sizeof(m_fsel)) {
                int fsel = (value >> (3 * i)) & 0x3;
                m_fsel[index] = fsel;
            }
        }
    }

    uint32_t get(uint8_t reg) {
        uint32_t value = 0;
        for(int i=0; i<10; i++) {
            uint32_t index = 10*reg + i;
            if(index < sizeof(m_fsel)) {
                value |= m_fsel[index] << (3 * i);
            }
        }
        return value;
    }

    uint8_t get_function(int index) {
        if(index >= 0 && index < 54) {
            return m_fsel[index];
        }
        return false;
    }

    bool is_in(int index) {
        return get_function(index) == 0;
    }

    bool is_out(int index) {
        return get_function(index) == 1;
    }
};

class nucleo_gpio: public Slave<>
{
public:
    SC_HAS_PROCESS (nucleo_gpio);
    nucleo_gpio(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
    virtual ~nucleo_gpio();

    VectorPort< InOutPort<bool> > p_gpios;

    uint32_t m_lev0, m_lev1;

    uint32_t gpiox_moder_reg;
    uint32_t gpiox_otyper_reg;
    uint32_t gpiox_speedr_reg;
    uint32_t gpiox_pupdr_reg;
    uint32_t gpiox_idr_reg;
    uint32_t gpiox_odr_reg;
    uint32_t gpiox_bsrr_reg;
    uint32_t gpiox_lckr_reg;
    uint32_t gpiox_afrl_reg;
    uint32_t gpiox_afrh_reg;

    uint32_t gpiox_lckr_reg_prev;
    uint8_t gpiox_lckk;
    uint8_t gpiox_lck_state;
    uint8_t gpiox_lock_config;

    sc_core::sc_event_or_list m_ev_gpios;

    void reset(const uint32_t& moder_rv, const uint32_t& ospeedr_rv, const uint32_t& pupdr_rv) {
        gpiox_moder_reg = moder_rv;
	    gpiox_otyper_reg = 0x00000000;
        gpiox_speedr_reg = ospeedr_rv;
        gpiox_pupdr_reg = pupdr_rv;
        gpiox_idr_reg = 0x00000000;
        gpiox_odr_reg = 0x00000000;
	    gpiox_bsrr_reg = 0x00000000;
        gpiox_lckr_reg = 0x00000000;
        gpiox_afrl_reg = 0x00000000;
        gpiox_afrh_reg = 0x00000000;
        
        gpiox_lckr_reg_prev = 0x00000000;
        gpiox_lckk = 0;
        gpiox_lck_state = 0;
    }


    // void gpset(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev);
    // void gpclr(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev);

    void set_weak_bits(uint16_t val, uint32_t &reg);

private:
    uint32_t moder_rv;
    uint32_t ospeedr_rv;
    uint32_t pupdr_rv;

    void bus_cb_read_32(uint64_t addr, uint32_t *data, bool &bErr);
    void bus_cb_write_32(uint64_t addr, uint32_t *data, bool &bErr);
    void bus_cb_read_16(uint64_t ofs, uint16_t *data, bool &bErr);
    void bus_cb_write_16(uint64_t ofs, uint16_t *data, bool &bErr);


    void gpio_thread();

    void end_of_elaboration();

    GPFSELn m_gpfsel;
};

#endif
