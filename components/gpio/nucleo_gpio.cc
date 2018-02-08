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
#include <cstdio>
#include <cstdlib>
#include "nucleo_gpio.h"
#include "rabbits/logger.h"

using namespace sc_core;
nucleo_gpio::nucleo_gpio(sc_core::sc_module_name name,
                               const Parameters &params, ConfigManager &c)
    : Slave(name, params, c)
    , p_gpios("gpios", NUCLEO_GPIO_COUNT)
{
    moder_rv   = params["moder"].as<uint32_t>();
    ospeedr_rv = params["ospeedr"].as<uint32_t>();
    pupdr_rv   = params["pupdr"].as<uint32_t>();

    nucleo_gpio::reset(moder_rv, ospeedr_rv, pupdr_rv);

    for(auto &p : p_gpios) {
        p.set_autoconnect_to(0);
    }
    SC_THREAD(gpio_thread);
}

nucleo_gpio::~nucleo_gpio()
{
}

void nucleo_gpio::end_of_elaboration()
{
    for (auto &p : p_gpios) {
        m_ev_gpios |= p.sc_p.default_event();
    }
}

void nucleo_gpio::set_weak_bits(uint16_t val, uint32_t &reg) {
    reg &= 0xFFFF0000;
    reg |= (uint32_t) val;
}

void nucleo_gpio::bus_cb_write(uint64_t ofs, uint8_t *data, unsigned int len, bool &bErr)
{
    uint32_t value = *((uint32_t*) data + 0);
	MLOG_F(SIM, DBG, "Write ofs: 0x%x - val: 0x%x - len: %d\n", ofs, value, len);
    switch (ofs) {
    case GPIOx_MODER:
        // bit de config à 00 input, 01 general output, 10 AF, 11 analog
        for (int i = 0; i < 16; i++) {
            if (((gpiox_lckr_reg >> i) & 1U) == 0) {
                uint16_t x = (value >> i) & 1U;
                gpiox_moder_reg ^= (-(unsigned long)x ^ gpiox_moder_reg) & (1UL << i);
                x = (value >> (i + 1)) & 1U;
                gpiox_moder_reg ^= (-(unsigned long)x ^ gpiox_moder_reg) & (1UL << (i + 1));
            } else {
                bErr = true;
                return;
            }
        }
        break;
    case GPIOx_OTYPER:
        set_weak_bits(value, gpiox_otyper_reg);
        break;
    case GPIOx_OSPEEDER:
        gpiox_speedr_reg = value;
        break;
    case GPIOx_PUPDR:
        gpiox_pupdr_reg = value;
        break;
    case GPIOx_IDR:
        /* doc : The data input through the I/O are stored
	     * into the input data register (GPIOx_IDR), a read-only register
	     */
	    // METTRE UN IF (I/O) ?? PAS SENSE ETRE ACCESSIBLE EN ECRITURE
        // set_weak_bits((uint16_t)*data, nucleo_gpio::gpiox_idr_reg);
        break;
    case GPIOx_ODR:
        /* doc : GPIx_ODR stores the data to be output,
	     * it is read/write accessible
	     */
        for (int i = 0; i < 16; i++) {
            if (((gpiox_moder_reg) >> (i * 2) & 0x3) == 0b01) {
                uint16_t x = (value >> i) & 1U;
                gpiox_odr_reg ^= (-(unsigned long)x ^ gpiox_odr_reg) & (1UL << i);
            }
        }

		update_pin_output();
        break;
    case GPIOx_BSRR:
        gpiox_bsrr_reg = value;
        /* Set the BR part (bit 31 downto 16)
         */
        for(int i=31; i>=16; i--) {
            // check bit state
            if(((gpiox_bsrr_reg >> i) & 1U) == 1) {
                // reset (clear) the corresponding bit
                gpiox_odr_reg &= ~(1UL << (i-16));
            }
        }
        /* Set the BS part (bit 15 downto 0)
         */
        for(int i= 15; i>=0; i--) {
            if(((gpiox_bsrr_reg >> i) & 1U) == 1) {
                // Set the corresponding bit ODR
                gpiox_odr_reg |= 1UL << i;
            }
        }
		update_pin_output();

        break;
    case GPIOx_LCKR:
        gpiox_lckr_reg = *data;
        gpiox_lckr_reg &= 0x0000FFFF;
        gpiox_lckk = gpiox_lckr_reg & 0x00010000;

        if (gpiox_lckk == 0 && gpiox_lck_state == 0) {
            // nothing : stay in the initial state
        } else if (gpiox_lckk == 1 && gpiox_lck_state == 0) {
            gpiox_lckr_reg_prev = gpiox_lckr_reg;
            gpiox_lck_state = 1;
        } else if (gpiox_lckk == 0 && gpiox_lck_state == 1) {
            gpiox_lck_state = 2;
        } else if (gpiox_lckk == 1 && gpiox_lck_state == 2) {
            gpiox_lck_state = 3;
            // sequence ok : locking the config
        } else {
            gpiox_lck_state = 0;
            gpiox_lckr_reg = 0x00000000;
        }
        break;

    case GPIOx_AFRL:
    break;

    case GPIOx_AFRH:
    break;

    default :
        MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
               (unsigned int) ofs);
        bErr = true;
        return;
    }

    bErr = false;

}

void nucleo_gpio::bus_cb_read(uint64_t ofs, uint8_t *data, unsigned int len,  bool &bErr)
{
    uint32_t *value = (uint32_t *) data;

    switch (ofs) {
    case GPIOx_MODER:
        // bit de config à 01 pour mode output (lecture)
        *value = gpiox_moder_reg;
        break;
    case GPIOx_OTYPER:
        *value = gpiox_otyper_reg;
        break;
    case GPIOx_OSPEEDER:
        *value = gpiox_speedr_reg;
        break;
    case GPIOx_PUPDR:
        *value = gpiox_pupdr_reg;
        break;

    case GPIOx_IDR:
        //doc : The data input through the I/O are stored into the input data register (GPIOx_IDR), a read-only register
        *value = gpiox_idr_reg;
        break;
    case GPIOx_ODR:
        //doc : GPIx_ODR stores the data to be output, it is read/write accessible
		*value = gpiox_odr_reg;
        break;
    case GPIOx_BSRR:
		*value = 0x0; // read always returns 0x0
        break;
    case GPIOx_LCKR:
    break;
    case GPIOx_AFRL:
    break;
    case GPIOx_AFRH:
    break;
    default:
        MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
               (unsigned int) ofs);
        bErr = true;
        return;
    }
    bErr = false;

	MLOG_F(SIM, DBG, "Read ofs: 0x%x - val: 0x%x - len: %d\n", ofs, *value, len);
}

void nucleo_gpio::update_pin_output(){
	int i = 0;
	for(auto &p : p_gpios) {
		p.sc_p = (gpiox_odr_reg >> i) & 1 ;
		i++;
	}
}



void nucleo_gpio::gpio_thread()
{
    while(1) {
        wait(m_ev_gpios);
        int i = 0;
        for(auto &p: p_gpios) {
            if(p.sc_p.read()) {
                gpiox_idr_reg |= 1UL << i;
            } else {
                gpiox_idr_reg &= ~(1UL << i);
            }
            i++;
        }


    }
}
