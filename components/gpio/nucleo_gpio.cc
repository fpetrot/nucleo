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

// void nucleo_gpio::gpset(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev) {
//     uint32_t changes = val & ~*lev;
//     uint32_t cur = 1;
//
//     for (int i = 0; i < count; i++) {
//         if ((changes & cur) && (m_gpfsel.is_out(start + i))) {
//             p_gpios[start + i].sc_p = !!(val & cur);
//         }
//         cur <<= 1;
//     }
//
//     *lev |= val;
// }
//
// void nucleo_gpio::gpclr(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev) {
//     uint32_t changes = val & *lev;
//     uint32_t cur = 1;
//
//     for (int i = 0; i < count; i++) {
//         if ((changes & cur) && (m_gpfsel.is_out(start + i))) {
//             p_gpios[start + i].sc_p = !!(val & cur);
//         }
//         cur <<= 1;
//     }
//
//     *lev &= ~val;
// }

void nucleo_gpio::set_weak_bits(uint16_t val, uint32_t &reg) {
    reg &= 0xFFFF0000;
    reg |= (uint32_t) val;
}

void nucleo_gpio::bus_cb_write_32(uint64_t ofs, uint32_t *data, bool &bErr)
{
    switch (ofs) {
    case GPIOx_MODER:
        // bit de config à 00 pour mode input (écriture)
	    set_weak_bits(0, nucleo_gpio::gpiox_moder_reg);
        // set_weak_bits((uint16_t)*data, nucleo_gpio::gpiox_idr_reg);
        break; 
    case GPIOx_OTYPER: 
        break;
    case GPIOx_OSPEEDER:
        // pas de sens en simulation
        break;
    case GPIOx_PUPDR:
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
	    if (nucleo_gpio::gpiox_moder_reg == 0x00000000) {
            set_weak_bits((uint16_t)*data, nucleo_gpio::gpiox_odr_reg);
	    }	
        break;
#if 0
    case GPIOx_BSRR:
    case GPIOx_LCKR:
    case GPIOx_AFRL:
    case GPIOx_AFRH:
#endif
    default:
        MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X, data=0x%X-%X!\n", name(),
               __FUNCTION__, (unsigned int) ofs,
               (unsigned int) *((uint32_t *) data + 0),
               (unsigned int) *((uint32_t *) data + 1));
        bErr = true;
        return;
    }

    bErr = false;
}

void nucleo_gpio::bus_cb_read_32(uint64_t ofs, uint32_t *data, bool &bErr)
{
    switch (ofs) {
    case GPIOx_MODER:
        // bit de config à 01 pour mode output (lecture)
        set_weak_bits(0b0101010101010101, nucleo_gpio::gpiox_moder_reg);
        break;
    case GPIOx_OTYPER:
        break;
    case GPIOx_OSPEEDER:
        // pas de sens en simulation
        break;
    case GPIOx_PUPDR:
        break;
    case GPIOx_IDR:
        //doc : The data input through the I/O are stored into the input data register (GPIOx_IDR), a read-only register
        *data = (uint32_t)((uint16_t)(nucleo_gpio::gpiox_idr_reg));
        break;
    case GPIOx_ODR:
        //doc : GPIx_ODR stores the data to be output, it is read/write accessible
        if (nucleo_gpio::gpiox_moder_reg == 0b0101010101010101) {
            *data = (uint32_t)((uint16_t)(nucleo_gpio::gpiox_odr_reg));	
        } else {
            *data = 0x00000000;
        }
        break;
    // case GPIOx_BSRR:
    // case GPIOx_LCKR:
    // case GPIOx_AFRL:
    // case GPIOx_AFRH:
    default:
        MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
               (unsigned int) ofs);
        bErr = true;
        return;
    }
#if 0
    uint16_t val;

    // set 0 to config bits (0 to 15) of MODER register
    val = 0b0101010101010101;
    set_weak_bits(val, gpiox_moder_reg);

    // read data of output register
    *data = (uint32_t)((uint16_t)(*gpiox_odr_reg));
#endif
    bErr = false;
}

void nucleo_gpio::gpio_thread()
{
    for(;;) {
        wait(m_ev_gpios);

        unsigned int i = 0;
        for (auto &p : p_gpios) {
            sc_inout<bool> &sc_p = p.sc_p;

            if (sc_p.event() && m_gpfsel.is_in(i)) {
                uint32_t new_val = sc_p.read();

                if(i < 32) {
                    m_lev0 &= ~(1 << i);
                    m_lev0 |= new_val << i;
                }
                else {
                    m_lev1 &= ~(1 << (i - 32));
                    m_lev1 |= new_val << (i - 32);
                }
            }

            i++;
        }
    }
}
