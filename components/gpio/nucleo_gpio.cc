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
    const uint32_t moder_rv   = params["moder"].as<uint32_t>();
    const uint32_t ospeedr_rv = params["ospeedr"].as<uint32_t>();
    const uint32_t pupdr_rv   = params["pupdr"].as<uint32_t>();

    m_gpfsel.reset(moder_rv, ospeedr_rv, pupdr_rv);

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

void nucleo_gpio::gpset(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev) {
    uint32_t changes = val & ~*lev;
    uint32_t cur = 1;

    for (int i = 0; i < count; i++) {
        if ((changes & cur) && (m_gpfsel.is_out(start + i))) {
            p_gpios[start + i].sc_p = !!(val & cur);
        }
        cur <<= 1;
    }

    *lev |= val;
}

void nucleo_gpio::gpclr(uint32_t val, uint8_t start, uint8_t count, uint32_t *lev) {
    uint32_t changes = val & *lev;
    uint32_t cur = 1;

    for (int i = 0; i < count; i++) {
        if ((changes & cur) && (m_gpfsel.is_out(start + i))) {
            p_gpios[start + i].sc_p = !!(val & cur);
        }
        cur <<= 1;
    }

    *lev &= ~val;
}

void nucleo_gpio::bus_cb_write_32(uint64_t ofs, uint32_t *data, bool &bErr)
{
    switch (ofs) {
#if 0
    case GPFSEL0:
    case GPFSEL1:
    case GPFSEL2:
    case GPFSEL3:
    case GPFSEL4:
    case GPFSEL5: {
        uint8_t reg = ofs / 4;
        m_gpfsel.set(reg, *data);
        break;
    }
    case GPSET0:
        gpset(*data, 0, 32, &m_lev0);
        break;
    case GPSET1:
        gpset(*data, 32, 22, &m_lev1);
        break;
    case GPCLR0:
        gpclr(*data, 0, 32, &m_lev0);
        break;
    case GPCLR1:
        gpclr(*data, 32, 22, &m_lev1);
        break;
    case GPLEV0:
    case GPLEV1:
        // Read Only
        break;
    case GPEDS0:
    case GPEDS1:
    case GPREN0:
    case GPREN1:
    case GPFEN0:
    case GPFEN1:
    case GPHEN0:
    case GPHEN1:
    case GPLEN0:
    case GPLEN1:
    case GPAREN0:
    case GPAREN1:
    case GPAFEN0:
    case GPAFEN1:
    case GPPUD:
    case GPPUDCLK0:
    case GPPUDCLK1:
        MLOG_F(SIM, WRN, "writing to register [0x%02X] is not yet implemented\n", ofs & 0xff);
        break;
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

void nucleo_gpio::bus_cb_read_32(uint64_t ofs, uint32_t *data , bool &bErr)
{
    switch (ofs) {
#if 0
    case GPFSEL0:
    case GPFSEL1:
    case GPFSEL2:
    case GPFSEL3:
    case GPFSEL4:
    case GPFSEL5: {
        uint8_t reg = ofs / 4;
        *data = m_gpfsel.get(reg);
        break;
    }
    case GPSET0:
    case GPSET1:
        // Write Only
        *data = 0;
        break;
    case GPCLR0:
    case GPCLR1:
        // Write Only
        *data = 0;
        break;
    case GPLEV0:
        *data = m_lev0;
        break;
    case GPLEV1:
        *data = m_lev1;
        break;
    case GPEDS0:
    case GPEDS1:
    case GPREN0:
    case GPREN1:
    case GPFEN0:
    case GPFEN1:
    case GPHEN0:
    case GPHEN1:
    case GPLEN0:
    case GPLEN1:
    case GPAREN0:
    case GPAREN1:
    case GPAFEN0:
    case GPAFEN1:
    case GPPUD:
    case GPPUDCLK0:
    case GPPUDCLK1:
        MLOG_F(SIM, WRN, "reading from register [0x%02X] is not yet implemented\n", ofs & 0xff);
        *data = 0;
        break;
#endif
    default:
        MLOG_F(SIM, ERR, "Bad %s::%s ofs=0x%X!\n", name(), __FUNCTION__,
               (unsigned int) ofs);
        bErr = true;
        return;
    }
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

