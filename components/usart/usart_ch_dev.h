/*
 *  This file is part of Rabbits
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

#ifndef _COMPONENT_CHANNEL_USART_DEV_H
#define _COMPONENT_CHANNEL_USART_DEV_H

#include <systemc>

typedef struct data9bit_s{
  uint16_t data : 9;
  // uint16_t stopBit : 2;
}data9bit;

typedef struct data8bit_s{
  uint16_t data : 8;
}data8bit;

class UsartDeviceSystemCInterface : public virtual sc_core::sc_interface {
public:

    virtual void send(std::vector<data9bit> &data) = 0;
    virtual void recv(std::vector<data9bit> &data) = 0;

    virtual void send(std::vector<data8bit> &data) = 0;
    virtual void recv(std::vector<data8bit> &data) = 0;

    virtual bool empty() const = 0;
  };


class UsartDeviceChannel : public UsartDeviceSystemCInterface,
                          public sc_core::sc_prim_channel
{
private:
    std::vector<data9bit> m_buffer9bit;
    std::vector<data8bit> m_buffer8bit;

    sc_core::sc_event m_recv_ev;

    void recv(std::vector<data8bit> &data)
    {
        if (m_buffer8bit.empty()) {
          sc_core::wait(m_recv_ev);
        }

        /* TODO: avoid data copy */
        data.clear();
        data.insert(data.end(), m_buffer8bit.begin(), m_buffer8bit.end());
        m_buffer8bit.clear();
    }

    void recv(std::vector<data9bit> &data)
    {
        if (m_buffer9bit.empty()) {
          sc_core::wait(m_recv_ev);
        }

        /* TODO: avoid data copy */
        data.clear();
        data.insert(data.end(), m_buffer9bit.begin(), m_buffer9bit.end());
        m_buffer9bit.clear();
    }

public:
    void send(std::vector<data8bit> &data)
    {
        m_buffer8bit.insert(m_buffer8bit.end(), data.begin(), data.end());
        request_update();
    }

    void send(std::vector<data9bit> &data)
    {
        m_buffer9bit.insert(m_buffer9bit.end(), data.begin(), data.end());
        request_update();
    }

    void update()
    {
        m_recv_ev.notify(sc_core::SC_ZERO_TIME);
    }

    bool empty() const {
        return (m_buffer8bit.empty() && m_buffer9bit.empty()) ;
    }

    const sc_core::sc_event & default_event() const {
        return m_recv_ev;
    }
};
#endif
