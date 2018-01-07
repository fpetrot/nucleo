/*
 *  This file want to be a part of Rabbits
 *  Copyright (C) 2017 Joris Collomb
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

#ifndef _COMPONENT_CONNECTION_STRATEGY_USART_DEV_H
#define _COMPONENT_CONNECTION_STRATEGY_USART_DEV_H

#include <systemc>

#include <rabbits/component/connection_strategy.h>
#include "usart_ch_dev.h"

class UsartDeviceCS : public ConnectionStrategy<UsartDeviceCS> {
public:
    using typename ConnectionStrategyBase::BindingResult;
    using ConnectionStrategyBase::ConnectionInfo;

private:
    sc_core::sc_port<UsartDeviceSystemCInterface> &m_tx;
    sc_core::sc_port<UsartDeviceSystemCInterface> &m_rx;

    UsartDeviceChannel chan;

public:
    UsartDeviceCS(sc_core::sc_port<UsartDeviceSystemCInterface> & tx,
                 sc_core::sc_port<UsartDeviceSystemCInterface> & rx)
        : m_tx(tx), m_rx(rx) {}

    virtual ~UsartDeviceCS() {}

    BindingResult bind_peer(UsartDeviceCS &cs, ConnectionInfo &info, PlatformDescription &d)
    {
        m_tx(chan);
        cs.m_rx(chan);

        m_rx(cs.chan);
        cs.m_tx(cs.chan);

        return BindingResult::BINDING_OK;
    }

    BindingResult bind_hierarchical(UsartDeviceCS &parent_cs, ConnectionInfo &info)
    {
        m_tx(parent_cs.m_tx);
        m_rx(parent_cs.m_rx);

        return BindingResult::BINDING_OK;
    }

    virtual const char * get_typeid() const { return "Usart-dev"; }
};

#endif
