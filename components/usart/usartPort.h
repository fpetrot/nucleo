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

#ifndef _RABBITS_COMPONENT_PORT_USART_PORT_H
#define _RABBITS_COMPONENT_PORT_USART_PORT_H

#include <systemc>
#include <cstdlib>

#include <rabbits/component/port.h>
#include <rabbits/logger.h>
#include "usart_ch_dev.h"
#include "usart_cs_dev.h"

class UsartPort : public Port {
public:
    enum eMode { CHAR_DEV, SIGNALS };

private:
    eMode m_mode;
    UsartDeviceCS m_usartdev_cs;

    sc_core::sc_event m_data_rcv_ev;

public:
    sc_core::sc_port<UsartDeviceSystemCInterface> tx, rx;

    UsartPort(const std::string & name)
        : Port(name), m_usartdev_cs(tx, rx)
        , tx((name + "-tx").c_str())
        , rx((name + "-rx").c_str())
    {
        add_connection_strategy(m_usartdev_cs);
        declare_parent(tx.get_parent_object());
        add_attr_to_parent("char-port", name);

        //trying to print the connected
        HasPortIface *parent=dynamic_cast<HasPortIface*>(tx.get_parent_object());

        if(parent!=NULL){
          printf("parent find %p\n",parent);
          if(parent->port_begin()==parent->port_end())printf("but no port connected\n");
          for (HasPortIface::port_iterator c = parent->port_begin() ; c != parent->port_end() ; c++){
            printf("Construction UsartPort: %s / %s\n",c->first.c_str(), c->second->name().c_str());
          }
        }else printf("no parent\n");
    }


    virtual ~UsartPort() {}

    virtual void selected_strategy(ConnectionStrategyBase &cs) {
        if (&cs == &m_usartdev_cs) {
            m_mode = CHAR_DEV;
        } else {
            MLOG(APP, ERR) << "Selected strategy is invalid\n";
            std::abort();
        }
    }

    const sc_core::sc_event & default_event() const { return rx->default_event(); }

    void recv(std::vector<data8bit> &data) { rx->recv(data); }
    void send(std::vector<data8bit> &data) { tx->send(data); }

    void recv(std::vector<data9bit> &data) { rx->recv(data); }
    void send(std::vector<data9bit> &data) { tx->send(data); }

    bool data_pending() { return !rx->empty(); }

    const char * get_typeid() const { return "usart"; }
};
#endif
