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

#ifndef _COMPONENT_CHANNEL_USART_DEV_H
#define _COMPONENT_CHANNEL_USART_DEV_H

#include <systemc>
//#define LOG_BUF //if you want to print data and buff a envery transfer

typedef struct usart_data_s{
  bool length;
      //0:8 bits
      //1:9 bits
  uint16_t data;
  char stopBit;
      // 00 :1    bit stop
      // 01 :0.5  bit stop
      // 10 :2    bit stop
      // 11 :1,5  bit stop
}usart_data;

class UsartDeviceSystemCInterface : public virtual sc_core::sc_interface {
public:

    virtual void send(usart_data &data) = 0;
    virtual void recv(usart_data &data) = 0;

    virtual bool empty() const = 0;
  };


class UsartDeviceChannel : public UsartDeviceSystemCInterface,
                          public sc_core::sc_prim_channel
{
private:

    struct usart_data_buffer{
      usart_data frame;
      bool validity;
    } m_buffer;

    sc_core::sc_event m_recv_ev;

    void recv(usart_data &data)
    {
        if (m_buffer.validity==false) {
          #ifdef LOG_BUF
          printf("waiting m_recv_ev\n");
          #endif
          sc_core::wait(m_recv_ev);
        }

      #ifdef LOG_BUF
      printf("recv: buffer state:\n");
      printf("8 bits/0x%01x Stop  (0x%02x)\n",m_buffer.frame.stopBit, m_buffer.frame.data);
      #endif

      data = m_buffer.frame;
      m_buffer.validity = false;
    }

public:
    void send(usart_data &data)
    {
      #ifdef LOG_BUF
      printf("send: data to send:\n");
      printf("%d bits/0x%01x Stop  (0x%02x)\n",data.length ? 9:8 , data.stopBit, data.data);
      printf("send: buffer state:\n");
      if(m_buffer.validity)printf("%d bits/0x%01x Stop  (0x%02x)\n",m_buffer.frame.length ? 9:8 , m_buffer.frame.stopBit, m_buffer.frame.data);
      #endif


      m_buffer.frame = data;

      if (m_buffer.frame.length)  //Masking with length, 8 or 9 bits setable
        m_buffer.frame.data&=0b111111111;
      else
        m_buffer.frame.data&=0b11111111;

      m_buffer.validity = true;
      request_update();
    }

    void update()
    {
        m_recv_ev.notify(sc_core::SC_ZERO_TIME);
    }

    bool empty() const {
        return (m_buffer.validity) ;
    }

    const sc_core::sc_event & default_event() const {
        return m_recv_ev;
    }
};
#endif
