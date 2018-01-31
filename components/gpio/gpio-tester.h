#ifndef _gpioTester_H
#define _gpioTester_H

#include <rabbits/component/slave.h>
#include <rabbits/component/port/out.h>
#include <rabbits/component/port/in.h>

class gpioTester : public Component
{
public:
SC_HAS_PROCESS (gpioTester);
gpioTester(sc_core::sc_module_name name, const Parameters &params, ConfigManager &c);
virtual ~gpioTester();

private:

void send_thread();

public:
OutPort<bool>  p_in;

};

#endif

