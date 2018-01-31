#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>

#include <rabbits/logger.h>

#include "gpio-tester.h"

using namespace sc_core;

gpioTester::gpioTester(sc_core::sc_module_name name,
                               const Parameters &params, ConfigManager &c)
    : Component(name, c)
    ,p_in("gpio-pin")
{
    SC_THREAD(send_thread);
}

gpioTester::~gpioTester()
{
}

void gpioTester::send_thread() {
    while(1) {
        p_in.sc_p = false;
        wait(10,SC_SEC);
        p_in.sc_p = true;
        wait(10,SC_SEC);
    }
}
