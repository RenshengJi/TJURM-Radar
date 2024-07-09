#ifndef RM2024_SERIAL_SERIAL__H_
#define RM2024_SERIAL_SERIAL__H_

#include "data_manager/param.h"
#include "data_manager/base.h"
#include <openrm/cudatools.h>
#include <unistd.h>



void serial_port_init();
void serial_port_recv();

#endif