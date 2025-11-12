#ifndef __conntrack_h__
#define __conntrack_h__

#include <stdbool.h>


extern bool isConnected;
void setConnected(bool connected);
void connection_tracking_init();
#endif