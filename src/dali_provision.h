#ifndef __daliprov_h__
#define __daliprov_h__

#include <stdbool.h>

extern void dali_provision_task(void *data);
extern void startProvisioning(int device, bool incremental);

#endif