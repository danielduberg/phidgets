#ifndef PHIDGETS_ERROR_HANDLING_H
#define PHIDGETS_ERROR_HANDLING_H

// Phidget
extern "C" {
#include <phidget22.h>
}

namespace phidgets
{

void openWaitForAttachment(PhidgetHandle handle, int hub_port, uint32_t timeout_ms);

void closeAndDelete(PhidgetHandle *handle);
}  // namespace phidgets

#endif  // PHIDGETS_ERROR_HANDLING_H