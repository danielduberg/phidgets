#ifndef PHIDGETS_ERROR_HANDLING_H
#define PHIDGETS_ERROR_HANDLING_H

// Phidget
extern "C" {
#include <phidget22.h>
}

namespace phidgets
{
bool handleError();

void CCONV attachCallback(PhidgetHandle ch, void *ctx);

void CCONV detachCallback(PhidgetHandle ch, void *ctx);

void CCONV errorCallback(PhidgetHandle ch, void *ctx, Phidget_ErrorEventCode code,
                         char const *description);
}  // namespace phidgets

#endif  // PHIDGETS_ERROR_HANDLING_H