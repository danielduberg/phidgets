#ifndef PHIDGETS_ERROR_H
#define PHIDGETS_ERROR_H

// Phidget
extern "C" {
#include <phidget22.h>
}

// STL
#include <exception>
#include <string>

namespace phidgets
{
class PhidgetError : public std::exception
{
 public:
	explicit PhidgetError(std::string const& msg, PhidgetReturnCode code);

	char const* what() const noexcept;

 private:
	std::string msg_;
};
}  // namespace phidgets

#endif  // PHIDGETS_ERROR_H