// Phidgets
#include <phidgets/util.h>

// STL
#include <string>

namespace phidgets
{
bool handleError()
{
	// TODO: Implement
	PhidgetReturnCode error_code;
	char const* error_str;
	char error_detail[100];
	size_t error_detail_len = 100;
	Phidget_getLastError(&error_code, &error_str, error_detail, &error_detail_len);
	switch (error_code) {
		case EPHIDGET_OK:
			return false;
		case EPHIDGET_NOENT:
			break;
		case EPHIDGET_TIMEOUT:
			break;
		case EPHIDGET_INTERRUPTED:
			break;
		case EPHIDGET_ACCESS:
			break;
		case EPHIDGET_BUSY:
			break;
		case EPHIDGET_INVALID:
			break;
		case EPHIDGET_NOSPC:
			break;
		case EPHIDGET_UNSUPPORTED:
			break;
		case EPHIDGET_INVALIDARG:
			break;
		case EPHIDGET_UNEXPECTED:
			break;
		case EPHIDGET_DUPLICATE:
			break;
		case EPHIDGET_WRONGDEVICE:
			break;
		case EPHIDGET_UNKNOWNVAL:
			break;
		case EPHIDGET_NOTATTACHED:
			break;
		case EPHIDGET_NOTCONFIGURED:
			break;
		case EPHIDGET_UNKNOWNVALHIGH:
			break;
		case EPHIDGET_UNKNOWNVALLOW:
			break;
		default:
			break;
	}
	PhidgetLog_log(PHIDGET_LOG_CRITICAL, error_str);
	// ROS_FATAL("Error (%d): %s", error_code, error_str);
	return true;
}

void attachCallback(PhidgetHandle ch, void* ctx)
{
	PhidgetLog_loge(NULL, 0, NULL, reinterpret_cast<std::string*>(ctx)->c_str(),
	                PHIDGET_LOG_INFO, "Attached");
}

void detachCallback(PhidgetHandle ch, void* ctx)
{
	PhidgetLog_loge(NULL, 0, NULL, reinterpret_cast<std::string*>(ctx)->c_str(),
	                PHIDGET_LOG_INFO, "Detached");
}

void errorCallback(PhidgetHandle ch, void* ctx, Phidget_ErrorEventCode code,
                   char const* description)
{
	PhidgetLog_loge(NULL, 0, NULL, reinterpret_cast<std::string*>(ctx)->c_str(),
	                PHIDGET_LOG_INFO, "Error");
}
}  // namespace phidgets