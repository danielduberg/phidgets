// Phidgets
#include <phidgets/util.h>

// ROS
#include <ros/console.h>

// STL
#include <string>

namespace phidgets
{
void handleError(PhidgetReturnCode ret_code, int exit_status, char const* name)
{
	if (EPHIDGET_OK == ret_code) {
		return;
	}

	// TODO: Implement
	PhidgetReturnCode error_code;
	char const* error_str;
	char error_detail[100];
	size_t error_detail_len = 100;
	Phidget_getLastError(&error_code, &error_str, error_detail, &error_detail_len);
	switch (error_code) {
		case EPHIDGET_OK:
			return;
		case EPHIDGET_UNKNOWNVALLOW:
			break;
		case EPHIDGET_NOENT:
			break;
		case EPHIDGET_TIMEOUT:
			break;
		case EPHIDGET_KEEPALIVE:
			break;
		case EPHIDGET_INTERRUPTED:
			break;
		case EPHIDGET_IO:
			break;
		case EPHIDGET_NOMEMORY:
			break;
		case EPHIDGET_ACCESS:
			break;
		case EPHIDGET_FAULT:
			break;
		case EPHIDGET_BUSY:
			break;
		case EPHIDGET_EXIST:
			break;
		case EPHIDGET_NOTDIR:
			break;
		case EPHIDGET_ISDIR:
			break;
		case EPHIDGET_INVALID:
			break;
		case EPHIDGET_NFILE:
			break;
		case EPHIDGET_MFILE:
			break;
		case EPHIDGET_NOSPC:
			break;
		case EPHIDGET_FBIG:
			break;
		case EPHIDGET_ROFS:
			break;
		case EPHIDGET_RO:
			break;
		case EPHIDGET_UNSUPPORTED:
			break;
		case EPHIDGET_INVALIDARG:
			break;
		case EPHIDGET_PERM:
			break;
		case EPHIDGET_NOTEMPTY:
			break;
		case EPHIDGET_UNEXPECTED:
			break;
		case EPHIDGET_DUPLICATE:
			break;
		case EPHIDGET_BADPASSWORD:
			break;
		case EPHIDGET_NETUNAVAIL:
			break;
		case EPHIDGET_CONNREF:
			break;
		case EPHIDGET_CONNRESET:
			break;
		case EPHIDGET_HOSTUNREACH:
			break;
		case EPHIDGET_NODEV:
			break;
		case EPHIDGET_WRONGDEVICE:
			break;
		case EPHIDGET_PIPE:
			break;
		case EPHIDGET_RESOLV:
			break;
		case EPHIDGET_UNKNOWNVAL:
			break;
		case EPHIDGET_NOTATTACHED:
			break;
		case EPHIDGET_INVALIDPACKET:
			break;
		case EPHIDGET_2BIG:
			break;
		case EPHIDGET_BADVERSION:
			break;
		case EPHIDGET_CLOSED:
			break;
		case EPHIDGET_NOTCONFIGURED:
			break;
		case EPHIDGET_EOF:
			break;
		case EPHIDGET_FAILSAFE:
			break;
		case EPHIDGET_UNKNOWNVALHIGH:
			break;
		case EPHIDGET_AGAIN:
			break;
	}
	PhidgetLog_log(PHIDGET_LOG_CRITICAL, error_str);
	ROS_FATAL("Phidget %s error (%d): %s", name, error_code, error_str);
	exit(exit_status);
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
	                PHIDGET_LOG_ERROR, "Error");
}
}  // namespace phidgets