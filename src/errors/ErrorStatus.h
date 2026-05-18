#ifndef ERROR_STATUS_H
#define ERROR_STATUS_H

#include "ErrorCode.h"

#include <string>

struct ErrorStatus {
    ErrorCode code = ErrorCode::Ok;
    ErrorDomain domain = ErrorDomain::None;
    std::string message;
    bool retryable = false;
    bool disconnect = false;

    static ErrorStatus ok()
    {
        return {};
    }

    static ErrorStatus failure(ErrorCode code_value,
                               ErrorDomain domain_value,
                               const std::string& message_value,
                               bool retryable_value = false,
                               bool disconnect_value = false)
    {
        ErrorStatus status;
        status.code = code_value;
        status.domain = domain_value;
        status.message = message_value;
        status.retryable = retryable_value;
        status.disconnect = disconnect_value;
        return status;
    }
};

inline ErrorStatus mapLegacyError(const std::string& error_code,
                                  const std::string& message)
{
    ErrorStatus status = ErrorStatus::failure(ErrorCode::InternalError,
                                              ErrorDomain::System,
                                              message,
                                              false,
                                              false);

    if (error_code == "UNKNOWN_COMMAND") {
        status.code = ErrorCode::UnknownCommand;
        status.domain = ErrorDomain::Protocol;
    } else if (error_code == "HOMING_FAILED") {
        status.code = ErrorCode::HomingFailed;
        status.domain = ErrorDomain::Device;
    } else if (error_code == "CAPTURE_FAILED" ||
               error_code == "COPY_FAILED" ||
               error_code == "INVALID_BEV_BUFFER") {
        status.code = ErrorCode::CaptureFailed;
        status.domain = ErrorDomain::Device;
    } else if (error_code == "DETECTION_FAILED" ||
               error_code == "REFRESH_DETECT_FAILED" ||
               error_code == "JSON_BUILD_FAILED" ||
               error_code == "THICKNESS_FAILED") {
        status.code = ErrorCode::DetectionFailed;
        status.domain = ErrorDomain::Vision;
    } else if (error_code == "CALIBRATION_FAILED" ||
               error_code == "FOCAL_FAILED" ||
               error_code == "TOTALHIGH_FAILED" ||
               error_code == "TOTALHIGH_READ_FAILED") {
        status.code = ErrorCode::CalibrationFailed;
        status.domain = ErrorDomain::Calibration;
    } else if (error_code == "NO_BEV_FRAME" ||
               error_code == "REFRESH_FRAME_TIMEOUT") {
        status.code = ErrorCode::Timeout;
        status.domain = ErrorDomain::Vision;
        status.retryable = true;
    } else if (error_code == "PROTOCOL_ERROR") {
        status.code = ErrorCode::ProtocolError;
        status.domain = ErrorDomain::Protocol;
        status.disconnect = true;
    }

    return status;
}

#endif // ERROR_STATUS_H
