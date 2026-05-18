#ifndef ERROR_CODE_H
#define ERROR_CODE_H

enum class ErrorCode {
    Ok,
    Busy,
    UnknownCommand,
    ProtocolError,
    SendFailed,
    HomingFailed,
    CaptureFailed,
    DetectionFailed,
    CalibrationFailed,
    Timeout,
    InternalError
};

enum class ErrorDomain {
    None,
    Protocol,
    Scheduler,
    Device,
    Vision,
    Calibration,
    System
};

inline const char* toString(ErrorCode code)
{
    switch (code) {
    case ErrorCode::Ok:
        return "OK";
    case ErrorCode::Busy:
        return "BUSY";
    case ErrorCode::UnknownCommand:
        return "UNKNOWN_COMMAND";
    case ErrorCode::ProtocolError:
        return "PROTOCOL_ERROR";
    case ErrorCode::SendFailed:
        return "SEND_FAILED";
    case ErrorCode::HomingFailed:
        return "HOMING_FAILED";
    case ErrorCode::CaptureFailed:
        return "CAPTURE_FAILED";
    case ErrorCode::DetectionFailed:
        return "DETECTION_FAILED";
    case ErrorCode::CalibrationFailed:
        return "CALIBRATION_FAILED";
    case ErrorCode::Timeout:
        return "TIMEOUT";
    case ErrorCode::InternalError:
        return "INTERNAL_ERROR";
    }
    return "INTERNAL_ERROR";
}

inline const char* toString(ErrorDomain domain)
{
    switch (domain) {
    case ErrorDomain::None:
        return "none";
    case ErrorDomain::Protocol:
        return "protocol";
    case ErrorDomain::Scheduler:
        return "scheduler";
    case ErrorDomain::Device:
        return "device";
    case ErrorDomain::Vision:
        return "vision";
    case ErrorDomain::Calibration:
        return "calibration";
    case ErrorDomain::System:
        return "system";
    }
    return "system";
}

#endif // ERROR_CODE_H
