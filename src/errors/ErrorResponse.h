#ifndef ERROR_RESPONSE_H
#define ERROR_RESPONSE_H

#include "ErrorStatus.h"

#include <sstream>
#include <string>

inline std::string escapeJsonString(const std::string& input)
{
    std::ostringstream oss;
    for (char c : input) {
    switch (c) {
    case '"':
        oss << "\\\"";
        break;
    case '\\':
        oss << "\\\\";
        break;
    case '\n':
        oss << "\\n";
        break;
    case '\r':
        oss << "\\r";
        break;
    case '\t':
        oss << "\\t";
        break;
    default:
        oss << c;
        break;
    }
    }
    return oss.str();
}

inline std::string buildErrorResponseJson(const ErrorStatus& status,
                                          const std::string& command)
{
    std::ostringstream json;
    json << "{"
         << "\"status\":\"FAILED\","
         << "\"code\":\"" << toString(status.code) << "\","
         << "\"domain\":\"" << toString(status.domain) << "\","
         << "\"message\":\"" << escapeJsonString(status.message) << "\","
         << "\"retryable\":" << (status.retryable ? "true" : "false") << ","
         << "\"disconnect\":" << (status.disconnect ? "true" : "false") << ","
         << "\"command\":\"" << escapeJsonString(command) << "\""
         << "}\n";
    return json.str();
}

#endif // ERROR_RESPONSE_H
