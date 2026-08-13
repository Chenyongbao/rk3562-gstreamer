#ifndef JSON_UTILS_H
#define JSON_UTILS_H

#include <string>
#include <sstream>

namespace JsonUtils {

inline std::string escape(const std::string& input) {
    std::ostringstream oss;
    for (char c : input) {
        switch (c) {
        case '"':  oss << "\\\""; break;
        case '\\': oss << "\\\\"; break;
        case '\n': oss << "\\n";  break;
        case '\r': oss << "\\r";  break;
        case '\t': oss << "\\t";  break;
        default:   oss << c;      break;
        }
    }
    return oss.str();
}

inline bool parseNumber(const std::string& body, const char* key, double& out) {
    std::string pattern = std::string("\"") + key + "\"";
    size_t pos = body.find(pattern);
    if (pos == std::string::npos) return false;
    pos = body.find(':', pos);
    if (pos == std::string::npos) return false;
    const char* start = body.c_str() + pos + 1;
    while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r') ++start;
    char* endptr = nullptr;
    double v = std::strtod(start, &endptr);
    if (endptr == start) return false;
    out = v;
    return true;
}

} // namespace JsonUtils

#endif
