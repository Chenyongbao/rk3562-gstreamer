#ifndef TOTAL_HIGH_H
#define TOTAL_HIGH_H

#include <string>
#include "../config.h"

class TotalHighService {
public:
    explicit TotalHighService(double feedrate = CALIB_FEEDRATE,
                              const char* confPath = REALLINK_CV_CONF_PATH);
    ~TotalHighService() = default;

    bool measure(double& outHeight, std::string& errorMsg);
    bool readTotalHigh(double& outHeight, std::string& errorMsg) const;

private:
    void sendGcodeScript(const std::string& script);
    void saveTotalHigh(double height);

    double feedrate_;
    std::string conf_path_;
};

#endif // TOTAL_HIGH_H
