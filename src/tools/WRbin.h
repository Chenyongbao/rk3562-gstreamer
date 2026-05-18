#ifndef WRITEBIN_H
#define WRITEBIN_H

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

class WRbin {
public:
    static WRbin& instance() {
        static WRbin inst;
        return inst;
    }

    bool write(const std::string& path, const void* data, size_t size) const {
        FILE* bin_fp = fopen(path.c_str(), "wb");
        if (!bin_fp) {
            std::cerr << "[WRbin] Failed to open " << path << " for binary write" << std::endl;
            return false;
        }

        size_t written = fwrite(data, size, 1, bin_fp);
        fclose(bin_fp);

        if (written != 1) {
            std::cerr << "[WRbin] Failed to write binary to " << path << std::endl;
            return false;
        }

        std::cout << "[WRbin] Saved binary to " << path << std::endl;
        return true;
    }

    bool read(const std::string& path, void* data, size_t size) const {
        FILE* bin_fp = fopen(path.c_str(), "rb");
        if (!bin_fp) {
            std::cerr << "[WRbin] Failed to open " << path << " for binary read" << std::endl;
            return false;
        }

        size_t nread = fread(data, 1, size, bin_fp);
        fclose(bin_fp);

        if (nread != size) {
            std::cerr << "[WRbin] Failed to read binary from " << path << " (expected "
                      << size << ", got " << nread << ")" << std::endl;
            return false;
        }

        std::cout << "[WRbin] Read binary from " << path << " (" << nread << " bytes)" << std::endl;
        return true;
    }

    bool readAll(const std::string& path, std::vector<uint8_t>& out_data) const {
        FILE* bin_fp = fopen(path.c_str(), "rb");
        if (!bin_fp) {
            std::cerr << "[WRbin] Failed to open " << path << " for binary read" << std::endl;
            return false;
        }

        if (fseek(bin_fp, 0, SEEK_END) != 0) {
            fclose(bin_fp);
            std::cerr << "[WRbin] Failed to seek end of " << path << std::endl;
            return false;
        }

        long file_size = ftell(bin_fp);
        if (file_size < 0) {
            fclose(bin_fp);
            std::cerr << "[WRbin] Failed to get file size for " << path << std::endl;
            return false;
        }

        if (fseek(bin_fp, 0, SEEK_SET) != 0) {
            fclose(bin_fp);
            std::cerr << "[WRbin] Failed to seek start of " << path << std::endl;
            return false;
        }

        out_data.resize(static_cast<size_t>(file_size));
        size_t nread = fread(out_data.data(), 1, static_cast<size_t>(file_size), bin_fp);
        fclose(bin_fp);

        if (nread != static_cast<size_t>(file_size)) {
            std::cerr << "[WRbin] Failed to read entire file " << path << " (expected "
                      << file_size << ", got " << nread << ")" << std::endl;
            return false;
        }

        std::cout << "[WRbin] Read " << file_size << " bytes from " << path << std::endl;
        return true;
    }

    bool append(const std::string& path, const void* data, size_t size) const {
        FILE* bin_fp = fopen(path.c_str(), "ab");
        if (!bin_fp) {
            std::cerr << "[WRbin] Failed to open " << path << " for binary append" << std::endl;
            return false;
        }

        size_t written = fwrite(data, size, 1, bin_fp);
        fclose(bin_fp);

        if (written != 1) {
            std::cerr << "[WRbin] Failed to append binary to " << path << std::endl;
            return false;
        }

        std::cout << "[WRbin] Appended " << size << " bytes to " << path << std::endl;
        return true;
    }

private:
    WRbin() = default;
    ~WRbin() = default;
    WRbin(const WRbin&) = delete;
    WRbin& operator=(const WRbin&) = delete;
};

#endif // WRITEBIN_H
