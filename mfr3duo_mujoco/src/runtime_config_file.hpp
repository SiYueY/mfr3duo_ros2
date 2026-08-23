#pragma once

#include <filesystem>
#include <string>

namespace mfr3duo_mujoco {

class RuntimeConfigFile {
public:
    RuntimeConfigFile() = default;
    ~RuntimeConfigFile();
    RuntimeConfigFile(const RuntimeConfigFile&) = delete;
    RuntimeConfigFile& operator=(const RuntimeConfigFile&) = delete;

    bool create(const std::string& model_path, bool viewer_enabled);
    void reset() noexcept;
    const std::filesystem::path& path() const noexcept;
    const std::string& error() const noexcept;

private:
    std::filesystem::path path_;
    std::string error_;
};

}  // namespace mfr3duo_mujoco
