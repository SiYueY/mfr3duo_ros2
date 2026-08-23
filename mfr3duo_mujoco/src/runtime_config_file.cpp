#include "runtime_config_file.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include <unistd.h>

namespace mfr3duo_mujoco {
namespace {
bool is_readable_regular_file(const std::filesystem::path& path) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) || error) return false;
    std::ifstream input(path, std::ios::binary);
    return input.good();
}

std::string escape_xml(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char character : value) {
        switch (character) {
            case '&':
                escaped += "&amp;";
                break;
            case '<':
                escaped += "&lt;";
                break;
            case '>':
                escaped += "&gt;";
                break;
            case '\"':
                escaped += "&quot;";
                break;
            case '\'':
                escaped += "&apos;";
                break;
            default:
                escaped += character;
                break;
        }
    }
    return escaped;
}

bool replace_once(std::string& xml, const char* placeholder, const std::string& value) {
    const auto location = xml.find(placeholder);
    if (location == std::string::npos ||
        xml.find(placeholder, location + std::char_traits<char>::length(placeholder)) !=
            std::string::npos)
        return false;
    xml.replace(location, std::char_traits<char>::length(placeholder), value);
    return true;
}

bool write_temporary_config(const std::string& contents, std::filesystem::path& path) {
    std::error_code error;
    const auto directory = std::filesystem::temp_directory_path(error);
    if (error) return false;
    std::string pattern = (directory / "mfr3duo_robot_mujoco_XXXXXX.xml").string();
    const int descriptor = mkstemps(pattern.data(), 4);
    if (descriptor == -1) return false;
    std::size_t written = 0;
    while (written < contents.size()) {
        const auto count = write(descriptor, contents.data() + written, contents.size() - written);
        if (count > 0) {
            written += static_cast<std::size_t>(count);
            continue;
        }
        if (count == -1 && errno == EINTR) continue;
        break;
    }
    close(descriptor);
    if (written != contents.size()) {
        std::filesystem::remove(pattern, error);
        return false;
    }
    path = std::move(pattern);
    return true;
}
}  // namespace

RuntimeConfigFile::~RuntimeConfigFile() { reset(); }

bool RuntimeConfigFile::create(const std::string& model_path, bool viewer_enabled) {
    reset();
    error_.clear();
    const std::filesystem::path model(model_path);
    if (model_path.empty() || !model.is_absolute() || !is_readable_regular_file(model)) {
        error_ = "model path must be an existing, readable absolute file";
        return false;
    }
    try {
        const auto template_path =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("mfr3duo_mujoco")) /
            "config" / "robot_mujoco.xml.in";
        std::ifstream template_file(template_path, std::ios::binary);
        if (!template_file) {
            error_ = "cannot read installed robot_mujoco.xml.in";
            return false;
        }
        std::string xml(
            (std::istreambuf_iterator<char>(template_file)), std::istreambuf_iterator<char>());
        if (!replace_once(xml, "@MJCF_PATH@", escape_xml(model.string())) ||
            !replace_once(xml, "@VIEWER_ENABLED@", viewer_enabled ? "true" : "false")) {
            error_ = "installed robot_mujoco.xml.in has invalid placeholders";
            return false;
        }
        if (!write_temporary_config(xml, path_)) {
            error_ = "cannot create temporary robot_mujoco.xml";
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error_ = exception.what();
        reset();
        return false;
    }
}

void RuntimeConfigFile::reset() noexcept {
    if (path_.empty()) return;
    std::error_code error;
    std::filesystem::remove(path_, error);
    path_.clear();
}

const std::filesystem::path& RuntimeConfigFile::path() const noexcept { return path_; }

const std::string& RuntimeConfigFile::error() const noexcept { return error_; }

}  // namespace mfr3duo_mujoco
