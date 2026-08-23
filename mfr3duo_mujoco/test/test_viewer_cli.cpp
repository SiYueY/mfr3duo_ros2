#include "runtime_config_file.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {
std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), {}};
}

std::string xml_escape(std::string value) {
    std::size_t position = 0;
    while ((position = value.find('&', position)) != std::string::npos) {
        value.replace(position, 1, "&amp;");
        position += 5;
    }
    return value;
}

TEST(RuntimeConfigFile, RendersHeadlessAndViewerVariants) {
    const auto canonical =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("mfr3duo_description")) /
        "mjcf" / "mfr3duo.xml";
    ASSERT_TRUE(std::filesystem::is_regular_file(canonical));

    mfr3duo_mujoco::RuntimeConfigFile headless;
    ASSERT_TRUE(headless.create(canonical.string(), false));
    EXPECT_NE(read_file(headless.path()).find("enabled=\"false\""), std::string::npos);
    EXPECT_TRUE(std::filesystem::exists(headless.path()));

    mfr3duo_mujoco::RuntimeConfigFile viewer;
    ASSERT_TRUE(viewer.create(canonical.string(), true));
    EXPECT_NE(read_file(viewer.path()).find("enabled=\"true\""), std::string::npos);
    viewer.reset();
    EXPECT_FALSE(std::filesystem::exists(viewer.path()));
}

TEST(RuntimeConfigFile, EscapesModelPathAndRejectsInvalidPaths) {
    const auto canonical =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("mfr3duo_description")) /
        "mjcf" / "mfr3duo.xml";
    ASSERT_TRUE(std::filesystem::is_regular_file(canonical));
    const auto escaped_model =
        std::filesystem::temp_directory_path() / "mfr3duo_runtime_config_escape&test.xml";
    std::error_code error;
    std::filesystem::remove(escaped_model, error);
    std::filesystem::create_symlink(canonical, escaped_model, error);
    ASSERT_FALSE(error);

    mfr3duo_mujoco::RuntimeConfigFile config;
    ASSERT_TRUE(config.create(escaped_model.string(), false));
    EXPECT_NE(read_file(config.path()).find(xml_escape(escaped_model.string())), std::string::npos);
    config.reset();
    std::filesystem::remove(escaped_model, error);
    EXPECT_FALSE(error);

    EXPECT_FALSE(config.create("relative/mfr3duo.xml", false));
    EXPECT_FALSE(config.create("/definitely/not/a/model.xml", false));
}
}  // namespace
