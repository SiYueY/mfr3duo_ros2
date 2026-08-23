#include "mfr3duo_mujoco/simulation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

namespace {
std::set<std::filesystem::path> temporary_configs() {
    std::set<std::filesystem::path> paths;
    std::error_code error;
    const auto directory = std::filesystem::temp_directory_path(error);
    if (error) return paths;
    for (const auto& entry : std::filesystem::directory_iterator(directory, error)) {
        if (error) break;
        const auto filename = entry.path().filename().string();
        if (filename.rfind("mfr3duo_robot_mujoco_", 0) == 0 && entry.path().extension() == ".xml")
            paths.insert(entry.path());
    }
    return paths;
}
}  // namespace

TEST(Simulation, RejectsEmptyAndRelativeModelPaths) {
    mfr3duo_mujoco::Simulation simulation;
    EXPECT_FALSE(simulation.initialize({}));
    EXPECT_FALSE(simulation.initialize({"relative/model.xml"}));
}

TEST(Simulation, InitializesTheInstalledTemplateAndCleansItUp) {
    const auto model =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("mfr3duo_description")) /
        "mjcf" / "mfr3duo.xml";
    ASSERT_TRUE(std::filesystem::is_regular_file(model));
    const auto before = temporary_configs();

    mfr3duo_mujoco::Simulation simulation;
    ASSERT_TRUE(simulation.initialize({model.string()}));
    const auto during = temporary_configs();
    ASSERT_EQ(during.size(), before.size() + 1U);
    std::set<std::filesystem::path> created;
    std::set_difference(
        during.begin(), during.end(), before.begin(), before.end(),
        std::inserter(created, created.end()));
    ASSERT_EQ(created.size(), 1U);
    const auto& generated = *created.begin();
    std::ifstream config(generated);
    const std::string contents((std::istreambuf_iterator<char>(config)), {});
    EXPECT_NE(contents.find(model.string()), std::string::npos);
    EXPECT_TRUE(simulation.step());
    EXPECT_TRUE(simulation.reset());
    EXPECT_TRUE(simulation.reset("home"));
    EXPECT_TRUE(simulation.shutdown());
    EXPECT_EQ(temporary_configs(), before);
}
