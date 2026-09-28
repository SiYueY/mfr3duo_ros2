# mfr3duo_ros2

Mobile FR3 Duo 的 ROS 2 Humble 集成工作区（Ubuntu 22.04）。本仓库管理六个包及其构建顺序；`mfr3duo_description` 和 `mfr3duo_mujoco` 是固定提交的 Git submodule。当前 `mfr3duo_hardware`、`mfr3duo_moveit`、`mfr3duo_nav`、`mfr3duo_robot` 只提供包骨架和依赖声明，尚不提供硬件插件、规划、导航或启动功能。

## 获取源码

```bash
git clone --recurse-submodules -b develop https://github.com/SiYueY/mfr3duo_ros2.git
cd mfr3duo_ros2
```

已有检出可运行 `git submodule update --init --recursive` 取得父仓库固定的提交。需要主动跟踪两个 submodule 的 `develop` 分支时，运行 `git submodule update --remote --recursive`，检查变更后提交父仓库更新的 gitlink。

## 依赖与构建

先安装 ROS 2 Humble、colcon 和六个包声明的 ROS 依赖，并构建安装 `romujoco >= 0.1.0`。MuJoCo 依赖的完整准备方法见 [mfr3duo_mujoco 的构建说明](mfr3duo_mujoco/README.md)。例如，在工作区外执行：

```bash
git clone --recurse-submodules -b develop https://github.com/SiYueY/robot_mujoco.git
cd robot_mujoco/romujoco
./scripts/mujoco.sh build -j 2
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/romujoco-install" -DROMUJOCO_BUILD_TESTS=OFF
cmake --build build --parallel 2
cmake --install build
```

将 `romujoco` 的安装前缀加入 `CMAKE_PREFIX_PATH`。本工作区不要求安装 Pinocchio，因为 colcon 构建会关闭 MuJoCo teleop，独立构建的默认设置不变。

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$HOME/romujoco-install:${CMAKE_PREFIX_PATH}"
colcon list --topological-order
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

期望发现六个包；`mfr3duo_description` 先于 `mfr3duo_mujoco`，后者先于 `mfr3duo_hardware`。硬件包的链接测试检查 `mfr3duo_mujoco::mfr3duo_mujoco` 可被安装包消费者使用。更多边界与依赖关系见 [架构说明](docs/architecture.md)。
