#include <mfr3duo_mujoco/simulation.hpp>

int main()
{
    const mfr3duo_mujoco::Simulation simulation;
    return simulation.status() == mfr3duo_mujoco::SimulationStatus::Uninitialized
        ? 0 : 1;
}
