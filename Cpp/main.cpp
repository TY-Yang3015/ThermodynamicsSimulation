#include "include/Vanilla/Simulation.h"
// #include "include/VDW/Simulation.h"
// #include "include/LJ/Simulation.h"

#pragma GCC optimize ("O3")

int main() {
    Simulation sim = Simulation(1500, 10, 1000, 100, 1000, 4, 8);
    // sim.runByCollision(10000);
    sim.recordSimulationSpecs();
    sim.setSystemLog(true, true);
    sim.runByTime(10000, 100000);
    // sim.run(10000, 100000);
}

