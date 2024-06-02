// include "include/Vanilla/Simulation.h"
// include "include/VDW/Simulation.h"
#include "include/LJ/Simulation.h"

#pragma GCC optimize ("O3")

int main() {
    Simulation sim = Simulation(1500, 100, 100, 100, 1000, 4, 6);
    sim.recordSimulationSpecs();
    sim.setSystemLog(true, true);
    // sim.runByCollision(10000);
    // sim.runByTime(10000, 100000);
    sim.run(100, 100000);
}

