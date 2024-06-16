#include "include/Vanilla/Simulation.h"
// #include "include/VDW/Simulation.h"
// #include "include/LJ/Simulation.h"

#pragma GCC optimize ("O3")

int main() {
    int radList[] = {5, 10, 15, 20};
    for (auto& rad : radList) {
        Simulation sim = Simulation(1000, rad, 100, 1e-22, 800, 6, 8);
        sim.recordSimulationSpecs();
        sim.setSystemLog(true, true);
        //sim.runByCollision(10000);
        sim.runByTimeAnimationOff(10000, 20000);
        // sim.runByCollisionAnimationOff(30000);
        // sim.run(100, 100000);
    }
}

