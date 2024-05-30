#include <iostream>
#include "Simulation.h"



int main() {
    Simulation sim = Simulation(1000, 10, 10, 1, 700, 2, 6);
    // sim.runByCollision(10000);
    sim.run(10000, 100000);
}

