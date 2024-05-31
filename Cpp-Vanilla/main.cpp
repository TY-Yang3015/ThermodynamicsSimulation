#include <iostream>
#include "Simulation.h"



int main() {
    Simulation sim = Simulation(1500, 10, 1000, 1, 1000, 4, 8);
    // sim.runByCollision(10000);
    sim.runByTime(10000, 100000);
}

