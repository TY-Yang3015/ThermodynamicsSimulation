#include <iostream>
#include "Simulation.h"



int main() {
    Simulation sim = Simulation(1000, 10, 10, 0.01, 700, 6, 6);
    // sim.runByCollision(10000);
    sim.runByTime(10000, 100000);
}
