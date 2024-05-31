#include <iostream>
#include "Simulation.h"

#pragma GCC optimize ("O3")

int main() {
    Simulation sim = Simulation(1500, 100, 100, 1, 1000, 4, 8);
    sim.setAttractionStrength(10000);
    sim.run(10000, 1000000);
}

