#include <iostream>
#include "Simulation.h"



int main() {
    Simulation sim = Simulation(1000, 100, 10, 0.01, 700, 3, 6);
    sim.run(10000);
}

