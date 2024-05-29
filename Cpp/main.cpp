#include <iostream>
#include "Simulation.h"



int main() {
    Simulation sim = Simulation(1000, 10, 10, 0.01, 700, 10, 6);
    sim.run(10000);
}

