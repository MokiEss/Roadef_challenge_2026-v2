//
// Created by uha on 27/03/2026.
//
#include "utilities.h"
int genRandomInt(int size) {
    random_device rd;  // Obtain a random number from hardware
    mt19937 gen(rd()); // Seed the generator
    uniform_int_distribution<> distr(0, size - 1); // Define the range
    return distr(gen);
}
double genRandomDouble(double size) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> distr(0, size);
    return distr(gen);
}