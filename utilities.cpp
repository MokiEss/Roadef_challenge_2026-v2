//
// Created by uha on 27/03/2026.
//
#include "utilities.h"
#include <random>
#include <algorithm>

namespace {
    std::mt19937& rng() {
        static thread_local std::mt19937 eng([]{
            std::random_device rd;
            std::seed_seq seed{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
            return std::mt19937(seed);
        }());
        return eng;
    }
} // namespace

int genRandomInt(int size) {
    if (size <= 0) return 0; // or handle as error
    std::uniform_int_distribution<int> dist(0, size - 1);
    return dist(rng());
}
double genRandomDouble(double size) {
    if (size <= 0.0) return 0.0; // or handle as error
    std::uniform_real_distribution<double> dist(0.0, size);
    return dist(rng());
}