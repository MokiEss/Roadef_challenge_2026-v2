#pragma once

// C++20 standard includes
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cassert>
#include <limits>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>
#include <numeric>
#include <optional>
#include <span>
#include <ranges>



using namespace std;

// ── Compile-time constants ────────────────────────────────────────
static constexpr double INF_DIST   = std::numeric_limits<double>::infinity();
static constexpr double EPSILON    = 1e-9;
static constexpr double EPSILON2   = 1e-6;
static constexpr long long LL_INF  = std::numeric_limits<long long>::max() / 2;
