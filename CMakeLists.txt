cmake_minimum_required(VERSION 4.0)
project(Roadef_challenge_2026)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wno-c++11-narrowing -Wno-switch)

add_executable(Roadef_challenge_2026 main.cpp
        DataAnalysis.h
        DataAnalysis.cpp
        readerInstance.h
        readerInstance.cpp
        solution.cpp
        solution.h
        utilities.h
        utilities.cpp
        solver.h
        solver.cpp)

target_include_directories(Roadef_challenge_2026 PRIVATE
        ${CMAKE_SOURCE_DIR}/networktools-main/networktools
)

target_include_directories(Roadef_challenge_2026 PRIVATE
        ${CMAKE_SOURCE_DIR}/challenge-roadef-2026-main/checker/src
)