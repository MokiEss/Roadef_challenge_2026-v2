// main.cpp

#include "solver.h"
#include <chrono>  // Add this include

#include "readerInstance.h"




int main() {
    string nInstance; cout << "Instance number " ; cin >> nInstance ;
    Instance      inst;
    bool use_ftxui = false ;
    ResultBuilder result_builder(inst, use_ftxui);
    Scenario scenario;
    SegmentRouting sr(inst.network, inst.metrics);
    PreprocessingEngine engine = readAndPreprocess( nInstance, inst, use_ftxui, result_builder, scenario);

    solver s(nInstance,inst, use_ftxui, result_builder, scenario, sr);
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    s.optimize();
    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Optimization time: " << duration.count() << " seconds" << std::endl;


    return 0;
}