// main.cpp

#include "solver.h"


#include "readerInstance.h"




int main() {
    string nInstance; cout << "Instance number " ; cin >> nInstance ;
    Instance      inst;
    bool use_ftxui = false ;
    ResultBuilder result_builder(inst, use_ftxui);
    Scenario scenario;
    SegmentRouting sr(inst.network, inst.metrics);
    PreprocessingEngine engine = readAndPreprocess( nInstance, inst, use_ftxui, result_builder, scenario);

    solver s(inst, use_ftxui, result_builder, scenario, sr);
    s.optimize();
    // Validate budget constraints

    return 0;
}