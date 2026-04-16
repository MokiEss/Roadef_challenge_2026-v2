//
// Created by uha on 26/03/2026.
//

#ifndef ROADEF_CHALLENGE_2026_READERINSTANCE_H
#define ROADEF_CHALLENGE_2026_READERINSTANCE_H

// main.cpp
#include "utilities.h"
#include "DataAnalysis.h"
using namespace  std ;
void ReadNetworkInstance(string net_file, string scenario_file,  string tm_file, Instance    &  inst,
    bool use_ftxui, ResultBuilder & result_builder, Scenario & scenario );

PreprocessingEngine readAndPreprocess(string nInstance, Instance    &  inst,
    bool & use_ftxui, ResultBuilder & result_builder, Scenario & scenario );




#endif //ROADEF_CHALLENGE_2026_READERINSTANCE_H