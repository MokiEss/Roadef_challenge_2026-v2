//
// Created by uha on 26/03/2026.
//

#include "readerInstance.h"
void ReadNetworkInstance(string net_file, string scenario_file,  string tm_file, Instance    &  inst,
    bool use_ftxui, ResultBuilder & result_builder, Scenario & scenario ) {
    int i_max_decimal_places = 12;
    // Validate input files format and semantics before loading the instance
    if (!validateNetworkJsonFormat(net_file)  ) {
        //  result_builder.setValid(false);
        // result_builder.display(i_max_decimal_places);

        return ;
    }
    else {

    }
    if ( !validateTrafficMatrixJsonFormat(tm_file)  ) {
        //  result_builder.setValid(false);
        // result_builder.display(i_max_decimal_places);

        return ;
    }
    else {

    }

    if (!validateScenarioJsonFormat(scenario_file) ) {
        //  result_builder.setValid(false);
        // result_builder.display(i_max_decimal_places);

        return ;
    }
    else {

    }

    // Load TE instance
    if (!loadAndCheckTeInstance(net_file, tm_file, inst, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return ;
    }


    // Load interventions scenario

    if (!loadAndCheckScenario(scenario_file, inst, scenario, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return ;
    }
    else {
        cout<<"finish loading scenario"<<endl;
    }

    cout << "Done!" << endl;



}


PreprocessingEngine readAndPreprocess(string nInstance, Instance    &  inst,
    bool & use_ftxui, ResultBuilder & result_builder, Scenario & scenario )

{
    string net_file = "../challenge-roadef-2026-main/setA/setA-" + nInstance + "-net.json";
    string scenario_file = "../challenge-roadef-2026-main/setA/setA-" + nInstance + "-scenario.json";
    string tm_file = "../challenge-roadef-2026-main/setA/setA-" + nInstance + "-tm.json";
    use_ftxui = false ;
    ReadNetworkInstance( net_file,  scenario_file,   tm_file,  inst,
         use_ftxui,  result_builder, scenario );

    PreprocessingEngine engine(inst, scenario);
    //PreprocessingData prep_data = engine.preprocess();

    // Display results
   // engine.displayResults(12);
    return engine;
}