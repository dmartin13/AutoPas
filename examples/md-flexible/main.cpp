/**
 * @file main.cpp
 * @date 23.02.2018
 * @author F. Gratl
 */

#include <iostream>

#include "PrintableMolecule.h"
#include "Simulation.h"
#include "parsing/MDFlexParser.h"

int main(int argc, char **argv) {
  // start simulation timer
  Simulation<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>> simulation;
  // Parsing
  MDFlexConfig config;

  if (not MDFlexParser::parseInput(argc, argv, config)) {
    exit(EXIT_FAILURE);
  }
  // make sure sim box is big enough
  config.calcSimulationBox();

  std::cout << config;

  // Initialization
  simulation.initialize(config);
  std::cout << std::endl << "Using " << autopas::autopas_get_max_threads() << " Threads" << std::endl;

  // Simulation
  std::cout << "Starting simulation... " << std::endl;
  simulation.simulate();
  std::cout << "Simulation done!" << std::endl << std::endl;

  simulation.printStatistics();

  // print config.yaml file of current run
  if (config.dontCreateEndConfig) {
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream nowStrStr;
    tm unused;
    nowStrStr << std::put_time(localtime_r(&now, &unused), "%Y-%m-%d_%H-%M-%S");
    std::ofstream configFileEnd("MDFlex_end_" + nowStrStr.str() + ".yaml");
    configFileEnd << "# Generated by:" << std::endl;
    configFileEnd << "# ";
    for (int i = 0; i < argc; ++i) {
      configFileEnd << argv[i] << " ";
    }
    configFileEnd << std::endl;
    configFileEnd << config;
    configFileEnd.close();
  }

  return EXIT_SUCCESS;
}
