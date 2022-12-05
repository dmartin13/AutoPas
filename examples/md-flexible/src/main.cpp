/**
 * @file main.cpp
 * @date 23.02.2018
 * @author F. Gratl
 */

#include <fstream>

#include "Simulation.h"
#include "autopas/utils/WrapMPI.h"
#include "TypeDefinitions.h"

// Declare the main AutoPas class as extern template instantiation. It is instantiated in AutoPasClass.cpp.
extern template class autopas::AutoPas<ParticleType>;

/**
 * The main function for md-flexible.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
  autopas::AutoPas_MPI_Init(&argc, &argv);
  {
    MDFlexConfig configuration(argc, argv);

    auto domainDecomposition = std::make_shared<RegularGridDecomposition>(configuration);

    if (not configuration.checkpointfile.value.empty()) {
      configuration.flushParticles();
      configuration.loadParticlesFromCheckpoint(domainDecomposition->getDomainIndex(),
                                                domainDecomposition->getSubdomainCount());
    }

    // print start configuration and parallelization info
    if (domainDecomposition->getDomainIndex() == 0) {
      std::cout << configuration.to_string() << std::endl;
      std::cout << std::endl << "Using " << autopas::autopas_get_max_threads() << " Threads" << std::endl;
#if defined(AUTOPAS_INCLUDE_MPI)
      std::cout << "MPI is running with " << domainDecomposition->getNumberOfSubdomains() << " ranks." << std::endl;
#else
      std::cout << "MPI is disabled." << std::endl;
#endif
    }

    Simulation simulation(configuration, domainDecomposition);
    simulation.run();
    simulation.finalize();

    // if desired, print the configuration as a file at the end of the simulation.
    if (domainDecomposition->getDomainIndex() == 0) {
      if (configuration.dontCreateEndConfig.value) {
        std::ofstream configFileEnd("MDFlex_end_" + autopas::utils::Timer::getDateStamp() + ".yaml");
        if (configFileEnd.is_open()) {
          configFileEnd << "# Generated by:" << std::endl;
          configFileEnd << "# ";
          for (int i = 0; i < argc; ++i) {
            configFileEnd << argv[i] << " ";
          }
          configFileEnd << std::endl;
          configFileEnd << configuration;
          configFileEnd.close();
        }
      }
    }
  }
  autopas::AutoPas_MPI_Finalize();
  return EXIT_SUCCESS;
}
