/**
 * @file AutoPasConfigurationCommunicator.h
 * @author W. Thieme
 * @date 30.04.2020
 */

#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "WrapMPI.h"
#include "autopas/containers/ParticleContainerInterface.h"
#include "autopas/molecularDynamics/MoleculeLJ.h"
#include "autopas/selectors/Configuration.h"
#include "autopas/utils/ExceptionHandler.h"
#include "autopas/utils/NumberSet.h"
#include "autopas/utils/SimilarityFunctions.h"

/**
 * Provides several functions for handling configurations among mpi ranks.
 * This includes functionality for (de)serialization of configurations, splitting up search spaces based on ranks,
 * and finding the globally optimal configuration given time measurements.
 */

namespace autopas::utils::AutoPasConfigurationCommunicator {

/**
 * type definition for the serialization of configurations. A serialized config is an array of 12 bytes.
 * */
using SerializedConfiguration = std::array<std::byte, 13>;

/**
 * Simply a shorter way of static_casting from Option to std::byte.
 * @tparam TOption
 * @param option
 * @return
 */
template <typename TOption>
inline std::byte castToByte(TOption option) {
  return static_cast<std::byte>(static_cast<typename TOption::Value>(option));
}

/**
 * Calculates the maximum number of valid configs from several sets of options.
 * This does not equal the cartesian product as not all containers are compatible with all traversals.
 * @param containerOptions
 * @param cellSizeFactors The size of cellSizeFactors will only be taken into account if the NumberSet is finite.
 * @param traversalOptions
 * @param loadEstimatorOptions
 * @param dataLayoutOptions
 * @param newton3Options
 * @return
 */
size_t getSearchSpaceSize(const std::set<ContainerOption> &containerOptions, const NumberSet<double> &cellSizeFactors,
                          const std::set<TraversalOption> &traversalOptions,
                          const std::set<LoadEstimatorOption> &loadEstimatorOptions,
                          const std::set<DataLayoutOption> &dataLayoutOptions,
                          const std::set<Newton3Option> &newton3Options);

/**
 * Distributes the provided configurations globally for equal work loads.
 * All parameters' values (except for comm) are only relevant at the root node (0).
 * All parameters' values (except for comm) will be changed by this function.
 * @param containerOptions
 * @param cellSizeFactors
 * @param traversalOptions
 * @param loadEstimatorOptions
 * @param dataLayoutOptions
 * @param newton3Options
 * @param rank
 * @param commSize
 */
void distributeConfigurations(std::set<ContainerOption> &containerOptions, NumberSet<double> &cellSizeFactors,
                              std::set<TraversalOption> &traversalOptions,
                              std::set<LoadEstimatorOption> &loadEstimatorOptions,
                              std::set<DataLayoutOption> &dataLayoutOptions, std::set<Newton3Option> &newton3Options,
                              int rank, int commSize);

/**
 * Distribute ranks in buckets, which contain only ranks with similar scenarios.
 * Each bucket then has its own search space.
 * @tparam Particle
 * @param comm MPI communicator
 * @param bucket new MPI communicator for its bucket
 * @param container container of current simulation
 * @param MPITuningMaxDifferenceForBucket For MPI-tuning: Maximum of the relative difference in the comparison metric for two ranks which exchange their tuning information.
 * @param MPITuningWeightForMaxDensity For MPI-tuning: Weight for maxDensity in the calculation for bucket distribution.
 */
template <class Particle>
void distributeRanksInBuckets(AutoPas_MPI_Comm comm, AutoPas_MPI_Comm *bucket,
                              const std::shared_ptr<autopas::ParticleContainerInterface<Particle>> &container,
                              double MPITuningMaxDifferenceForBucket, double MPITuningWeightForMaxDensity) {
  int rank;
  AutoPas_MPI_Comm_rank(comm, &rank);
  int commSize;
  AutoPas_MPI_Comm_size(comm, &commSize);

  std::vector<double> scenarios(commSize);
  std::pair<double, double> homogeneityAndMaxDensity =
      autopas::utils::calculateHomogeneityAndMaxDensity<Particle>(container);
  double scenario = homogeneityAndMaxDensity.first + MPITuningWeightForMaxDensity * homogeneityAndMaxDensity.second;

  AutoPas_MPI_Allgather(&scenario, 1, AUTOPAS_MPI_DOUBLE, scenarios.data(), 1, AUTOPAS_MPI_DOUBLE, comm);

  std::sort(scenarios.begin(), scenarios.end());

  std::vector<double> differences;
  std::adjacent_difference(scenarios.begin(), scenarios.end(), std::back_inserter(differences));

  // convert differences to percentage changes
  std::transform(differences.begin(), differences.end(), scenarios.begin(), differences.begin(),
                 std::divides<double>());

  int current_bucket = 0;
  int my_bucket = 0;
  // print out the results

  for (int i = 0; (size_t)i < scenarios.size(); i++) {
    // if a difference exceeds 20%, start a new group:
    if (differences[i] > MPITuningMaxDifferenceForBucket) current_bucket++;

    // print out an item:
    AutoPasLog(debug, "rank: " + std::to_string(rank) + " bucket: " + std::to_string(current_bucket) +
                          "  new value: " + std::to_string(scenarios[i]));
    if (scenarios[i] == scenario) my_bucket = current_bucket;
  }
  AutoPas_MPI_Comm_split(comm, my_bucket, rank, bucket);
}

/**
 * Serializes a configuration object for communication via MPI.
 * @param configuration: the configuration to be sent.
 * @return The serialization
 */
SerializedConfiguration serializeConfiguration(Configuration configuration);

/**
 * Recreates a Configuration object from the object obtained by _serializeConfiguration.
 * @param config: The SerializedConfiguration objects returned by _serializeConfiguration.
 * @return The deserialized Configuration object.
 */
Configuration deserializeConfiguration(SerializedConfiguration config);

/**
 * Handles communication to select the globally best configuration.
 * @param comm: The communicator used for sending and receiving the optimal configuration.
 * @param localOptimalConfig: The locally optimal configuration to be compared with others.
 * @param localOptimalTime: The time measured for localOptimalConfig.
 * @return The globally optimal configuration.
 */
Configuration optimizeConfiguration(AutoPas_MPI_Comm comm, Configuration localOptimalConfig, size_t localOptimalTime);

}  // namespace autopas::utils::AutoPasConfigurationCommunicator
