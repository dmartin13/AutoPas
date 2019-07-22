#pragma once

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include "autopas/autopasIncludes.h"
#include "autopas/utils/NumberSet.h"
using namespace std;
class YamlParser {
  /**
   * @file MDFlexParser.h
   * @date 23.02.2018
   * @author F. Gratl
   */
 public:
  enum FunctorOption { lj12_6, lj12_6_AVX };
  enum GeneratorOption { grid, uniform, gaussian };

    struct Object {
        std::array<double, 3> velocity;
        std::array<size_t, 3> particlesPerDim;
        double particleSpacing;
        std::array<double, 3> boxLength;
        size_t numParticles;
        double distributionMean;
        double distributionStdDev;
        std::array<double, 3> center;
        unsigned long id;
        int radius;
    };

    struct CubeGrid : Object {
//      CubeGrid(): particlesPerDim({20,20,20}), particleSpacing(.4),velocity({0.,0.,0.}){}
        std::array<size_t, 3> particlesPerDim;
        double particleSpacing;
        std::array<double, 3> velocity;
    };
    struct CubeGauss : Object {
        std::array<double, 3> boxLength;
        size_t numParticles;
        double distributionMean;
        double distributionStdDev;
        std::array<double, 3> velocity;
    };
    struct CubeUniform : Object {
        std::array<double, 3> boxLength;
        size_t numParticles;
        std::array<double, 3> velocity;
    };
    struct Sphere : Object {
        std::array<double, 3> center;
        int radius;
        double particleSpacing;
        unsigned long id;
        std::array<double, 3> velocity;
    };
  /**Constructor für YAMl Parser:
   * */
  YamlParser() = default;
  /**Parses the Input for the simulation
   * @param filename
   * */
  void parseInput(std::string &filename);

  /**Prints Configuration of Simulation:
   * */
  void printConfig();

  const set<ContainerOption> &getContainerOptions() const;

  const set<DataLayoutOption> &getDataLayoutOptions() const;

  SelectorStrategyOption getSelectorStrategy() const;

  const set<TraversalOption> &getTraversalOptions() const;

  TuningStrategyOption getTuningStrategyOption() const;

  const set<Newton3Option> &getNewton3Options() const;

  const NumberSet<double> &getCellSizeFactors() const;

  double getBoxLength() const;

  double getCutoff() const;

  double getDistributionMean() const;

  double getDistributionStdDev() const;

  FunctorOption getFunctorOption() const;

  GeneratorOption getGeneratorOption() const;

  size_t getIterations() const;

  spdlog::level::level_enum getLogLevel() const;

  bool getMeasureFlops() const;

  size_t getParticlesPerDim() const;

  size_t getParticlesTotal() const;

  double getParticleSpacing() const;

  unsigned int getTuningInterval() const;

  unsigned int getTuningSamples() const;

  unsigned int getTuningMaxEvidence() const;

  const string &getWriteVtk() const;

  const string &getLogFileName() const;

  unsigned int getVerletRebuildFrequency() const;

  double getVerletSkinRadius() const;

  double getEpsilon() const;

  double getSigma() const;

  double getDeltaT() const;

  double getMass() const;

 private:
  static constexpr size_t valueOffset = 32;
  // defaults:
  // AutoPas options:
  std::set<autopas::ContainerOption> containerOptions = autopas::allContainerOptions;
  std::set<autopas::DataLayoutOption> dataLayoutOptions = autopas::allDataLayoutOptions;
  autopas::SelectorStrategyOption selectorStrategy = autopas::SelectorStrategyOption::fastestAbs;
  std::set<autopas::TraversalOption> traversalOptions = autopas::allTraversalOptions;
  autopas::TuningStrategyOption tuningStrategyOption = autopas::TuningStrategyOption::fullSearch;
  std::set<autopas::Newton3Option> newton3Options = autopas::allNewton3Options;
  std::shared_ptr<autopas::NumberSet<double>> cellSizeFactors =
      std::make_shared<autopas::NumberSetFinite<double>>(std::set<double>{1.});

  // Simulation Options:
  double cutoff = 1.;

  FunctorOption functorOption = FunctorOption::lj12_6;
  GeneratorOption generatorOption = GeneratorOption::grid;

  std::vector<Object> ObjectGenerator= {};

  size_t iterations = 10;
  spdlog::level::level_enum logLevel = spdlog::level::info;
  bool measureFlops = true;


  size_t particlesPerDim = 20;
  size_t particlesTotal = 1000;
  double particleSpacing = .4;
    double distributionMean = 5.;
    double distributionStdDev = 2.;
    double boxLength = -1;


    unsigned int tuningInterval = 100;
  unsigned int tuningSamples = 3;
  unsigned int tuningMaxEvidence = 10;
  std::string writeVTK = "";
  std::string logFileName = "";
  unsigned int verletRebuildFrequency = 5;
  double verletSkinRadius = .2;
  double epsilon = 5.0;
  double sigma = 1.0;
  double delta_t = 0.001;
  double mass = 1.0;
  // Object Generation Options:
};
