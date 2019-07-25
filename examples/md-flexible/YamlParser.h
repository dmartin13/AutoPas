#pragma once

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include "Objects.h"
#include "autopas/autopasIncludes.h"
#include "autopas/utils/NumberSet.h"
#include <limits>
#include <array>
#include <algorithm>
#include <getopt.h>
class YamlParser {
  /**
   * @file MDFlexParser.h
   * @date 23.02.2018
   * @author F. Gratl
   */
 public:
  enum FunctorOption { lj12_6, lj12_6_AVX };

  /**Constructor für YAMl Parser:
   * */
  YamlParser() = default;

    const std::array<double, 3> &getBoxMin() const;

    const std::array<double, 3> &getBoxMax() const;

    bool parseInput(int argc, char **argv);


    /**Parses the Input for the simulation
     * @param filename
     * */
    void parseYamlFile();

  /**Prints Configuration of Simulation:
   * */
  void printConfig();
  /**Calculates the total number of Particles generated
   * @return particlestotal
   * */
  size_t particlesTotal();

  /**Calculate the required Box for the AutoPas Object
   * */
  void calcAutopasBox();

  const std::set<autopas::ContainerOption> &getContainerOptions() const;

  const std::set<autopas::DataLayoutOption> &getDataLayoutOptions() const;

    autopas::SelectorStrategyOption getSelectorStrategy() const;

  const std::set<autopas::TraversalOption> &getTraversalOptions() const;

    autopas::TuningStrategyOption getTuningStrategyOption() const;

  const std::set<autopas::Newton3Option> &getNewton3Options() const;

  const autopas::NumberSet<double> &getCellSizeFactors() const;

  double getCutoff() const;

  FunctorOption getFunctorOption() const;

  size_t getIterations() const;

  spdlog::level::level_enum getLogLevel() const;

  bool getMeasureFlops() const;

  unsigned int getTuningInterval() const;

  unsigned int getTuningSamples() const;

  unsigned int getTuningMaxEvidence() const;

  const std::string &getWriteVtk() const;

  const std::string &getLogFileName() const;

  unsigned int getVerletRebuildFrequency() const;

  double getVerletSkinRadius() const;

  double getEpsilon() const;

  double getSigma() const;

  double getDeltaT() const;

  double getMass() const;

  const std::vector<CubeGrid> &getCubeGrid() const;

  const std::vector<CubeGauss> &getCubeGauss() const;

  const std::vector<CubeUniform> &getCubeUniform() const;

  const std::vector<Sphere> &getSphere() const;

 private:
  static constexpr size_t valueOffset = 32;
  // defaults:
  std::string filename = "DefaultConfig.yaml";
  // AutoPas options:
  std::set<autopas::ContainerOption> containerOptions = autopas::allContainerOptions;
  std::set<autopas::DataLayoutOption> dataLayoutOptions = autopas::allDataLayoutOptions;
  autopas::SelectorStrategyOption selectorStrategy = autopas::SelectorStrategyOption::fastestAbs;
  std::set<autopas::TraversalOption> traversalOptions = autopas::allTraversalOptions;
  autopas::TuningStrategyOption tuningStrategyOption = autopas::TuningStrategyOption::fullSearch;
  std::set<autopas::Newton3Option> newton3Options = autopas::allNewton3Options;
  std::shared_ptr<autopas::NumberSet<double>> cellSizeFactors =
      std::make_shared<autopas::NumberSetFinite<double>>(std::set<double>{1.});
  spdlog::level::level_enum logLevel = spdlog::level::info;
  unsigned int tuningInterval = 100;
  unsigned int tuningSamples = 3;
  unsigned int tuningMaxEvidence = 10;
  std::string writeVTK = "";
  std::string logFileName = "";
  unsigned int verletRebuildFrequency = 5;
  double verletSkinRadius = .2;
  std::array<double,3> BoxMin={0.,0.,0.};
  std::array<double,3> BoxMax={10.,10.,10.};

  // Simulation Options:
  double cutoff = 1.;
  FunctorOption functorOption = FunctorOption::lj12_6;
  size_t iterations = 10;
  bool measureFlops = true;
  double epsilon = 5.0;
  double sigma = 1.0;
  double delta_t = 0.001;
  double mass = 1.0;

  // Object Generation:
  std::vector<CubeGrid> CubeGridObjects = {CubeGrid({10,10,10},1.,{0.,0.,0.},{5.,5.,5.})};
  std::vector<CubeGauss> CubeGaussObjects = {};
  std::vector<CubeUniform> CubeUniformObjects = {};
  std::vector<Sphere> SphereObjects = {};
};