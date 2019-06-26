/**
 * @file BayesianSearch.h
 * @author Jan Nguyen
 * @date 12.06.19
 */

#pragma once

#include <limits>
#include <map>
#include <set>
#include "GaussianProcess.h"
#include "TuningStrategyInterface.h"
#include "autopas/selectors/ContainerSelector.h"
#include "autopas/selectors/FeatureVector.h"
#include "autopas/utils/ExceptionHandler.h"
#include "autopas/utils/NumberSet.h"
#include "autopas/utils/StringUtils.h"

namespace autopas {

/**
 * Assume that the stochastic distribution of the execution time corresponds
 * to a Gaussian Process. This allows to estimate the 'gain' of testing a given
 * feature next.
 */
class BayesianSearch : public TuningStrategyInterface {
  const size_t featureSpaceDims = 4;
  /**
   * max limit of long as double
   */
  const double longMax = static_cast<double>(std::numeric_limits<long>::max());

 public:
  /**
   * Constructor
   * @param allowedContainerOptions
   * @param allowedTraversalOptions
   * @param allowedDataLayoutOptions
   * @param allowedNewton3Options
   * @param allowedCellSizeFactors
   * @param predAcqFunction acquisition function used for prediction while tuning.
   * @param predNumSamples number of samples used for prediction while tuning.
   * @param maxEvidences stop tuning after given number evidences provided.
   * @param lastAcqFunction acquisition function used for prediction of last tuning step.
   * @param lastNumSamples number of samples used for prediction of last tuning step.
   */
  BayesianSearch(const std::set<ContainerOption> &allowedContainerOptions = allContainerOptions,
                 const NumberSet<double> &allowedCellSizeFactors = NumberInterval<double>(1., 2.),
                 const std::set<TraversalOption> &allowedTraversalOptions = allTraversalOptions,
                 const std::set<DataLayoutOption> &allowedDataLayoutOptions = allDataLayoutOptions,
                 const std::set<Newton3Option> &allowedNewton3Options = allNewton3Options, size_t maxEvidences = 10,
                 AcquisitionFunction predAcqFunction = lcb, size_t predNumSamples = 1000,
                 AcquisitionFunction lastAcqFunction = ucb, size_t lastNumSamples = 1000)
      : _containerOptions(allowedContainerOptions),
        _traversalOptions(allowedTraversalOptions),
        _dataLayoutOptions(allowedDataLayoutOptions),
        _newton3Options(allowedNewton3Options),
        _cellSizeFactors(allowedCellSizeFactors.clone()),
        _traversalContainerMap(),
        _currentConfig(),
        _gp(1., std::vector<double>(featureSpaceDims, 1.), 0.001),
        _maxEvidences(maxEvidences),
        _predAcqFunction(predAcqFunction),
        _predNumSamples(predNumSamples),
        _lastAcqFunction(lastAcqFunction),
        _lastNumSamples(lastNumSamples),
        _rng() {
    /// @todo setting hyperparameters

    if (predNumSamples <= 0 or lastNumSamples <= 0) {
      utils::ExceptionHandler::exception(
          "BayesianSearch: Number of samples used for predictions must be greater than 0!");
    }

    // map each travesal to a container
    for (auto it = _traversalOptions.begin(); it != _traversalOptions.end();) {
      auto compatibleContainerOptions = compatibleTraversals::allCompatibleContainers(*it);
      std::set<ContainerOption> allowedAndCompatible;
      std::set_intersection(allowedContainerOptions.begin(), allowedContainerOptions.end(),
                            compatibleContainerOptions.begin(), compatibleContainerOptions.end(),
                            std::inserter(allowedAndCompatible, allowedAndCompatible.begin()));
      if (allowedAndCompatible.empty()) {
        // no compatible container found
        it = _traversalOptions.erase(it);
      } else {
        _traversalContainerMap[*it] = *allowedAndCompatible.begin();
        ++it;
      }
    }

    tune();
  }

  inline Configuration getCurrentConfiguration() override { return _currentConfig; }

  inline void removeN3Option(Newton3Option badNewton3Option) override;

  inline void addEvidence(long time) override {
    // transform long to a value between -1 and 1
    _gp.addEvidence(FeatureVector(_currentConfig), time / longMax);
  }

  inline void reset() override {
    _gp.clear();
    tune();
  }

  inline bool tune() override;

  inline std::set<ContainerOption> getAllowedContainerOptions() override { return _containerOptions; }

  inline bool searchSpaceIsTrivial() override;

  inline bool searchSpaceIsEmpty() override;

 private:
  /**
   * Generate n samples and predict their corresponding
   * acquisition function. Return the FeatureVector which
   * generates the smallest value.
   * @param n numSamples
   * @param af acquisition function
   */
  inline FeatureVector sampleOptimalFeatureVector(size_t n, AcquisitionFunction af);

  std::set<ContainerOption> _containerOptions;
  std::set<TraversalOption> _traversalOptions;
  std::set<DataLayoutOption> _dataLayoutOptions;
  std::set<Newton3Option> _newton3Options;
  std::unique_ptr<NumberSet<double>> _cellSizeFactors;

  std::map<TraversalOption, ContainerOption> _traversalContainerMap;

  Configuration _currentConfig;
  GaussianProcess<FeatureVector> _gp;
  size_t _maxEvidences;
  AcquisitionFunction _predAcqFunction;
  size_t _predNumSamples;
  AcquisitionFunction _lastAcqFunction;
  size_t _lastNumSamples;
  Random _rng;
};

bool BayesianSearch::tune() {
  if (searchSpaceIsEmpty()) {
    // no valid configuration
    _currentConfig = Configuration();
    return false;
  }

  if (_gp.numEvidences() >= _maxEvidences) {
    // select predicted best config
    _currentConfig = sampleOptimalFeatureVector(_lastNumSamples, _lastAcqFunction);
    AutoPasLog(debug, "Selected Configuration {}", _currentConfig.toString());
    return false;
  }

  // select predicted best config for tuning
  _currentConfig = sampleOptimalFeatureVector(_predNumSamples, _predAcqFunction);
  return true;
}

FeatureVector BayesianSearch::sampleOptimalFeatureVector(size_t n, AcquisitionFunction af) {
  // create n lhs samples
  std::vector<FeatureVector> samples = FeatureVector::lhsSampleFeatures(n, _rng, *_cellSizeFactors, _traversalOptions,
                                                                        _dataLayoutOptions, _newton3Options);

  // sample minimum of acquisition function
  auto best = _gp.sampleAquisitionMin(af, samples);

  // choose first compatible container
  best.container = _traversalContainerMap[best.traversal];
  return best;
}

bool BayesianSearch::searchSpaceIsTrivial() {
  if (searchSpaceIsEmpty()) {
    return false;
  }

  return _containerOptions.size() == 1 and (_cellSizeFactors->isFinite() && _cellSizeFactors->size() == 1) and
         _traversalOptions.size() == 1 and _dataLayoutOptions.size() == 1 and _newton3Options.size() == 1;
}

bool BayesianSearch::searchSpaceIsEmpty() {
  // if one enum is empty return true
  return _containerOptions.empty() or (_cellSizeFactors->isFinite() && _cellSizeFactors->size() == 0) or
         _traversalOptions.empty() or _dataLayoutOptions.empty() or _newton3Options.empty();
}

void BayesianSearch::removeN3Option(Newton3Option badNewton3Option) {
  _newton3Options.erase(badNewton3Option);

  if (this->searchSpaceIsEmpty()) {
    utils::ExceptionHandler::exception(
        "Removing all configurations with Newton 3 {} caused the search space to be empty!", badNewton3Option);
  }
}

}  // namespace autopas
