/**
 * @file IteratorTest.cpp
 * @author seckler
 * @date 22.07.19
 */

#include "IteratorTest.h"

#include "autopas/options/IteratorBehavior.h"
#include "autopas/utils/WrapOpenMP.h"
#include "autopasTools/generators/RandomGenerator.h"
#include "testingHelpers/EmptyFunctor.h"
#include "testingHelpers/TouchableParticle.h"
#include "testingHelpers/commonTypedefs.h"

using ::testing::_;

template <typename AutoPasT>
auto IteratorTest::defaultInit(AutoPasT &autoPas, autopas::ContainerOption &containerOption, double cellSizeFactor) {
  autoPas.setBoxMin({0., 0., 0.});
  autoPas.setBoxMax({10., 10., 10.});
  autoPas.setCutoff(1);
  autoPas.setVerletSkin(0.2);
  autoPas.setVerletRebuildFrequency(2);
  autoPas.setNumSamples(2);
  autoPas.setAllowedContainers(std::set<autopas::ContainerOption>{containerOption});
  autoPas.setAllowedTraversals(autopas::compatibleTraversals::allCompatibleTraversals(containerOption));
  autoPas.setAllowedCellSizeFactors(autopas::NumberSetFinite<double>(std::set<double>({cellSizeFactor})));

#ifdef AUTOPAS_CUDA
  autoPas.setVerletClusterSize(32);
#endif

  autoPas.init();

  auto haloBoxMin =
      autopas::utils::ArrayMath::subScalar(autoPas.getBoxMin(), autoPas.getVerletSkin() + autoPas.getCutoff());
  auto haloBoxMax =
      autopas::utils::ArrayMath::addScalar(autoPas.getBoxMax(), autoPas.getVerletSkin() + autoPas.getCutoff());

  return std::make_tuple(haloBoxMin, haloBoxMax);
}

///**
// * Iterate over all particles, generate a region iterator for each particle that spans a tiny space around them and
// * check if this region iterator finds exactly this particle.
// * @tparam AutoPasT
// * @param autoPas
// * @param behavior
// */
// template <typename AutoPasT>
// void checkRegionIteratorForAllParticles(AutoPasT &autoPas, autopas::IteratorBehavior behavior) {
//  for (auto iter1 = autoPas.begin(behavior); iter1.isValid(); ++iter1) {
//    unsigned int count = 0;
//    auto low = iter1->getR();
//    auto up = autopas::utils::ArrayMath::addScalar(low, 1e-10);
//
//    for (auto iter2 = autoPas.getRegionIterator(low, up, behavior); iter2.isValid(); ++iter2) {
//      ++count;
//      EXPECT_EQ(&(*iter1), &(*iter2)) << "Wrong particle found";
//    }
//    EXPECT_EQ(count, 1u) << "The following particle was not found exactly once:\n" << iter1->toString();
//  }
//}

template <class AutoPasT>
auto IteratorTest::fillContainerAroundBoundary(AutoPasT &autoPas) {
  constexpr size_t numParticles1dTotal = 10;

  auto cutoff = autoPas.getCutoff();
  auto skin = autoPas.getVerletSkin();

  // generator function for critical coordinates (along  one dimension)
  auto generateInteresting1DPositions = [&](double min, double max) -> auto {
    // ensure that all particles are at most skin away from halo!
    // interesting cases are:
    //   - outside of the halo by skin
    //   - edge of halo
    //   - in the halo
    //   - edge of actual domain
    //   - just inside the domain
    return std::array<double, numParticles1dTotal>{min - cutoff - skin + 1e-10,
                                                   min - cutoff,
                                                   min - skin / 4,
                                                   min,
                                                   min + skin / 4,
                                                   max - skin / 4,
                                                   max,
                                                   max + skin / 4,
                                                   max + cutoff,
                                                   max + cutoff + skin - 1e-10};
  };

  // fill container
  size_t id = 0;
  auto boxMin = autoPas.getBoxMin();
  auto boxMax = autoPas.getBoxMax();

  std::vector<size_t> particleIDsHalo;
  std::vector<size_t> particleIDsOwned;
  for (auto x : generateInteresting1DPositions(boxMin[0], boxMax[0])) {
    for (auto y : generateInteresting1DPositions(boxMin[1], boxMax[1])) {
      for (auto z : generateInteresting1DPositions(boxMin[2], boxMax[2])) {
        std::array<double, 3> pos{x, y, z};
        Molecule p(pos, {0., 0., 0.}, id++, 0);
        // add the particle as actual or halo particle
        if (autopas::utils::inBox(pos, boxMin, boxMax)) {
          autoPas.addParticle(p);
          particleIDsOwned.push_back(p.getID());
        } else {
          // AutoPas should set the ownership state of this particle to halo
          autoPas.addOrUpdateHaloParticle(p);
          particleIDsHalo.push_back(p.getID());
        }
      }
    }
  }

  // sanity check. Can not use assert because this introduces a different return.
  EXPECT_EQ(particleIDsOwned.size() + particleIDsHalo.size(),
            numParticles1dTotal * numParticles1dTotal * numParticles1dTotal);
  // getNumberOfParticles works via counters in the logic handler
  EXPECT_EQ(autoPas.getNumberOfParticles(autopas::IteratorBehavior::ownedOnly), particleIDsOwned.size());
  EXPECT_EQ(autoPas.getNumberOfParticles(autopas::IteratorBehavior::haloOnly), particleIDsHalo.size());
  return std::make_tuple(particleIDsOwned, particleIDsHalo);
}

///**
// * Tests the addition and iteration over particles.
// * Strategically places 10 particles around every corner of the domain
// * @param containerOption
// */
// template <bool testConstIterators>
// void IteratorTest::testAdditionAndIteration(autopas::ContainerOption containerOption, double cellSizeOption,
//                                            bool priorForceCalc) {
//  // create AutoPas object
//  autopas::AutoPas<Molecule> autoPas;
//
//  // Reference to the AutoPas object to be able to check const iterators.
//  std::conditional_t<testConstIterators, const autopas::AutoPas<Molecule> &, autopas::AutoPas<Molecule> &> autoPasRef
//  =
//      autoPas;
//
//  autoPas.setAllowedContainers(std::set<autopas::ContainerOption>{containerOption});
//  autoPas.setAllowedTraversals(autopas::compatibleTraversals::allCompatibleTraversals(containerOption));
//  autoPas.setAllowedCellSizeFactors(autopas::NumberSetFinite<double>(std::set<double>({cellSizeOption})));
//
//  auto [haloBoxMin, haloBoxMax] = defaultInit(autoPas, containerOption, cellSizeOption);
//
//  auto [numParticlesOwned, numParticlesTotal] = fillContainerAroundBoundary(autoPas);
//
//  if (priorForceCalc) {
//    // the prior force calculation is partially wanted as this sometimes changes the state of the internal containers.
//    EmptyFunctor<Molecule> eFunctor;
//    autoPas.iteratePairwise(&eFunctor);
//  }
//  {
//    size_t count = 0;
//    for (auto iter = autoPasRef.begin(autopas::IteratorBehavior::ownedOnly); iter.isValid(); ++iter) {
//      ++count;
//      EXPECT_TRUE(iter->isOwned()) << "Iterator: Found a particle that is not owned while looking only for owned
//      ones!";
//    }
//
//    EXPECT_EQ(count, numParticlesOwned) << "Iterator: Found incorrect number of owned particles!";
//  }
//
//  // check number of halo particles
//  {
//    size_t count = 0;
//    for (auto iter = autoPasRef.begin(autopas::IteratorBehavior::haloOnly); iter.isValid(); ++iter) {
//      ++count;
//      EXPECT_FALSE(iter->isOwned())
//          << "Iterator: Found a particle that is owned while looking only for not owned ones!";
//    }
//
//    EXPECT_EQ(count, numParticlesTotal - numParticlesOwned)
//        << "Iterator: Found incorrect number of halo only particles!";
//  }
//
//  // check number of particles
//  {
//    size_t count = 0;
//    for (auto iter = autoPasRef.begin(autopas::IteratorBehavior::haloAndOwned); iter.isValid(); ++iter) {
//      ++count;
//    }
//    EXPECT_EQ(count, numParticlesTotal) << "Iterator: Found incorrect number of halo + owned particles!";
//  }
//
//  // check number of halo particles for region iterator
//  {
//    size_t count = 0;
//    for (auto iter = autoPasRef.getRegionIterator(haloBoxMin, haloBoxMax, autopas::IteratorBehavior::haloOnly);
//         iter.isValid(); ++iter) {
//      ++count;
//      EXPECT_FALSE(iter->isOwned())
//          << "RegionIterator: Found a particle that is owned while looking only for not owned ones!";
//    }
//
//    EXPECT_EQ(count, numParticlesTotal - numParticlesOwned)
//        << "RegionIterator: Found incorrect number of halo only particles!";
//  }
//
//  // check number of particles for region iterator
//  {
//    size_t count = 0;
//    for (auto iter = autoPasRef.getRegionIterator(haloBoxMin, haloBoxMax, autopas::IteratorBehavior::haloAndOwned);
//         iter.isValid(); ++iter) {
//      ++count;
//    }
//    EXPECT_EQ(count, numParticlesTotal) << "RegionIterator: Found incorrect number of halo + owned particles!";
//  }
//
//  // check number of owned particles for region iterator
//  {
//    size_t count = 0;
//    for (auto iter = autoPasRef.getRegionIterator(haloBoxMin, haloBoxMax, autopas::IteratorBehavior::ownedOnly);
//         iter.isValid(); ++iter) {
//      ++count;
//      EXPECT_TRUE(iter->isOwned())
//          << "RegionIterator: Found a particle that is not owned while looking only for owned ones!";
//    }
//
//    EXPECT_EQ(count, numParticlesOwned) << "RegionIterator: Found incorrect number of owned particles!";
//  }
//
//  // check all particles are in region iterator of their position, ownedOnly
//  checkRegionIteratorForAllParticles(autoPasRef, autopas::IteratorBehavior::ownedOnly);
//
//  // check all particles are in region iterator of their position, haloOnly
//  checkRegionIteratorForAllParticles(autoPasRef, autopas::IteratorBehavior::haloOnly);
//
//  // check all particles are in region iterator of their position, haloAndOwned
//  checkRegionIteratorForAllParticles(autoPasRef, autopas::IteratorBehavior::haloAndOwned);
//
//  // Now move particles by at most skin/2, as this is the maximal distance they are allowed to move.
//  // Don't use autoPasRef here, as we are modifying things!
//  auto boxMax = autoPas.getBoxMax();
//  auto boxMin = autoPas.getBoxMin();
//  auto skin = autoPas.getVerletSkin();
//  for (auto iter = autoPas.begin(autopas::IteratorBehavior::haloAndOwned); iter.isValid(); ++iter) {
//    auto pos = iter->getR();
//    for (auto d = 0; d < 3; ++d) {
//      if (pos[d] < boxMin[d]) {
//        pos[d] += skin / 2;
//      } else if (pos[d] >= boxMax[d]) {
//        pos[d] -= skin / 2;
//      } else if (pos[d] < (boxMax[d] + boxMin[d]) / 2) {
//        pos[d] -= skin / 2;
//      } else {
//        pos[d] += skin / 2;
//      }
//    }
//    iter->setR(pos);
//  }
//
//  // check all particles are in region iterator of their position, ownedOnly
//  checkRegionIteratorForAllParticles(autoPasRef, autopas::IteratorBehavior::ownedOnly);
//
//  // check all particles are in region iterator of their position, haloOnly
//  checkRegionIteratorForAllParticles(autoPasRef, autopas::IteratorBehavior::haloOnly);
//
//  // check all particles are in region iterator of their position, haloAndOwned
//  checkRegionIteratorForAllParticles(autoPasRef, autopas::IteratorBehavior::haloAndOwned);
//}
//
// template <bool testConstIterators>
// void IteratorTest::testRangeBasedIterator(autopas::ContainerOption containerOption, double cellSizeOption,
//                                          bool priorForceCalc) {
//  // create AutoPas object
//  autopas::AutoPas<Molecule> autoPas;
//  // Reference to the AutoPas object to be able to check const iterators.
//  std::conditional_t<testConstIterators, const autopas::AutoPas<Molecule> &, autopas::AutoPas<Molecule> &> autoPasRef
//  =
//      autoPas;
//
//  autoPas.setAllowedContainers(std::set<autopas::ContainerOption>{containerOption});
//  autoPas.setAllowedTraversals(autopas::compatibleTraversals::allCompatibleTraversals(containerOption));
//  autoPas.setAllowedCellSizeFactors(autopas::NumberSetFinite<double>(std::set<double>({cellSizeOption})));
//
//  auto [haloBoxMin, haloBoxMax] = defaultInit(autoPas, containerOption, cellSizeOption);
//
//  auto [numParticlesOwned, numParticlesTotal] = fillContainerAroundBoundary(autoPas);
//
//  if (priorForceCalc) {
//    // the prior force calculation is partially wanted as this sometimes changes the state of the internal containers.
//    EmptyFunctor<Molecule> eFunctor;
//    autoPas.iteratePairwise(&eFunctor);
//  }
//
//  for (Particle &particle : autoPas) {
//    particle.setF({42., 42., 42.});
//  }
//
//  for (auto &particle : autoPasRef) {
//    decltype(particle.getF()) comparison = {42., 42., 42};
//    ASSERT_EQ(particle.getF(), comparison);
//  }
//
//  for (auto iter = autoPasRef.begin(); iter.isValid(); ++iter) {
//    decltype(iter->getF()) comparison = {42., 42., 42};
//    ASSERT_EQ(iter->getF(), comparison);
//  }
//}

/////////////////////////////////// NEW TESTS ///////////////////////////////////

template <class AutoPasT, class F>
void IteratorTest::applyIterator(bool useRegionIterator, bool useConstIterator, autopas::IteratorBehavior behavior,
                                 AutoPasT &autoPas, F fun) {
  if (useRegionIterator) {
    const auto interactionLength = autoPas.getCutoff() + autoPas.getVerletSkin();
    // halo has width of interactionLength
    const auto haloBoxMin = autopas::utils::ArrayMath::subScalar(autoPas.getBoxMin(), interactionLength);
    const auto haloBoxMax = autopas::utils::ArrayMath::addScalar(autoPas.getBoxMax(), interactionLength);
    if (useConstIterator) {
      const auto &autoPasRef = autoPas;
      typename AutoPasT::const_iterator_t iter = autoPasRef.getRegionIterator(haloBoxMin, haloBoxMax, behavior);
      fun(autoPasRef, iter);
    } else {
      typename AutoPasT::iterator_t iter = autoPas.getRegionIterator(haloBoxMin, haloBoxMax, behavior);
      fun(autoPas, iter);
    }
  } else {
    if (useConstIterator) {
      typename AutoPasT::const_iterator_t iter = autoPas.cbegin(behavior);
      fun(autoPas, iter);
    } else {
      typename AutoPasT::iterator_t iter = autoPas.begin(behavior);
      fun(autoPas, iter);
    }
  }
}

template <class AutoPasT, class IteratorT>
void IteratorTest::findParticles(AutoPasT &autopas, IteratorT &iterator,
                                 const std::vector<size_t> &particleIDsExpected) {
  std::vector<size_t> particleIDsFound;

#ifdef AUTOPAS_OPENMP
#pragma omp parallel  // reduction(vecMerge : particleIDsFound)
#endif
  {
    std::vector<size_t> myParticleIDsFound;
    for (; iterator.isValid(); ++iterator) {
      myParticleIDsFound.push_back(iterator->getID());
    }
#pragma omp critical
    { particleIDsFound.insert(particleIDsFound.end(), myParticleIDsFound.begin(), myParticleIDsFound.end()); }
  }

  // check that everything was found
  EXPECT_THAT(particleIDsFound, ::testing::UnorderedElementsAreArray(particleIDsExpected));
}

TEST_P(IteratorTest, emptyContainer) {
  auto [containerOption, cellSizeFactor, useRegionIterator, useConstIterator, priorForceCalc, behavior] = GetParam();

  // init autopas and fill it with some particles
  autopas::AutoPas<Molecule> autoPas;
  defaultInit(autoPas, containerOption, cellSizeFactor);

  if (priorForceCalc) {
    // the prior force calculation is partially wanted as this sometimes changes the state of the internal containers.
    EmptyFunctor<Molecule> eFunctor;
    autoPas.iteratePairwise(&eFunctor);
  }

  // actual test
  applyIterator(useRegionIterator, useConstIterator, behavior, autoPas,
                [&](const auto &autopas, auto &iter) { findParticles(autoPas, iter, {}); });
}

TEST_P(IteratorTest, findAllParticles) {
  auto [containerOption, cellSizeFactor, useRegionIterator, useConstIterator, priorForceCalc, behavior] = GetParam();

  // init autopas and fill it with some particles
  autopas::AutoPas<Molecule> autoPas;
  defaultInit(autoPas, containerOption, cellSizeFactor);
  auto [particleIDsOwned, particleIDsHalo] = fillContainerAroundBoundary(autoPas);

  if (priorForceCalc) {
    // the prior force calculation is partially wanted as this sometimes changes the state of the internal containers.
    EmptyFunctor<Molecule> eFunctor;
    autoPas.iteratePairwise(&eFunctor);
  }

  // set up expectations
  std::vector<size_t> expectedIDs;
  switch (behavior) {
    case autopas::IteratorBehavior::ownedOnly: {
      expectedIDs = particleIDsOwned;
      break;
    }
    case autopas::IteratorBehavior::haloOnly: {
      expectedIDs = particleIDsHalo;
      break;
    }
    case autopas::IteratorBehavior::haloAndOwned: {
      expectedIDs = particleIDsOwned;
      expectedIDs.insert(expectedIDs.end(), particleIDsHalo.begin(), particleIDsHalo.end());
      break;
    }
    case autopas::IteratorBehavior::haloOwnedAndDummy: {
      GTEST_FAIL() << "IteratorBehavior::haloOwnedAndDummy should not be tested through this test"
                      " as container behavior with dummy particles is not uniform.";
      break;
    }
  }

  // actual test
  applyIterator(useRegionIterator, useConstIterator, behavior, autoPas,
                [&](const auto &autopas, auto &iter) { findParticles(autoPas, iter, expectedIDs); });
}

/////////////////////////////////// OLD TESTS ///////////////////////////////////

// TEST_P(IteratorTest, ParticleAdditionAndIteratorTestNormal) {
//  const auto &[containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testAdditionAndIteration<true>(containerOption, cellSizeFactor, priorForceCalc);
//  } else {
//    testAdditionAndIteration<false>(containerOption, cellSizeFactor, priorForceCalc);
//  }
//}
//
// TEST_P(IteratorTest, RangeBasedIterator) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testRangeBasedIterator<true>(containerOption, cellSizeFactor, priorForceCalc);
//  } else {
//    testRangeBasedIterator<false>(containerOption, cellSizeFactor, priorForceCalc);
//  }
//}
//
///**
// * The main idea of this test is to compare the iterators using openmp with the iterators not using openmp.
// * If OPENMP is disabled, this tests mainly checks that no particle is traversed twice.
// */
// template <bool useConstIterator>
// void IteratorTest::testOpenMPIterators(autopas::ContainerOption containerOption, double cellSizeFactor,
//                                       autopas::IteratorBehavior behavior, bool testRegionIterators,
//                                       bool priorForceCalc) {
//  std::array<double, 3> min = {1, 1, 1};
//  std::array<double, 3> max = {8, 8, 8};
//
//  std::array<double, 3> lowCorner = {0, 0, 0};
//  std::array<double, 3> highCorner = {3, 3, 3};
//
//  autopas::AutoPas<TouchableParticle> apContainer;
//  // Reference to the AutoPas object to be able to check const iterators.
//  std::conditional_t<useConstIterator, const autopas::AutoPas<TouchableParticle> &,
//                     autopas::AutoPas<TouchableParticle> &>
//      autoPasRef = apContainer;
//
//  apContainer.setAllowedContainers({containerOption});
//  apContainer.setAllowedTraversals(autopas::compatibleTraversals::allCompatibleTraversals(containerOption));
//  apContainer.setCellSizeFactor(cellSizeFactor);
//
//  apContainer.setBoxMin(min);
//  apContainer.setBoxMax(max);
//  apContainer.setCutoff(1.);
//  apContainer.setVerletSkin(0.2);
//  apContainer.setVerletClusterSize(64);
//
//  apContainer.init();
//
//  autopasTools::generators::RandomGenerator::fillWithParticles(apContainer, TouchableParticle({0., 0., 0.}, 0),
//                                                               apContainer.getBoxMin(), apContainer.getBoxMax(),
//                                                               500);
//  autopasTools::generators::RandomGenerator::fillWithHaloParticles(
//      apContainer, TouchableParticle({0., 0., 0.}, 0), 1, 50,
//      [](decltype(apContainer) &c, TouchableParticle p) { c.addOrUpdateHaloParticle(p); });
//
//  if (priorForceCalc) {
//    // the prior force calculation is partially wanted as this sometimes changes the state of the internal
//    containers. EmptyFunctor<TouchableParticle> eFunctor; apContainer.iteratePairwise(&eFunctor);
//  }
//
//  int numTouchedOpenMP = 0, numTouchedSerial = 0;
//#ifdef AUTOPAS_OPENMP
//#pragma omp parallel reduction(+ : numTouchedOpenMP)
//#endif
//  {
//    // with OpenMP:
//    auto begin = testRegionIterators ? apContainer.getRegionIterator(lowCorner, highCorner, behavior)
//                                     : apContainer.begin(behavior);
//    for (auto iter = begin; iter.isValid(); ++iter) {
//      iter->touch();
//      ++numTouchedOpenMP;
//      if (behavior == autopas::IteratorBehavior::ownedOnly) {
//        EXPECT_TRUE(iter->isOwned());
//      } else if (behavior == autopas::IteratorBehavior::haloOnly) {
//        EXPECT_FALSE(iter->isOwned());
//      }
//    }
//  }
//  {
//    // without OpenMP:
//    auto begin = testRegionIterators ? autoPasRef.getRegionIterator(lowCorner, highCorner, behavior)
//                                     : autoPasRef.begin(behavior);
//    for (auto iter = begin; iter.isValid(); ++iter) {
//      EXPECT_EQ(1, iter->getNumTouched());
//      ++numTouchedSerial;
//      if (behavior == autopas::IteratorBehavior::ownedOnly) {
//        EXPECT_TRUE(iter->isOwned());
//      } else if (behavior == autopas::IteratorBehavior::haloOnly) {
//        EXPECT_FALSE(iter->isOwned());
//      }
//    }
//  }
//  EXPECT_GT(numTouchedSerial, 0) << "Serial iterator did not touch anything!";
//  EXPECT_EQ(numTouchedSerial, numTouchedOpenMP) << "Iterator found a different number of particles when using
//  OpenMP!";
//}
//
///**
// * Compare the OpenMP iterator behavior for owned only.
// */
// TEST_P(IteratorTest, testOpenMPIteratorsOwnedOnly) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testOpenMPIterators<true>(containerOption, cellSizeFactor, autopas::IteratorBehavior::ownedOnly, false,
//                              priorForceCalc);
//  } else {
//    testOpenMPIterators<false>(containerOption, cellSizeFactor, autopas::IteratorBehavior::ownedOnly, false,
//                               priorForceCalc);
//  }
//}
//
///**
// * Compare the OpenMP iterator behavior for halo and owned particles.
// */
// TEST_P(IteratorTest, testOpenMPIteratorsHaloAndOwned) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testOpenMPIterators<true>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloAndOwned, false,
//                              priorForceCalc);
//  } else {
//    testOpenMPIterators<false>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloAndOwned, false,
//                               priorForceCalc);
//  }
//}
//
///**
// * Compare the OpenMP iterator behavior for halo only.
// */
// TEST_P(IteratorTest, testOpenMPIteratorsHaloOnly) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testOpenMPIterators<true>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloOnly, false,
//                              priorForceCalc);
//  } else {
//    testOpenMPIterators<false>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloOnly, false,
//                               priorForceCalc);
//  }
//}
//
///**
// * Compare the OpenMP RegionIterator behavior for owned only.
// */
// TEST_P(IteratorTest, testOpenMPRegionIteratorsOwnedOnly) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testOpenMPIterators<true>(containerOption, cellSizeFactor, autopas::IteratorBehavior::ownedOnly, true,
//                              priorForceCalc);
//  } else {
//    testOpenMPIterators<false>(containerOption, cellSizeFactor, autopas::IteratorBehavior::ownedOnly, true,
//                               priorForceCalc);
//  }
//}
//
///**
// * Compare the OpenMP RegionIterator behavior for halo and owned particles.
// */
// TEST_P(IteratorTest, testOpenMPRegionIteratorsHaloAndOwned) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testOpenMPIterators<true>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloAndOwned, true,
//                              priorForceCalc);
//  } else {
//    testOpenMPIterators<false>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloAndOwned, true,
//                               priorForceCalc);
//  }
//}
//
///**
// * Compare the OpenMP RegionIterator behavior for halo only.
// */
// TEST_P(IteratorTest, testOpenMPRegionIteratorsHaloOnly) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testOpenMPIterators<true>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloOnly, true,
//                              priorForceCalc);
//  } else {
//    testOpenMPIterators<false>(containerOption, cellSizeFactor, autopas::IteratorBehavior::haloOnly, true,
//                               priorForceCalc);
//  }
//}
//
// template <bool useConstIterator>
// void IteratorTest::testRegionIteratorDeletion(autopas::ContainerOption containerOption, double cellSizeFactor,
//                                              bool priorForceCalc) {
//  // create AutoPas object
//  autopas::AutoPas<Molecule> autoPas;
//  // Reference to the AutoPas object to be able to check const iterators.
//  std::conditional_t<useConstIterator, const autopas::AutoPas<Molecule> &, autopas::AutoPas<Molecule> &>
//  autoPasRef
//  =
//      autoPas;
//
//  autoPas.setAllowedContainers(std::set<autopas::ContainerOption>{containerOption});
//  autoPas.setAllowedTraversals(autopas::compatibleTraversals::allCompatibleTraversals(containerOption));
//  autoPas.setAllowedCellSizeFactors(autopas::NumberSetFinite<double>(std::set<double>({cellSizeFactor})));
//
//  auto [haloBoxMin, haloBoxMax] = defaultInit(autoPas, containerOption, cellSizeFactor);
//  auto boxMin = autoPas.getBoxMin();
//  auto boxMax = autoPas.getBoxMax();
//
//  size_t numParticles = 200;
//
//  srand(42);
//  for (size_t id = 0; id < numParticles; ++id) {
//    auto pos = autopasTools::generators::RandomGenerator::randomPosition(haloBoxMin, haloBoxMax);
//    Molecule p(pos, {0., 0., 0.}, id);
//    // add the particle!
//    if (autopas::utils::inBox(pos, boxMin, boxMax)) {
//      autoPas.addParticle(p);
//    } else {
//      autoPas.addOrUpdateHaloParticle(p);
//    }
//  }
//
//  if (priorForceCalc) {
//    // the prior force calculation is partially wanted as this sometimes changes the state of the internal
//    containers. EmptyFunctor<Molecule> eFunctor; autoPas.iteratePairwise(&eFunctor);
//  }
//
//  std::set<unsigned long> beforeParticles;
//  for (auto &particle : autoPasRef) {
//    beforeParticles.insert(particle.getID());
//  }
//  ASSERT_EQ(beforeParticles.size(), numParticles);
//
//  // delete some particles
//  std::set<unsigned long> deletedParticles;
//  for (auto it = autoPas.getRegionIterator({0., 0., 3.}, {10., 10., 4.5}); it != autoPas.end(); ++it) {
//    deletedParticles.insert(it->getID());
//    autoPas.deleteParticle(it);
//  }
//
//  // calculate the particles that should still be there.
//  std::set<unsigned long> shouldBeParticles;
//  for (auto id : beforeParticles) {
//    if (deletedParticles.find(id) == deletedParticles.end()) {
//      // Has not been deleted!
//      shouldBeParticles.insert(id);
//    }
//  }
//
//  // check whether they are still there.
//  std::vector<unsigned long> afterParticles;
//  for (auto &particle : autoPasRef) {
//    EXPECT_NE(shouldBeParticles.find(particle.getID()), shouldBeParticles.end()) << "id:" << particle.getID();
//    afterParticles.push_back(particle.getID());
//  }
//
//  EXPECT_EQ(afterParticles.size(), shouldBeParticles.size());
//}
//
// TEST_P(IteratorTest, RegionIteratorDeletion) {
//  auto [containerOption, cellSizeFactor, useConstIterator, priorForceCalc] = GetParam();
//  if (useConstIterator) {
//    testRegionIteratorDeletion<true>(containerOption, cellSizeFactor, priorForceCalc);
//  } else {
//    testRegionIteratorDeletion<false>(containerOption, cellSizeFactor, priorForceCalc);
//  }
//}

using ::testing::Combine;
using ::testing::UnorderedElementsAreArray;
using ::testing::Values;
using ::testing::ValuesIn;

static inline auto getTestableContainerOptions() {
#ifdef AUTOPAS_CUDA
  return autopas::ContainerOption::getAllOptions();
#else
  auto containerOptions = autopas::ContainerOption::getAllOptions();
  containerOptions.erase(containerOptions.find(autopas::ContainerOption::verletClusterCells));
  return containerOptions;
#endif
}

static inline auto getIteratorBehaviorOptions() {
  auto allOptions = autopas::IteratorBehavior::getAllOptions();
  std::set<autopas::IteratorBehavior> retSet;
  // we ignore dummy particles in the general tests because they can behave differently depending on the container
  std::set<autopas::IteratorBehavior> ignoredOptions = {autopas::IteratorBehavior::haloOwnedAndDummy};
  std::set_difference(allOptions.begin(), allOptions.end(), ignoredOptions.begin(), ignoredOptions.end(),
                      std::inserter(retSet, retSet.begin()));
  return retSet;
}

INSTANTIATE_TEST_SUITE_P(Generated, IteratorTest,
                         Combine(ValuesIn(getTestableContainerOptions()), Values(0.5, 1., 1.5), Values(true, false),
                                 Values(true, false), Values(true, false), ValuesIn(getIteratorBehaviorOptions())),
                         IteratorTest::PrintToStringParamName());
