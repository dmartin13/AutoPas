/**
 * @file LJFunctorTest.h
 * @author seckler
 * @date 06.11.18
 */

#pragma once

#include <gtest/gtest.h>

#include "AutoPasTestBase.h"
#include "autopas/molecularDynamics/LJFunctor.h"
#include "autopas/molecularDynamics/ParticlePropertiesLibrary.h"
#include "autopasTools/generators/RandomGenerator.h"

class LJFunctorTest : public AutoPasTestBase {
 public:
  LJFunctorTest() : AutoPasTestBase() {
    cutoff = 1.;
    epsilon = 1.;
    sigma = 1.;
    epsilon2 = 2.;
    sigma2 = 2.;
    shift = 0.1;
    lowCorner = {0., 0., 0.};
    highCorner = {5., 5., 5.};
    expectedForce = {-4547248.8989645941, -9094497.7979291882, -13641746.696893783};
    expectedForceMixing = {-835415983.7676939964294, -1670831967.5353879928588, -2506247951.3030819892883};
    expectedEnergy = 3178701.6514326506 / 6.;
    expectedVirial = 6366148.4585504318;
    absDelta = 1e-7;
  }

  void SetUp() override{};

  void TearDown() override{};

 protected:
  template <bool Mixing>
  void testAoSNoGlobals(bool newton3);

  enum InteractionType { own, pair, verlet };

  template <bool Mixing>
  void testSoANoGlobals(bool newton3, InteractionType interactionType);

  enum where_type { inside, boundary, outside };
  void testAoSGlobals(where_type where, bool newton3, bool duplicatedCalculation);
  void testSoAGlobals(where_type where, bool newton3, bool duplicatedCalculation, InteractionType interactionType,
                      size_t additionalParticlesToVerletNumber);

  double cutoff;
  double epsilon;
  double sigma;
  double epsilon2;
  double sigma2;
  double shift;
  std::array<double, 3> lowCorner;
  std::array<double, 3> highCorner;

  std::array<double, 3> expectedForce;
  std::array<double, 3> expectedForceMixing;

  double expectedVirial;
  double expectedEnergy;
  double absDelta;
};
