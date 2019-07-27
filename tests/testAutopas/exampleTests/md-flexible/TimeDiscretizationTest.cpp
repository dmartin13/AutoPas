//
// Created by nicola on 22.05.19.
//
#include "TimeDiscretizationTest.h"

void TimeDiscretizationTest::globalForceTest(
    autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>> &auto1,
    autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>> &auto2, int iterations) {
  auto1.iteratePairwise(&functor);
  auto2.iteratePairwise(&functor);
  double particleD = 0.01;
  TimeDiscretization<decltype(auto1)> td1(particleD);
  // to compare OldForce entry of auto2 Particles with Force entries of auto1:
  td1.VSCalculateX(auto2);
  auto2.iteratePairwise(&functor);
  ASSERT_EQ(auto1.getNumberOfParticles(), auto2.getNumberOfParticles());
  for (int i = 0; i < iterations; i++) {
    auto iter1 = auto1.begin();
    auto iter2 = auto2.begin();
    for (unsigned long i = 0; i < auto1.getNumberOfParticles(); i++) {
      EXPECT_EQ(iter1->getF(), iter2->getOldf());
      ++iter1;
      ++iter2;
    }
    td1.VSCalculateX(auto1);
    auto1.iteratePairwise(&functor);
    td1.VSCalculateX(auto2);
    auto2.iteratePairwise(&functor);
  }
}

void TimeDiscretizationTest::initFillWithParticles(
    autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>> &autopas,
    std::array<unsigned long, 3> particlesPerDim) {
  autopas.setBoxMin(boxmin);
  autopas.setBoxMax(boxmax);
  autopas.init();
  PrintableMolecule dummy;
  GridGenerator::fillWithParticles(autopas, particlesPerDim, dummy, {1, 1, 1}, {0., 0., 0.});
}

std::array<double, 3> TimeDiscretizationTest::nextPosition(std::array<double, 3> position, std::array<double, 3> force,
                                                           std::array<double, 3> velocity, double particle_delta_t) {
  auto m = 1.0;  // mass is =1 for all particles in test scope
  velocity = autopas::ArrayMath::mulScalar(velocity, particle_delta_t);
  force = autopas::ArrayMath::mulScalar(force, (particle_delta_t * particle_delta_t / (2 * m)));
  auto newR = autopas::ArrayMath::add(velocity, force);
  return autopas::ArrayMath::add(position, newR);
}

std::array<double, 3> TimeDiscretizationTest::nextVelocity(std::array<double, 3> velocity, std::array<double, 3> force,
                                                           std::array<double, 3> oldf, double particle_delta_t) {
  auto m = 1.0;
  auto newV = autopas::ArrayMath::mulScalar((autopas::ArrayMath::add(force, oldf)), particle_delta_t / (2 * m));
  return autopas::ArrayMath::add(velocity, newV);
}

void TimeDiscretizationTest::Pos_and_Velo_Test(
    autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>> &autopas,
    size_t numberOfParticles, int iterations) {
  // initialize the domain:
  PrintableMolecule::setMass(1.0);
  PrintableMolecule dummy;
  autopas.setBoxMin(boxmin);
  autopas.setBoxMax(boxmax);
  autopas.init();
  RandomGenerator::fillWithParticles(autopas, dummy, numberOfParticles, {0., 0., 0.});
  double particleD = 0.01;
  TimeDiscretization<decltype(autopas)> td1(particleD);
  // initialize force and oldforce values:
  autopas.iteratePairwise(&functor);
  td1.VSCalculateX(autopas);

  // comparing Position and Velocities values calculated in TimeDiscretization Class with calculated value using
  // nextPosition and nextVelocity that implement the same formula used in the TimeD class.
  std::vector<std::array<double, 3>> oldVelocityValues;
  std::vector<std::array<double, 3>> oldPositionValues;
  std::vector<std::array<double, 3>> forces;
  std::vector<std::array<double, 3>> oldforces;
  int i = 0;
  // testing for each time Step: forces are static
  // for each time step values used in the formula must be reset
  for (int currentIteration = 0; i < iterations; currentIteration++) {
    oldVelocityValues.clear();
    oldPositionValues.clear();
    forces.clear();
    oldforces.clear();
    i = 0;
    ASSERT_EQ(oldVelocityValues.size(), 0);
    ASSERT_EQ(oldPositionValues.size(), 0);
    ASSERT_EQ(forces.size(), 0);
    ASSERT_EQ(oldforces.size(), 0);
    // filling vectors with values in timeStep i
#ifdef AUTOPAS_OPENMP
#pragma omp parallel
#endif
    for (auto iter = autopas.begin(); iter.isValid(); ++iter) {
      oldVelocityValues.emplace_back(iter->getV());
      oldPositionValues.emplace_back(iter->getR());
      forces.emplace_back(iter->getF());
      oldforces.emplace_back(iter->getOldf());
    }
    td1.VSCalculateX(autopas);
    td1.VSCalculateV(autopas);
    ASSERT_EQ(oldPositionValues.size(), autopas.getNumberOfParticles());
    ASSERT_EQ(oldVelocityValues.size(), autopas.getNumberOfParticles());
    ASSERT_EQ(forces.size(), autopas.getNumberOfParticles());
    ASSERT_EQ(oldforces.size(), autopas.getNumberOfParticles());
    // checking for equality of values calculated for timeStep i+1
#ifdef AUTOPAS_OPENMP
#pragma omp parallel
#endif
    for (auto iter = autopas.begin(); iter.isValid(); ++iter) {
      ASSERT_EQ(iter->getID(), i);
      EXPECT_EQ(iter->getV(), nextVelocity(oldVelocityValues.at(i), forces.at(i), oldforces.at(i), particleD));
      EXPECT_EQ(iter->getR(), nextPosition(oldPositionValues.at(i), forces.at(i), oldVelocityValues.at(i), particleD));
      i++;
    }
  }
}

TEST_F(TimeDiscretizationTest, GlobalForce) {
  PrintableMolecule::setMass(1.);
  auto auto1a = autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>>();
  auto auto1b = autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>>();
  auto auto2a = autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>>();
  auto auto2b = autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>>();
  std::array<unsigned long, 3> eightParticles = {2, 2, 2};
  std::array<unsigned long, 3> hundred_twenty_fiveParticles = {5, 5, 5};
  initFillWithParticles(auto1a, eightParticles);
  initFillWithParticles(auto1b, eightParticles);
  //    globalForceTest(auto1a,auto1b,10);
  //    globalForceTest(auto1a,auto1b,20);
  globalForceTest(auto1a, auto1b, 30);
  initFillWithParticles(auto2a, hundred_twenty_fiveParticles);
  initFillWithParticles(auto2b, hundred_twenty_fiveParticles);
  //    globalForceTest(auto2a,auto2b,10);
  //    globalForceTest(auto2a,auto2b,20);
  globalForceTest(auto2a, auto2b, 30);
}

TEST_F(TimeDiscretizationTest, PositionsAndVelocity) {
  PrintableMolecule::setMass(1.);
  auto autopas = autopas::AutoPas<PrintableMolecule, autopas::FullParticleCell<PrintableMolecule>>();
  Pos_and_Velo_Test(autopas, 25, 1);
  Pos_and_Velo_Test(autopas, 25, 10);
  Pos_and_Velo_Test(autopas, 100, 1);
  Pos_and_Velo_Test(autopas, 100, 10);
}