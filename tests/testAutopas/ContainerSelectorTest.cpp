/**
 * ContainerSelectorTest.cpp
 *
 *  Created on: 6/22/18
 *     Aauthor: F. Gratl
 */

#include "ContainerSelectorTest.h"

TEST(ContainerSelectorTest, testGetOptimalContainerOneOption) {

  std::vector<autopas::ContainerOptions> optionVectorDir = {autopas::ContainerOptions::directSum};
  std::vector<autopas::ContainerOptions> optionVectorLC = {autopas::ContainerOptions::linkedCells};
  std::vector<autopas::ContainerOptions> optionVectorVerlet = {autopas::ContainerOptions::verletLists};

  std::vector<autopas::TraversalOptions> traversals = {autopas::TraversalOptions::c08};

  std::array<double, 3> bBoxMin = {0, 0, 0},
      bBoxMax = {10, 10, 10};
  autopas::ContainerSelector<Particle, FPCell> containerSelectorDir(bBoxMin, bBoxMax, 1, optionVectorDir, traversals);
  autopas::ContainerSelector<Particle, FPCell> containerSelectorLC(bBoxMin, bBoxMax, 1, optionVectorLC, traversals);
  autopas::ContainerSelector<Particle, FPCell> containerSelectorVerlet(bBoxMin, bBoxMax, 1, optionVectorVerlet, traversals);

  auto contaienrDir = containerSelectorDir.getOptimalContainer();
  auto contaienrLC = containerSelectorLC.getOptimalContainer();
  auto contaienrVerlet = containerSelectorVerlet.getOptimalContainer();

  ASSERT_TRUE((dynamic_cast<autopas::DirectSum<Particle, FPCell>*>(contaienrDir.get())));
  ASSERT_TRUE((dynamic_cast<autopas::LinkedCells<Particle, FPCell>*>(contaienrLC.get())));
  ASSERT_TRUE((dynamic_cast<autopas::VerletLists<Particle, FPCell>*>(contaienrVerlet.get())));
}