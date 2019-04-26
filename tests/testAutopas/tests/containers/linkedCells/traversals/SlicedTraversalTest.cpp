/**
 * @file SlicedTraversalTest.cpp
 * @author F. Gratl
 * @date 01.05.18
 */

#include "SlicedTraversalTest.h"
#include "testingHelpers/GridGenerator.h"
#include "testingHelpers/NumThreadGuard.h"

using ::testing::_;
using ::testing::AtLeast;

void testSlicedTraversal(const std::array<size_t, 3>& edgeLength, unsigned long overlap = 1ul) {
  MFunctor functor;
  std::vector<FPCell> cells;
  cells.resize(edgeLength[0] * edgeLength[1] * edgeLength[2]);

  GridGenerator::fillWithParticles<autopas::Particle>(cells, edgeLength);

  NumThreadGuard(4);

  autopas::SlicedTraversal<FPCell, MFunctor, false, true> slicedTraversal(edgeLength, &functor, overlap);

  // every particle interacts with 13 others. Last layer of each dim is covered
  // by previous interactions
  EXPECT_CALL(functor, AoSFunctor(_, _, true))
      .Times((edgeLength[0] - 1) * (edgeLength[1] - 1) * (edgeLength[2] - 1) * 13);
  slicedTraversal.traverseCellPairs(cells);
}

TEST_F(SlicedTraversalTest, testTraversalCube) {
  std::array<size_t, 3> edgeLength = {10, 10, 10};
  testSlicedTraversal(edgeLength);
}

TEST_F(SlicedTraversalTest, DISABLED_testTraversalCubeOverlap2) {
  std::array<size_t, 3> edgeLength = {10, 10, 10};
  testSlicedTraversal(edgeLength, 2);
}

/**
 * This test is the same as testTraversalCube except that the domain is too small for 4 threads.
 * It expects the sliced traversal to start less threads but still work.
 */
TEST_F(SlicedTraversalTest, testTraversalCubeShrink) {
  std::array<size_t, 3> edgeLength = {3, 3, 3};
  testSlicedTraversal(edgeLength);
}

TEST_F(SlicedTraversalTest, testTraversalCuboid) {
  std::array<size_t, 3> edgeLength = {5, 7, 10};
  testSlicedTraversal(edgeLength);
}

TEST_F(SlicedTraversalTest, testIsApplicableTooSmall) {
  NumThreadGuard(4);

  autopas::SlicedTraversal<FPCell, MFunctor, false, true> slicedTraversal({1, 1, 1}, nullptr);

  EXPECT_FALSE(slicedTraversal.isApplicable());
}

TEST_F(SlicedTraversalTest, testIsApplicableShrinkable) {
  NumThreadGuard(4);

  autopas::SlicedTraversal<FPCell, MFunctor, false, true> slicedTraversal({5, 5, 5}, nullptr);

  EXPECT_TRUE(slicedTraversal.isApplicable());
}

TEST_F(SlicedTraversalTest, testIsApplicableOk) {
  NumThreadGuard(4);

  autopas::SlicedTraversal<FPCell, MFunctor, false, true> slicedTraversal({11, 11, 11}, nullptr);

  EXPECT_TRUE(slicedTraversal.isApplicable());
}

TEST_F(SlicedTraversalTest, testIsApplicableOkOnlyOneDim) {
  NumThreadGuard(4);

  autopas::SlicedTraversal<FPCell, MFunctor, false, true> slicedTraversal({1, 1, 11}, nullptr);

  EXPECT_TRUE(slicedTraversal.isApplicable());
}