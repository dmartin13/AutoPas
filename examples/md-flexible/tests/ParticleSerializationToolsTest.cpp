/**
 * @file ParticleSerializationToolsTest.cpp
 * @author J. Körner
 * @date 13.05.2021
 */
#include "ParticleSerializationToolsTest.h"

#include <gtest/gtest.h>

#include "src/ParticleSerializationTools.h"
#include "testingHelpers/commonTypedefs.h"

ParticleSerializationToolsTest::ParticleSerializationToolsTest() : AutoPasTestBase() {
  _molecule.setR({5.0, 6.0, 7.0});
  _molecule.setV({10.0, 20.0, 30.0});
  _molecule.setF({0.05, 0.06, 0.07});
  _molecule.setID(12);
  _molecule.setOwnershipState(autopas::OwnershipState::halo);
  _molecule.setTypeId(5);
  _molecule.setOldF({0.01, 0.02, 0.03});
#if MD_FLEXIBLE_MODE == MULTISITE
  _molecule.setQ({0.2, 0.3, 0.4, 0.5});
  _molecule.setAngularVel({-1., 2., -3.});
  _molecule.setTorque({0.2, -0.5, 0.8});
#endif
}

TEST_F(ParticleSerializationToolsTest, testSeralizeAndDeserializeParticle) {
  std::vector<char> serializedParticle;
  ParticleSerializationTools::serializeParticle(_molecule, serializedParticle);

  ParticleType deserializedParticle;
  ParticleSerializationTools::deserializeParticle(&serializedParticle[0], deserializedParticle);

  EXPECT_EQ(deserializedParticle.getR(), _molecule.getR());
  EXPECT_EQ(deserializedParticle.getV(), _molecule.getV());
  EXPECT_EQ(deserializedParticle.getF(), _molecule.getF());
  EXPECT_EQ(deserializedParticle.getOldF(), _molecule.getOldF());
  EXPECT_EQ(deserializedParticle.getID(), _molecule.getID());
  EXPECT_EQ(deserializedParticle.getTypeId(), _molecule.getTypeId());
#if MD_FLEXIBLE_MODE == MULTISITE
  EXPECT_EQ(deserializedParticle.getQ(), _molecule.getQ());
  EXPECT_EQ(deserializedParticle.getAngularVel(), _molecule.getAngularVel());
  EXPECT_EQ(deserializedParticle.getTorque(), _molecule.getTorque());
#endif
}
