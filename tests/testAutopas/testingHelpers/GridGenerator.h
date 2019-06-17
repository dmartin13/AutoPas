/**
 * @file GaussianGenerator.h
 * @author F. Gratl
 * @date 5/25/18
 */

#pragma once

#include "autopas/AutoPas.h"

/**
 * Generator for grids of particles.
 */
class GridGenerator {
 public:
  /**
   * fills a cell vector with a cuboid mesh of particles
   * @tparam Particle Type of particle to be generated
   * @tparam ParticleCell
   * @param cells
   * @param particlesPerDim Number of particles per dimension
   * @param spacing Factor for distance between two particles along one
   * dimension (default is 1)
   * @param offset Offset to move all particles
   */
  template <class Particle, class ParticleCell>
  static void fillWithParticles(std::vector<ParticleCell> &cells, std::array<size_t, 3> particlesPerDim,
                                const Particle &defaultParicle = autopas::Particle(),
                                std::array<double, 3> spacing = std::array<double, 3>{1, 1, 1},
                                std::array<double, 3> offset = std::array<double, 3>{.5, .5, .5});

  /**
   * fills a autopas object with a cuboid mesh of particles
   * @tparam Particle Type of particle to be generated
   * @tparam ParticleCell
   * @param autoPas
   * @param particlesPerDim Number of particles per dimension.
   * @param spacing Factor for distance between two particles along one
   * dimension (default is 1).
   * @param offset Offset to move all particles.
   */
  template <class Particle, class ParticleCell>
  static void fillWithParticles(autopas::AutoPas<Particle, ParticleCell> &autoPas,
                                std::array<size_t, 3> particlesPerDim,
                                const Particle &defaultParticle = autopas::Particle(),
                                std::array<double, 3> spacing = std::array<double, 3>{1, 1, 1},
                                std::array<double, 3> offset = std::array<double, 3>{.5, .5, .5});
    /**Fills Autopas Object with a Grid of Particles starting at @param startingPositions
   * @pararm Autopas Object
   * @param startingPositions
   * @param particlesPerDim
   * @param defaultParticle
   * @param spacing
   * @param offset
   *
   * */
   template<class Particle,class ParticleCell>
   static void fillWithParticlesOnR(autopas::AutoPas<Particle,ParticleCell> &autopas,std::array<double,3> startingPositions,std::array<size_t,3> particlesPerDim, const Particle &defaultParticle,
           std::array<double,3> spacing, std::array<double,3> offset);

    <
};

template <class Particle, class ParticleCell>
void GridGenerator::fillWithParticles(std::vector<ParticleCell> &cells, std::array<size_t, 3> particlesPerDim,
                                      const Particle &defaultParicle, std::array<double, 3> spacing,
                                      std::array<double, 3> offset) {
  size_t id = 0;
  size_t cellId = 0;
  for (unsigned int z = 0; z < particlesPerDim[2]; ++z) {
    for (unsigned int y = 0; y < particlesPerDim[1]; ++y) {
      for (unsigned int x = 0; x < particlesPerDim[0]; ++x) {
        Particle p(defaultParicle);
        p.setR({x * spacing[0] + offset[0], y * spacing[1] + offset[1], z * spacing[2] + offset[2]});
        p.setID(id++);
        cells[cellId++].addParticle(p);
      }
    }
  }
}

template <class Particle, class ParticleCell>
void GridGenerator::fillWithParticles(autopas::AutoPas<Particle, ParticleCell> &autoPas,
                                      std::array<size_t, 3> particlesPerDim, const Particle &defaultParticle,
                                      std::array<double, 3> spacing, std::array<double, 3> offset) {
  size_t id = 0;
  for (unsigned int z = 0; z < particlesPerDim[2]; ++z) {
    for (unsigned int y = 0; y < particlesPerDim[1]; ++y) {
      for (unsigned int x = 0; x < particlesPerDim[0]; ++x) {
        Particle p(defaultParticle);
        p.setR({x * spacing[0] + offset[0], y * spacing[1] + offset[1], z * spacing[2] + offset[2]});
        p.setID(id++);
        autoPas.addParticle(p);
      }
    }
  }
}

template<class Particle,class ParticleCell>
void GridGenerator::fillWithParticlesOnR(autopas::AutoPas<Particle,ParticleCell> &autopas,
                                        std::array<double,3> startingPositions,std::array<size_t,3> particlesPerDim, const Particle &defaultParticle,
                                        std::array<double,3> spacing,std::array<double,3> offset){
size_t id = 0;
double S_x= startingPositions[0];
double S_y= startingPositions[1];
double S_z= startingPositions[2];

for (unsigned int z = 0; z < particlesPerDim[2]; ++z) {
for (unsigned int y = 0; y < particlesPerDim[1]; ++y) {
for (unsigned int x = 0; x < particlesPerDim[0]; ++x) {
Particle p(defaultParticle);
p.setR({S_x * spacing[0] + offset[0], S_y * spacing[1] + offset[1], S_z * spacing[2] + offset[2]});
p.setID(id++);
autopas.addParticle(p);
}
}
}

}