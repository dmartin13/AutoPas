/**
 * @file Sphere.h
 * @author N. Fottner
 * @date 29/10/19
 */
#pragma once

#include "autopas/utils/ArrayMath.h"
#include "Object.h"
#include "src/ParticleAttributes.h"

/**
 * Class describing a regular 3D spherical particle grid object.
 */
class Sphere : public Object {
 public:
  /**
   * Constructor.
   * @param velocity
   * @param typeId
   * @param epsilon
   * @param sigma
   * @param mass
   * @param center
   * @param radius
   * @param particleSpacing
   */
  Sphere(const std::array<double, 3> &velocity, unsigned long typeId, double epsilon, double sigma, double mass,
         const std::array<double, 3> &center, int radius, double particleSpacing)
      : Object(velocity, typeId, epsilon, sigma, mass),
        _center(center),
        _radius(radius),
        _particleSpacing(particleSpacing) {}

  /**
   * Getter for center of Sphere
   * @return center
   */
  [[nodiscard]] const std::array<double, 3> &getCenter() const { return _center; }

  /**
   * Getter for radius in number of Particles of Sphere
   * @return radius
   */
  [[nodiscard]] int getRadius() const { return _radius; }

  [[nodiscard]] double getParticleSpacing() const override { return _particleSpacing; }

  /**
   * Call f for every point on the sphere where a particle should be.
   * @param f Function called for every point.
   */
  void iteratePositions(const std::function<void(std::array<double, 3>)> &f) const {
		std::cout << "Radius: " << _radius << std::endl;
    // generate regular grid for 1/8th of the sphere
    for (int z = 0; z <= _radius; ++z) {
      for (int y = 0; y <= _radius; ++y) {
        for (int x = 0; x <= _radius; ++x) {
          // position relative to the center
          std::array<double, 3> relativePos = {(double)x, (double)y, (double)z};
          // mirror to rest of sphere
          for (int i = -1; i <= 1; i += 2) {
            for (int k = -1; k <= 1; k += 2) {
              for (int l = -1; l <= 1; l += 2) {
                std::array<double, 3> mirrorMultipliers = {(double)i, (double)k, (double)l};
                // position mirrored, scaled and absolute
                std::array<double, 3> posVector = autopas::utils::ArrayMath::add(
                    _center, autopas::utils::ArrayMath::mulScalar(
                                autopas::utils::ArrayMath::mul(relativePos, mirrorMultipliers), _particleSpacing));

                double distFromCentersSquare =
                    autopas::utils::ArrayMath::dot(autopas::utils::ArrayMath::sub(posVector, _center),
                                                   autopas::utils::ArrayMath::sub(posVector, _center));
                const auto r = (_radius + 1) * _particleSpacing;
                const auto rSquare = r * r;
                // since the loops create a cubic grid only apply f for positions inside the sphere
                if (distFromCentersSquare <= rSquare) {
                  f(posVector);
                }
                // avoid duplicates
                if (z == 0) break;
              }
              if (y == 0) break;
            }
            if (x == 0) break;
          }
        }
      }
    }
  }

  [[nodiscard]] size_t getParticlesTotal() const override {
    // this should look different if the generator for spheres changes
    int counter = 0;
    iteratePositions([&](auto pos) { ++counter; });
    return counter;
  }

  [[nodiscard]] std::array<double, 3> getBoxMin() const override {
    return {_center[0] - ((double)_radius) * _particleSpacing, _center[1] - ((double)_radius) * _particleSpacing,
            _center[2] - ((double)_radius) * _particleSpacing};
  }

  [[nodiscard]] std::array<double, 3> getBoxMax() const override {
    return {_center[0] + ((double)_radius) * _particleSpacing, _center[1] + ((double)_radius) * _particleSpacing,
            _center[2] + ((double)_radius) * _particleSpacing};
  }

  [[nodiscard]] std::string to_string() const override {
    std::ostringstream output;

    output << std::setw(_valueOffset) << std::left << "center"
           << ":  " << autopas::utils::ArrayUtils::to_string(_center) << std::endl;
    output << std::setw(_valueOffset) << std::left << "radius"
           << ":  " << _radius << std::endl;
    output << std::setw(_valueOffset) << std::left << "particle-spacing"
           << ":  " << _particleSpacing << std::endl;
    output << Object::to_string();
    return output.str();
  }

  void generate(std::vector<ParticleAttributes> &particles) const override {
    ParticleAttributes particle = getDummyParticle(particles.size());
    iteratePositions([&](auto pos) {
    	particle.position = { pos[0], pos[1], pos[2] };
      particles.push_back(particle);
      particle.id++;
    });
  }

 private:
  /**
   * coordinates of the sphere's center
   */
  std::array<double, 3> _center;

  /**
   * radius of the sphere in number of particles
   */
  int _radius;
  double _particleSpacing;
};
