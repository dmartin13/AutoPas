/**
 * @file OctreeInnerNode.h
 *
 * @author Johannes Spies
 * @date 15.04.2021
 */
#pragma once

#include "autopas/utils/ArrayMath.h"
#include "autopas/utils/inBox.h"
#include "autopas/containers/octree/OctreeNodeInterface.h"

namespace autopas {
    template<class Particle>
    class OctreeInnerNode : public OctreeNodeInterface<Particle> {
    public:
        OctreeInnerNode(std::array<double, 3> boxMin, std::array<double, 3> boxMax,
                        std::array<OctreeNodeInterface<Particle> *, 8> children)
                : OctreeNodeInterface<Particle>(boxMin, boxMax), _children(children) {}

        /**
         * Check if the node's axis aligned bounding box overlaps with the given axis aligned bounding box.
         * @param otherMin The minimum coordinate of the other box
         * @param otherMax The maximum coordinate of the other box
         * @return true iff the overlapping volume is non-negative
         */
        bool overlapsBox(std::array<double, 3> otherMin, std::array<double, 3> otherMax) {
            bool result = true;
            for (auto d = 0; d < 3; ++d) {
                result &= (this->boxMin[d] <= otherMax[d]) && (this->boxMax[d] >= otherMin[d]);
            }
            return result;
        }

        /**
         * @copydoc OctreeNodeInterface::insert()
         */
        OctreeNodeInterface<Particle> *insert(Particle p) override {
            //using namespace autopas::utils;
            assert(this->isInside(p.getR()));

            // Decide in which child the particle should be inserted.
            for (auto childIndex = 0; childIndex < _children.size(); ++childIndex) {
                if (_children[childIndex]->isInside(p.getR())) {
                    auto child = _children[childIndex]->insert(p);

                    // Check if the child changed its node type (maybe was split up?). If yes, deallocate the old child
                    // and put in the new child.
                    if (child != _children[childIndex]) {
                        delete _children[childIndex];
                        _children[childIndex] = child;
                    }

                    break;  // The particle should only be inserted into one box to avoid duplicate force calculations
                }
            }

            return this;
        }

        /**
         * @copydoc OctreeNodeInterface::appendAllParticles()
         */
        void appendAllParticles(std::vector<Particle> &ps) override {
            // An inner node does not contain particles, traverse down to the children.
            for (auto &child : _children) {
                child->appendAllParticles(ps);
            }
        }

        /**
         * @copydoc OctreeNodeInterface::clearChildren()
         */
        OctreeNodeInterface<Particle> *clearChildren() {
            OctreeNodeInterface<Particle> *result = 0;

            // Since there must be a leaf in this tree somewhere (by definition of the octree), this leaf can be reused in
            // order to provide a leaf node as a result when deleting an inner node. All nodes but this leaf are first
            // cleared an then deleted.
            for (auto &child : _children) {
                auto test = child->clearChildren();
                if (!result) {
                    result = test;
                } else {
                    delete test;
                }
            }

            // The inner node is deleted and the leaf node is returned to maintain the tree structure.
            delete this;
            return result;
        }

        /**
         * @copydoc OctreeNodeInterface::getNumParticles()
         */
        unsigned int getNumParticles() override {
            unsigned int result = 0;
            for(auto &child : _children) {
                result += child->getNumParticles();
            }
            return result;
        }

    private:
        /**
         * Each inner node of an octree can contain exactly 8 children.
         */
        std::array<OctreeNodeInterface<Particle> *, 8> _children;
    };
} // namespace autopas
