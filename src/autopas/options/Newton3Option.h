/**
 * @file Newton3Option.h
 * @author F. Gratl
 * @date 01/02/2019
 */

#pragma once

#include <vector>

namespace autopas {
/**
 * Possible choices for the particle data layout.
 */
enum Newton3Option { enabled, disabled };

/**
 * Provides a way to iterate over the possible choices of data layouts.
 */
static const std::vector<Newton3Option> allNewton3Options = {
    Newton3Option::enabled,
    Newton3Option::disabled,
};

}  // namespace autopas
