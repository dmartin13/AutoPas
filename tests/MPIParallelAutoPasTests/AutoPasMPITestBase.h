/**
 * @file AutoPasMPITestBase.h
 * @author W. Thieme
 * @date 05/01/20
 */

#include <gtest/gtest.h>

#include "autopas/utils/Logger.h"

#pragma once

class AutoPasMPITestBase : public testing::Test {
 public:
  AutoPasMPITestBase() { autopas::Logger::create(); }

  virtual ~AutoPasMPITestBase() { autopas::Logger::unregister(); }
};
