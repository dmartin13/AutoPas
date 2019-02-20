/**
 * @file CudaExceptionHandler.h
 * @author jspahl
 * @date 2/10/18
 */

#pragma once

#include "autopas/utils/ExceptionHandler.h"
#if defined(AUTOPAS_CUDA)
#include "cuda_runtime.h"
#endif

namespace autopas {
namespace utils {

class CudaExceptionHandler {
 public:
  CudaExceptionHandler() = delete;

#if defined(AUTOPAS_CUDA)
  static void checkErrorCode(cudaError_t error) {
    if (error == cudaSuccess) return;
    std::string errorname = std::string(cudaGetErrorName(error));
    std::string errorstring = std::string(cudaGetErrorString(error));

    autopas::utils::ExceptionHandler::exception(std::string("cuda error: ") + errorname);
  }

  static void checkLastCudaCall() {
    cudaError error = cudaGetLastError();
    if (error == cudaSuccess) return;
    std::string errorname = std::string(cudaGetErrorName(error));
    std::string errorstring = std::string(cudaGetErrorString(error));

    autopas::utils::ExceptionHandler::exception(std::string("cuda error: ") + errorname);
  }
#endif
};

}  // namespace utils
}  // namespace autopas
