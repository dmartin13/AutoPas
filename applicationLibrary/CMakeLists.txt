# --------------------------libraries-------------------------------------------

# --- molecular dynamics ---

file(
        GLOB_RECURSE
        LIB_MD_SRC
        "molecularDynamics/molecularDynamicsLibrary/*.h"
        "molecularDynamics/molecularDynamicsLibrary/*.cpp"
)

add_library(molecularDynamicsLibrary STATIC ${LIB_MD_SRC})

# Set CMake option for md-flexible mode
set(MD_FLEXIBLE_MODE
        "SINGLESITE"
        CACHE
        STRING "Choose the mode of md-flexible to be compiled."
        )
set_property(CACHE MD_FLEXIBLE_MODE PROPERTY STRINGS "SINGLESITE;MULTISITE")

# Define numerical values for different md-flexible modes
target_compile_definitions(molecularDynamicsLibrary PUBLIC SINGLESITE=0)
target_compile_definitions(molecularDynamicsLibrary PUBLIC MULTISITE=1)

string(TOLOWER "${MD_FLEXIBLE_MODE}" MD_FLEXIBLE_MODE_lower)

if (MD_FLEXIBLE_MODE_lower MATCHES  "singlesite")
    target_compile_definitions(molecularDynamicsLibrary PUBLIC MD_FLEXIBLE_MODE=0)
elseif(MD_FLEXIBLE_MODE_lower MATCHES "multisite")
    target_compile_definitions(molecularDynamicsLibrary PUBLIC MD_FLEXIBLE_MODE=1)
endif()

target_link_libraries(molecularDynamicsLibrary PUBLIC autopas)

target_include_directories(molecularDynamicsLibrary PUBLIC ./molecularDynamics)

# --- sph ---

file(
        GLOB_RECURSE
        LIB_SPH_SRC
        "sph/SPHLibrary/*.h"
        "sph/SPHLibrary/*.cpp"
)

add_library(SPHLibrary STATIC ${LIB_SPH_SRC})

target_link_libraries(SPHLibrary PUBLIC autopas)

target_include_directories(SPHLibrary PUBLIC ./sph)

# --------------------------------tests---------------------------------------

# --- molecular dynamics ---

if (AUTOPAS_BUILD_TESTS)
    message(STATUS "applicationLibrary - adding all tests")
    file(
            GLOB_RECURSE
            SS_TEST_SRC
            "molecularDynamics/tests/singleSiteTests/*.cpp"
            "molecularDynamics/tests/singleSiteTests/*.h"
            "molecularDynamics/tests/particlePropertiesLibraryTests/*.cpp"
            "molecularDynamics/tests/particlePropertiesLibraryTests/*.h"
            "molecularDynamics/tests/templateInstatiations/AutoPasInstantiations.cpp"
    )

    add_executable(mdLibSingleSiteTests ${SS_TEST_SRC} mdLibTests.cpp)

    target_compile_definitions(
            mdLibSingleSiteTests PRIVATE
            MD_FLEXIBLE_MODE=0 # set MD_FLEXIBLE_MODE to single-site
            # Tests need definitions of SINGLESITE and MULTISITE so it can correctly compile single-site tests
            SINGLESITE=0
            MULTISITE=1
    )

    target_include_directories(
            mdLibSingleSiteTests PUBLIC
            ${PROJECT_SOURCE_DIR}/tests/testAutopas molecularDynamics
    )

    target_link_libraries(
            mdLibSingleSiteTests PUBLIC
            autopas autopasTools molecularDynamicsLibrary gmock
    )

    file(
            GLOB_RECURSE
            MS_TEST_SRC
            "molecularDynamics/tests/multiSiteTests/*.cpp"
            "molecularDynamics/tests/multiSiteTests/*.h"
            "molecularDynamics/tests/particlePropertiesLibraryTests/*.cpp"
            "molecularDynamics/tests/particlePropertiesLibraryTests/*.h"
            "molecularDynamics/tests/templateInstatiations/AutoPasInstantiations.cpp"
    )

    add_executable(mdLibMultiSiteTests ${MS_TEST_SRC} mdLibTests.cpp)

    target_compile_definitions(
            mdLibMultiSiteTests PRIVATE
            MD_FLEXIBLE_MODE=1  # set MD_FLEXIBLE_MODE to multi-site
            # Tests need definitions of SINGLESITE and MULTISITE so it can correctly compile multi-site tests
            SINGLESITE=0
            MULTISITE=1
    )

    target_include_directories(
            mdLibMultiSiteTests PUBLIC
            ${PROJECT_SOURCE_DIR}/tests/testAutopas molecularDynamics
    )

    target_link_libraries(
            mdLibMultiSiteTests PUBLIC
            autopas autopasTools molecularDynamicsLibrary gmock
    )

    # this cmake module was only introduced in 3.10
    include(GoogleTest)
    # more robust, queries the compiled executable
    gtest_discover_tests(
            mdLibSingleSiteTests TEST_PREFIX "mdLibSingleSiteTests/"
            PROPERTIES
            ENVIRONMENT "${LSAN_OPTIONS_STR}"
    )
    gtest_discover_tests(
            mdLibMultiSiteTests TEST_PREFIX "mdLibMultiSiteTests/"
            PROPERTIES
            ENVIRONMENT "${LSAN_OPTIONS_STR}"
    )

# --- SPH ---
    file(
            GLOB_RECURSE
            SPH_TEST_SRC
            "sph/tests/*.cpp"
            "sph/tests/*.h"
    )

    add_executable(sphLibTests ${SPH_TEST_SRC} sphLibTests.cpp)

    target_include_directories(
            sphLibTests PUBLIC
            ${PROJECT_SOURCE_DIR}/tests/testAutopas sph
    )

    target_link_libraries(
            sphLibTests PUBLIC
            autopas autopasTools SPHLibrary gmock
    )

    # this cmake module was only introduced in 3.10
    include(GoogleTest)
    # more robust, queries the compiled executable
    gtest_discover_tests(
            sphLibTests TEST_PREFIX "sphLibTests/"
            PROPERTIES
            ENVIRONMENT "${LSAN_OPTIONS_STR}"
    )

else()
    message(STATUS "AUTOPAS_BUILD_TESTS is disabled. Not building Application Libraries tests!")
endif()