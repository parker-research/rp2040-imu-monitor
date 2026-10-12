# pico_sdk_import.cmake
#
# Resolution order:
#   1. -DPICO_SDK_PATH=... on the cmake command line
#   2. $PICO_SDK_PATH environment variable
#   3. Fetch from GitHub via FetchContent (cached in the build dir)

set(
  PICO_SDK_FETCH_FROM_GIT_TAG "2.2.0"
  CACHE STRING "Pico SDK git tag to fetch"
)

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
  set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
  message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}').")
endif ()

if (NOT PICO_SDK_PATH)
  message(
    "PICO_SDK_PATH not set. "
    "Fetching Pico SDK ${PICO_SDK_FETCH_FROM_GIT_TAG} from GitHub..."
  )
  include(FetchContent)
  FetchContent_Declare(
        pico_sdk
        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
        GIT_TAG        ${PICO_SDK_FETCH_FROM_GIT_TAG}
        GIT_SUBMODULES_RECURSE TRUE
    )
  FetchContent_MakeAvailable(pico_sdk)
  set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
endif ()

get_filename_component(
  PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}"
)
if (NOT EXISTS ${PICO_SDK_PATH})
  message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' not found")
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
  message(
    FATAL_ERROR
    "Directory '${PICO_SDK_PATH}' does not contain the Raspberry Pi Pico SDK"
  )
endif ()

set(
  PICO_SDK_PATH ${PICO_SDK_PATH}
  CACHE PATH "Path to the Raspberry Pi Pico SDK"
  FORCE
)

include(${PICO_SDK_INIT_CMAKE_FILE})
