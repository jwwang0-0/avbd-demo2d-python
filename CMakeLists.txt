cmake_minimum_required(VERSION 3.18)
project(avbd2d_python LANGUAGES CXX)

# Use modern Python discovery in pybind11 (silences a warning)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

set(AVBD_DIR ${CMAKE_SOURCE_DIR}/third_party/avbd-demo2d)

# Collect all core sources EXCEPT the demo entry point
file(GLOB AVBD_SOURCES CONFIGURE_DEPENDS "${AVBD_DIR}/source/*.cpp")
list(FILTER AVBD_SOURCES EXCLUDE REGEX ".*/main\\.cpp")

# --- Core static lib (demo's solver) ---
add_library(avbd_core STATIC ${AVBD_SOURCES})
target_include_directories(avbd_core PUBLIC ${AVBD_DIR}/source)
target_compile_features(avbd_core PUBLIC cxx_std_17)
if (MSVC)
  target_compile_options(avbd_core PRIVATE /W4 /EHsc)
else()
  target_compile_options(avbd_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# Link OpenGL: Windows needs opengl32; other OS use OpenGL::GL
if (WIN32)
  target_link_libraries(avbd_core PRIVATE opengl32)
else()
  find_package(OpenGL REQUIRED)
  target_link_libraries(avbd_core PRIVATE OpenGL::GL)
endif()

# --- Tiny API shim exposing a clean interface ---
add_library(avbd_api STATIC bindings/api_shim.cpp)
target_include_directories(avbd_api PUBLIC bindings ${AVBD_DIR}/source)
target_link_libraries(avbd_api PRIVATE avbd_core)

# --- Python module ---
pybind11_add_module(avbd2d MODULE bindings/avbd_py.cpp)
target_link_libraries(avbd2d PRIVATE avbd_api)
# On Windows also link the module to OpenGL in case of transitive refs
if (WIN32)
  target_link_libraries(avbd2d PRIVATE opengl32)
else()
  target_link_libraries(avbd2d PRIVATE OpenGL::GL)
endif()

# Install the Python extension into the wheel root
# (this is the crucial bit so the wheel actually contains avbd2d.pyd)
install(TARGETS avbd2d
        LIBRARY DESTINATION .
        RUNTIME DESTINATION .
        ARCHIVE DESTINATION .)

# Produce avbd2d.pyd without 'lib' prefix
set_target_properties(avbd2d PROPERTIES PREFIX "")
