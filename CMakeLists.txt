cmake_minimum_required(VERSION 3.16)

project(nonlinearcircuits
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include Eigen headers
include_directories(/home/mazi/Desktop/eigen-3.4.0)

add_executable(nonlinearcircuits
    main.cc
    src/twotermdevice.cc
    src/mosfet.cc             
    src/mosfetfreebody.cc
    src/circuits.cc
    src/utils.cc
    )

# Forzar flag -O3 para este objetivo específico
if(MSVC)
    target_compile_options(nonlinearcircuits PRIVATE /O2) # MSVC equivalente
else()
    target_compile_options(nonlinearcircuits PRIVATE -O3 -march=native) # GCC / Clang
endif()

target_include_directories(nonlinearcircuits
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)