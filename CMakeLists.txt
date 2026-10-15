cmake_minimum_required(VERSION 3.16)

project(nonlinearcircuits
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include Eigen headers
#include_directories(/home/mazi/Desktop/eigen-3.4.0)
#include_directories(/Users/kozuk/Desktop/eigen/eigen-3.4.0)

# Buscar Eigen3 en el sistema
#find_package(Eigen3 3.3 REQUIRED CONFIG)


add_executable(nonlinearcircuits
    main.cc
    src/twotermdevice.cc
    src/mosfet.cc             
    src/mosfetfreebody.cc
    src/circuits.cc
    src/simulation.cc
    )

# 1. Busca si Eigen está instalado de forma global en el sistema
find_package(Eigen3 3.3 QUIET)

if(TARGET Eigen3::Eigen)
    target_link_libraries(nonlinearcircuits PRIVATE Eigen3::Eigen)
# 2. Si estás en la PC de Windows (revisa si la carpeta existe)
elseif(EXISTS "C:/Users/kozuk/Desktop/eigen/eigen-3.4.0")
    target_include_directories(nonlinearcircuits PRIVATE "C:/Users/kozuk/Desktop/eigen/eigen-3.4.0")
# 3. Si estás en la PC de Linux (revisa si la carpeta existe)
elseif(EXISTS "/home/mazi/Desktop/eigen-3.4.0")
    target_include_directories(nonlinearcircuits PRIVATE "/home/mazi/Desktop/eigen-3.4.0")
else()
    message(FATAL_ERROR "No se encontró la carpeta de Eigen3 en ninguna de las rutas predeterminadas.")
endif()


# Forzar flag -O3 para este objetivo específico
#if(MSVC)
 #   target_compile_options(nonlinearcircuits PRIVATE /O2) # MSVC equivalente
#else()
 #   target_compile_options(nonlinearcircuits PRIVATE -O3 -march=native) # GCC / Clang
#endif()

#incluye los directorios de encabezados
target_include_directories(nonlinearcircuits
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

