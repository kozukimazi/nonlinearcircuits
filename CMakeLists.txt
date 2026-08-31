cmake_minimum_required(VERSION 3.10)
project(JordanWigner)

# Include Eigen headers
include_directories(/home/mazi/Desktop/eigen0/eigen-3.4.0)

# Add executable
add_executable(QT jordanwigner.cc Qtermo.cc Aplication.cc)

#TO remove the file that generate the code
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "datosap.dat")