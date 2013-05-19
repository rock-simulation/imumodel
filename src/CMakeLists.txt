rock_library(imumodel
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(imumodel_bin Main.cpp
    DEPS imumodel)

