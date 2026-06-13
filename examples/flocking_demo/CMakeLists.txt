add_executable(flocking_demo main.cpp)
target_link_libraries(flocking_demo PRIVATE mith::mith)
target_compile_features(flocking_demo PRIVATE cxx_std_17)
