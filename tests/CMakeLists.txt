add_executable(mith_unit_tests
    unit/test_main.cpp
    unit/test_uuid.cpp
    unit/test_hierarchical_id.cpp
    unit/test_bounded_queue.cpp
    unit/test_registry.cpp
    unit/test_builtin_components.cpp
    unit/test_scheduler.cpp
    unit/test_json_writer.cpp
    unit/test_trace_sink.cpp
    unit/test_view_and_snapshot.cpp
    unit/test_action.cpp
    unit/test_message.cpp
    unit/test_world.cpp
)

target_link_libraries(mith_unit_tests PRIVATE mith::mith)

target_include_directories(mith_unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/doctest
)

target_compile_features(mith_unit_tests PRIVATE cxx_std_17)
target_compile_options(mith_unit_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wshadow -Wconversion>
)

add_test(NAME mith_unit_tests COMMAND mith_unit_tests)
