cmake_minimum_required(VERSION 3.10)

project(learn-llvm)

find_package(LLVM REQUIRED CONFIG)

include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

llvm_map_components_to_libnames(llvm_libs support core irreader)

message(INFO ${llvm_libs})


add_executable(learn-llvm main.cc scanner.cc parser.cc)
target_link_libraries(learn-llvm ${llvm_libs} LLVMScalarOpts)
