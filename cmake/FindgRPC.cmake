# Minimal FindgRPC.cmake using pkg-config and defining imported targets
find_package(PkgConfig REQUIRED)

pkg_check_modules(GRPC REQUIRED grpc)
pkg_check_modules(GRPCPP REQUIRED grpc++)

add_library(gRPC::grpc INTERFACE IMPORTED)
set_target_properties(gRPC::grpc PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${GRPC_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES "${GRPC_LINK_LIBRARIES}"
)

add_library(gRPC::grpc++ INTERFACE IMPORTED)
set_target_properties(gRPC::grpc++ PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${GRPCPP_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES "${GRPCPP_LINK_LIBRARIES}"
)

# Optional: grpc_plugin path for codegen users
find_program(GRPC_CPP_PLUGIN_EXECUTABLE NAMES grpc_cpp_plugin)
set(gRPC_CPP_PLUGIN ${GRPC_CPP_PLUGIN_EXECUTABLE} CACHE FILEPATH "Path to grpc_cpp_plugin")
