add_library(rb_session 
  session.cpp
  zenoh_backend.cpp
)
target_include_directories(rb_session PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(rb_session PUBLIC
  zenohc::shared
  flatbuffers::flatbuffers
)