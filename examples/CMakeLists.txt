
add_executable(session_pub
    session_pub.cpp
)
target_link_libraries(session_pub PUBLIC 
    Session::session 
    flatbuffers::flatbuffers    
    zenohc
)

#####

add_executable(session_sub
    session_sub.cpp
)
target_link_libraries(session_sub PUBLIC
    Session::session 
    flatbuffers::flatbuffers    
    zenohc
)

#####

add_executable(session_serve
    session_serve.cpp
)
target_link_libraries(session_serve PUBLIC
    Session::session 
    flatbuffers::flatbuffers    
    zenohc
)

#####

add_executable(session_call
    session_call.cpp
)
target_link_libraries(session_call PUBLIC
    Session::session 
    flatbuffers::flatbuffers    
    zenohc
)

