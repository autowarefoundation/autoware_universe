# NOTE(esteve): avoid "symbol `fatbinData' is already defined" errors
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-lto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-lto")
