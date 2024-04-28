idf_component_register(
  SRCS 
  "deps/isotp-c/isotp.c"
  "util.cpp"
  "can.cpp"
  "main.cpp" 
  
  INCLUDE_DIRS
  "deps/isotp-c/"
  "deps/TaskScheduler/src/"
  "." 
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")