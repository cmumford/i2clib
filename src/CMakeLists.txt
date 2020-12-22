# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(SOURCES master.cc operation.cc)

idf_component_register(SRCS ${SOURCES})

target_compile_definitions(${COMPONENT_LIB})
