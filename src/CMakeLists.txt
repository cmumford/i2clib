# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(SOURCES address.cc bus.cc master.cc operation.cc simple_master.cc status_esp.cc)

idf_component_register(SRCS ${SOURCES})

target_compile_definitions(${COMPONENT_LIB} PUBLIC)
