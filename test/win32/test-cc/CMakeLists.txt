
set(SOURCES hello_soem.cc)
add_executable(hello_soem ${SOURCES})
target_link_libraries(hello_soem soem)
install(TARGETS hello_soem DESTINATION bin)
