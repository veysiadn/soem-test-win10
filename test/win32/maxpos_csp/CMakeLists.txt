
set(SOURCES main.c ecat_dc.c servo.c)
add_executable(maxpos_csp ${SOURCES})
target_link_libraries(maxpos_csp soem)
install(TARGETS maxpos_csp DESTINATION bin)
