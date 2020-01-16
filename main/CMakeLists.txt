idf_component_register(SRCS main.cpp)
target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)

include(../components/link_component/link/AbletonLinkConfig.cmake)
target_link_libraries(${COMPONENT_TARGET} Ableton::Link)
