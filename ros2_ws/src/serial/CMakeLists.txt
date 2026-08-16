cmake_minimum_required(VERSION 3.8)
project(serial)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)
ament_export_targets(serial_targets HAS_LIBRARY_TARGET)

## Sources
## Add serial library (static)
add_library(${PROJECT_NAME} STATIC
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

# 静态库链接进可执行文件不需要 PIC，但加上可兼容将来链接进共享库的场景
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(APPLE) # macOS
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc
    )
    target_link_libraries(${PROJECT_NAME} PUBLIC ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # .*nix
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
     )
    # PUBLIC + 导出 targets，依赖会传递给下游（静态库必须，动态库不需要）
    target_link_libraries(${PROJECT_NAME} PUBLIC rt pthread)
elseif(WIN32) # Windows
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
    )
    target_link_libraries(${PROJECT_NAME} PUBLIC setupapi)
endif()


## Include headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

## Install library
install(TARGETS ${PROJECT_NAME}
    EXPORT serial_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
    DESTINATION include/serial
)

## Export targets so downstream packages (e.g. base) inherit link dependencies
install(EXPORT serial_targets
    FILE serial_targets.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
