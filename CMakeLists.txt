cmake_minimum_required(VERSION 3.8)
project(ptu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ptu_messages REQUIRED)
find_package(std_srvs REQUIRED)


set(PTU_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include/sdk-2.0.4")
add_library(ptu_sdk STATIC
  include/ptu_driver/ptu_sdk.c
  ${PTU_SDK_ROOT}/examples/estrap.c
)

set_source_files_properties(
  include/ptu_driver/ptu_sdk.c
  ${PTU_SDK_ROOT}/examples/estrap.c
  PROPERTIES LANGUAGE C
)

target_include_directories(ptu_sdk
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include      # include/ptu_driver/...
    ${PTU_SDK_ROOT}                          # sdk headers
    ${PTU_SDK_ROOT}/cerial
    ${PTU_SDK_ROOT}/examples
)

target_link_libraries(ptu_sdk
  ${PTU_SDK_ROOT}/libcpi.a
  m
)


add_executable(driver src/PTU_driver.cpp)
ament_target_dependencies(driver rclcpp std_msgs ptu_messages std_srvs)


target_link_libraries(driver ptu_sdk)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
