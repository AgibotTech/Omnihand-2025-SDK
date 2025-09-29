# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2

file(GLOB BASE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/kinematics_solver/*.cc)

file(GLOB IMPLEMENTATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/implementation/c_agibot_hand_can/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/implementation/c_agibot_hand_rs/*.cc)
file(GLOB CAN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/can_bus_device/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/can_bus_device/zlg_usb_canfd/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/can_bus_device/socket_can/*.cc)
file(GLOB RS485_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/rs_485_device/*.cc)

add_library(omnihand2025 SHARED ${BASE_SOURCES} ${IMPLEMENTATION_SOURCES} ${CAN_SOURCES} ${RS485_SOURCES})

target_include_directories(
  omnihand2025
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
         ${CMAKE_CURRENT_SOURCE_DIR}/implementation
         ${serial_SOURCE_DIR}/include)

set_target_properties(omnihand2025 PROPERTIES INSTALL_RPATH "$ORIGIN" BUILD_WITH_INSTALL_RPATH TRUE)

target_link_libraries(
  omnihand2025
  PRIVATE serial
          yaml-cpp::yaml-cpp)


  if(NOT USER_PATH)
    set(USER_PATH ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328)
  endif()

  find_path(THIRD_PARTY_PATH zcan.h HINTS ${USER_PATH})
  target_include_directories(omnihand2025 PUBLIC ${THIRD_PARTY_PATH})
  target_link_directories(omnihand2025 PUBLIC ${THIRD_PARTY_PATH})
  target_link_libraries(omnihand2025 PUBLIC usbcanfd)


install(TARGETS omnihand2025 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(FILES c_agibot_hand_base.h proto.h export_symbols.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

install(FILES can_bus_device/c_can_bus_device.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/can_bus_device)

install(FILES kinematics_solver/kinematics_solver.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/kinematics_solver)

install(FILES implementation/c_agibot_hand_can/c_agibot_hand_can.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/implementation/c_agibot_hand_can)

install(FILES implementation/c_agibot_hand_rs/c_agibot_hand_rs.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/implementation/c_agibot_hand_rs)


  install(FILES ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so
                ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

