#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#                            All Rights Reserved.
#
#  This file is subject to the terms and conditions defined in
#  file 'LICENSE.md', which is part of this source code package.
#
#  NOTICE:  All information contained herein is, and remains the property of Klepsydra
#  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
#  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
#  trade secret or copyright law. Dissemination of this information or reproduction of
#  this material is strictly forbidden unless prior written permission is obtained from
#  Klepsydra Technologies GmbH.
#
#****************************************************************************

SET(PROJ_NAME kpsr_zmq_${PROJ_MAIN_NAME}_tests)
PROJECT(${PROJ_NAME})

# Source
#---------------------------------------------------#
KpsrEventGenerator("${CMAKE_SOURCE_DIR}/modules/codegen/kidl_test1" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test1" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_SOURCE_DIR}/modules/codegen/kidl_test2" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test2" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_SOURCE_DIR}/modules/codegen/kidl_test3" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test3" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_SOURCE_DIR}/modules/codegen/kidl_test4" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test4" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_SOURCE_DIR}/modules/codegen/kidl_test5" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test5" "klepsydra/codegen" "True" "False" "True")

FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

FILE(GLOB ${PROJ_NAME}_HEADERS
  "gen_test*/*.h"
)

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

# Link libraries with Project
#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME} PUBLIC kpsr_core kpsr_zmq_core gtest_main )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}
  PRIVATE
  ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test1/poco/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test1/serialization/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test2/poco/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test2/serialization/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test3/poco/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test3/serialization/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test4/poco/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test4/serialization/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test5/poco/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen_test5/serialization/include
  )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
