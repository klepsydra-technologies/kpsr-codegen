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

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/klepsydra-technologies/googletest.git
)
# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Klepsydra REQUIRED)

add_subdirectory(${PROJ_MAIN_NAME}_tests)

IF(KPSR_WITH_ZMQ)
   ADD_SUBDIRECTORY(zmq_${PROJ_MAIN_NAME}_tests)
ENDIF()
IF(KPSR_WITH_DDS)
  include (MacroConnext)
  find_package (RTIConnextDDS REQUIRED)
  ADD_SUBDIRECTORY(dds_${PROJ_MAIN_NAME}_tests_1_2)
  ADD_SUBDIRECTORY(dds_${PROJ_MAIN_NAME}_tests_3_4)
  ADD_SUBDIRECTORY(dds_${PROJ_MAIN_NAME}_tests_1_5)
ENDIF()
