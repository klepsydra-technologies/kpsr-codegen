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

SET(PROJ_NAME kpsr_dds_${PROJ_MAIN_NAME}_tests_3_4)
PROJECT(${PROJ_NAME})

file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/gen)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gen)

file(GLOB idls ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test3/dds/idl/*.idl
               ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test4/dds/idl/*.idl)
FOREACH(idl ${idls})
        Connext_IDLGEN (${idl} ${CMAKE_CURRENT_SOURCE_DIR})
        GET_FILENAME_COMPONENT(nfile ${idl} NAME)
        Connext_GENDATAMODEL(${nfile} ${CMAKE_CURRENT_SOURCE_DIR})
ENDFOREACH(idl)


# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
ADD_LIBRARY (kpsr_dds_${PROJ_MAIN_NAME}_datamodel_3_4 SHARED ${Connext_DATAMODEL})

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

#---------------------------------------------------#
TARGET_LINK_LIBRARIES(kpsr_dds_${PROJ_MAIN_NAME}_datamodel_3_4
  PUBLIC
  RTIConnextDDS::cpp2_api
  )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}
  PRIVATE
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test3/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test3/dds/include 
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test4/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test4/dds/include
  ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gen
)
TARGET_LINK_LIBRARIES(${PROJ_NAME} kpsr_core gtest_main
  kpsr_dds_core
  kpsr_dds_serialization_datamodel kpsr_dds_${PROJ_MAIN_NAME}_datamodel_3_4)

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
