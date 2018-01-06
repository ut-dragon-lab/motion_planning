function(tinyspline_build  TINYSPLINE_LIBS_VAR TINYSPLINE_INCLUDE_DIR_VAR)

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/tinyspline)
    execute_process(COMMAND git clone git@github.com:KahnShi/tinyspline.git ${CMAKE_CURRENT_BINARY_DIR}/tinyspline)
    execute_process(COMMAND sed -i -e "s/VERSION\ 3.4/VERSION\ 2.8/g" ${CMAKE_CURRENT_BINARY_DIR}/tinyspline/CMakeLists.txt)
    execute_process(COMMAND sed -i -e "/TINYSPLINE_DISABLE_CXX11_FEATURES/s/OFF/ON/gi" ${CMAKE_CURRENT_BINARY_DIR}/tinyspline/library/CMakeLists.txt)
    set(build_dir ${CMAKE_CURRENT_BINARY_DIR}/tinyspline/build)
    file(MAKE_DIRECTORY ${build_dir})
    execute_process(COMMAND ${CMAKE_COMMAND} .. WORKING_DIRECTORY ${build_dir})
    execute_process(COMMAND make WORKING_DIRECTORY ${build_dir})

  endif()

  set(TINYSPLINE_LIBS
    "${CMAKE_CURRENT_BINARY_DIR}/tinyspline/build/lib/libtinyspline.so"
    "${CMAKE_CURRENT_BINARY_DIR}/tinyspline/build/lib/libtinysplinecpp.so")

  set(${TINYSPLINE_INCLUDE_DIR_VAR} "${CMAKE_CURRENT_BINARY_DIR}/tinyspline/library" PARENT_SCOPE)
  set(${TINYSPLINE_LIBS_VAR} ${TINYSPLINE_LIBS} PARENT_SCOPE)

endfunction()
