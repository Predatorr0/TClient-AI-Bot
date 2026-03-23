# only find ssp when toolchain is gcc
if(TARGET_OS STREQUAL "windows"
  AND NOT CMAKE_C_COMPILER_ID STREQUAL "Clang"
  AND NOT CMAKE_C_COMPILER_ID STREQUAL "MSVC")
  set_extra_dirs_lib(SSP ssp)
  find_file(SSP_LIBRARY
    NAMES libssp-0.dll libssp.a
    HINTS ${HINTS_SSP_LIBDIR}
    PATHS ${PATHS_SSP_LIBDIR}
    ${CROSSCOMPILING_NO_CMAKE_SYSTEM_PATH}
  )

  is_bundled(SSP_BUNDLED "${SSP_LIBRARY}")
  if(NOT SSP_BUNDLED AND NOT SSP_LIBRARY)
    message(WARNING "could not find ssp paths, but continuing anyway")
  endif()
  if(SSP_LIBRARY MATCHES "libssp-0.dll")
    set(SSP_COPY_FILES
      "${EXTRA_SSP_LIBDIR}/libssp-0.dll"
    )
  endif()
endif()
