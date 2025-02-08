function(target_default_compiler_warnings target_name)
  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    target_compile_options(
      ${target_name}
      PRIVATE -Weverything
              -Wno-c++98-compat
              -Wno-c++98-compat-pedantic
              -Werror)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(
      ${target_name}
      PRIVATE -Wall
              -Wextra
              -Wpedantic
              -Wcast-align
              -Wcast-qual
              -Wconversion
              -Wctor-dtor-privacy
              -Wduplicated-branches
              -Wduplicated-cond
              -Wextra-semi
              -Wfloat-equal
              -Wlogical-op
              -Wnon-virtual-dtor
              -Wold-style-cast
              -Woverloaded-virtual
              -Wredundant-decls
              -Wsign-conversion
              -Wshadow
              -Wstrict-null-sentinel
              -Wuseless-cast
              -Wzero-as-null-pointer-constant
              -Werror)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${target_name} PRIVATE /W4 /WX)

  endif()
endfunction()
