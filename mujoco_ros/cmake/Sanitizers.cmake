include_guard()

option(ENABLE_ASAN "Enable address sanitizer" FALSE)
option(ENABLE_LSAN "Enable leak sanitizer" FALSE)
option(ENABLE_UBSAN "Enable undefined behavior sanitizer" FALSE)
option(ENABLE_TSAN "Enable thread sanitizer" FALSE)
option(ENABLE_MSAN "Enable memory sanitizer" FALSE)

function (configure_sanitizers)
  cmake_parse_arguments("" "" "TARGET" "" ${ARGN})

  if (NOT _TARGET)
    message(FATAL_ERROR "There should be at least one target specified")
  endif ()

  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(SANITIZERS
        $<$<BOOL:${ENABLE_ASAN}>:address> $<$<BOOL:${ENABLE_LSAN}>:leak> $<$<BOOL:${ENABLE_UBSAN}>:undefined>
        $<$<AND:$<BOOL:${ENABLE_TSAN}>,$<NOT:$<BOOL:${ENABLE_ASAN}>>,$<NOT:$<BOOL:${ENABLE_LSAN}>>>:thread>
        $<$<AND:$<BOOL:${ENABLE_MSAN}>,$<NOT:$<BOOL:${ENABLE_ASAN}>>,$<NOT:$<BOOL:${ENABLE_TSAN}>>,$<NOT:$<BOOL:${ENABLE_LSAN}>>>:memory>)

    if (ENABLE_TSAN AND (ENABLE_ASAN OR ENABLE_LSAN))
      message(WARNING "Thread sanitizer does not work with Address and Leak sanitizer enabled, skipping TSAN")
    endif ()

    if (ENABLE_MSAN AND CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
      # see https://github.com/google/sanitizers/wiki/MemorySanitizerLibcxxHowTo
      message(WARNING "MSAN requires all the code (including libc++) to be MSan-instrumented otherwise it reports false positives")
      if (ENABLE_ASAN OR ENABLE_TSAN OR ENABLE_LSAN)
        message(WARNING "Memory sanitizer does not work with Address, Thread and Leak sanitizer enabled, skipping MSAN")
      endif ()
    endif ()

    # see https://clang.llvm.org/docs/ThreadSanitizer.html, https://clang.llvm.org/docs/AddressSanitizer.html and
    # https://clang.llvm.org/docs/MemorySanitizer.html
    set(msan_flag $<$<BOOL:${ENABLE_MSAN}>:-fPIE -pie>)
    set(perfect_stacktraces $<$<BOOL:${ENABLE_ASAN}>:-fno-omit-frame-pointer -fno-optimize-sibling-calls>)
    set(disable_inline_optimization $<$<OR:$<BOOL:${ENABLE_ASAN}>,$<BOOL:${ENABLE_TSAN}>,$<BOOL:${ENABLE_MSAN}>>:-O1>)
    set(sanitizer_flag "$<JOIN:${SANITIZERS},$<COMMA>>")
    target_compile_options(${_TARGET} INTERFACE $<$<BOOL:${sanitizer_flag}>:-fsanitize=${sanitizer_flag}> ${disable_inline_optimization}
                                                ${perfect_stacktraces} ${msan_flag})
    target_link_options(${_TARGET} INTERFACE $<$<BOOL:${sanitizer_flag}>:-fsanitize=${sanitizer_flag}>)
  elseif (MSVC)
    if (${ENABLE_LSAN} OR ${ENABLE_UBSAN} OR ${ENABLE_TSAN} OR ${ENABLE_MSAN})
      message(WARNING "MSVC only supports address sanitizer")
    endif ()

    # https://docs.microsoft.com/en-us/cpp/sanitizers/asan?view=msvc-170
    set(SANITIZERS $<$<BOOL:${ENABLE_ASAN}>:/fsanitize=address>)
    set(perfect_stacktrace $<$<BOOL:${ENABLE_ASAN}>:/Zi>)
    target_compile_options(${_TARGET} INTERFACE ${SANITIZERS} ${perfect_stacktrace})
    target_link_options(${_TARGET} INTERFACE $<$<BOOL:${ENABLE_ASAN}>:/INCREMENTAL:NO>)
  endif ()
endfunction ()
