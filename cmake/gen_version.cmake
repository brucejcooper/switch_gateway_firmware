# Regenerates ${OUT} (version.h) with FW_VERSION from git, only rewriting when
# the string changes so unchanged builds don't trigger a recompile.
# Invoked both at configure time and as a build-time custom target.
#   -DSRC_DIR=<repo>  -DOUT=<path/version.h>  -DBASE_VERSION=<x.y.z>

execute_process(
    COMMAND git -C "${SRC_DIR}" describe --tags --always --dirty
    OUTPUT_VARIABLE GIT_DESC
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)

if(NOT GIT_DESC)
    set(GIT_DESC "nogit")
endif()

set(VERSION "${BASE_VERSION}+${GIT_DESC}")
set(CONTENT "#pragma once\n#define FW_VERSION \"${VERSION}\"\n")

set(OLD "")
if(EXISTS "${OUT}")
    file(READ "${OUT}" OLD)
endif()

if(NOT OLD STREQUAL CONTENT)
    file(WRITE "${OUT}" "${CONTENT}")
    message(STATUS "version.h: FW_VERSION = ${VERSION}")
endif()
