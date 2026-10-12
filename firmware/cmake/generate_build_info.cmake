# Runs at make-time (via add_custom_target ALL) — not at cmake configure time.
# Caller must pass: -DOUTPUT_FILE=... -DSOURCE_DIR=...

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%dT%H:%M:%SZ" UTC)

execute_process(
    COMMAND git -C "${SOURCE_DIR}" rev-parse --short HEAD
    OUTPUT_VARIABLE GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_RESULT
)
if(NOT GIT_RESULT EQUAL 0 OR NOT GIT_COMMIT)
    set(GIT_COMMIT "unknown")
endif()

execute_process(
    COMMAND git -C "${SOURCE_DIR}" status --porcelain
    OUTPUT_VARIABLE GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(GIT_STATUS)
    set(GIT_COMMIT "${GIT_COMMIT}-dirty")
endif()

file(WRITE "${OUTPUT_FILE}"
"/* Auto-generated at build time — do not edit. */
#pragma once
#define BUILD_TIMESTAMP \"${BUILD_TIMESTAMP}\"
#define GIT_COMMIT \"${GIT_COMMIT}\"
")
