file(GLOB_RECURSE MDFLIB_SRC "mdflib/src/*.cpp")

idf_component_register(
        SRCS ${MDFLIB_SRC} "port/timegm_port.cpp" "port/timegm_port.h"
        INCLUDE_DIRS "include" "port"
        REQUIRES expat zlib
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Convert only the two problematic warnings back to warnings (not errors) and silence them:
target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wno-error=pessimizing-move
        -Wno-error=reorder
        -Wno-pessimizing-move
        -Wno-reorder
        -Wno-deprecated-declarations    # optional: quiets std::filesystem u8path deprecation under GCC 14/C++23
)
