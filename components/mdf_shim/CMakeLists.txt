idf_component_register(
        SRCS "mdf_shim.cpp"
        INCLUDE_DIRS "include"
        REQUIRES mdflib expat zlib
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
