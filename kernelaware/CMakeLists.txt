project(kernelaware)

set(LIB_SOURCES
    kernelaware.cpp
    kerneldebug.cpp
)

set(LIB_HEADERS
    public/kernelaware.h
)

mark3_add_library(kernelaware ${LIB_SOURCES} ${LIB_HEADERS})

target_include_directories(kernelaware
    PUBLIC
        public
    )

target_link_libraries(kernelaware
    mark3
)
