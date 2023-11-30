FILE(GLOB_RECURSE app_sources ./src/*.*)

idf_component_register(SRCS ${app_sources}
                    INCLUDE_DIRS "include"
                    REQUIRES nofrendo
                    )
