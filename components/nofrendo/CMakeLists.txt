FILE(GLOB_RECURSE app_sources ./*.* ./nofrendo/cpu/*.* ./nofrendo/libsnss/*.* ./nofrendo/mappers/*.* ./nofrendo/nes/*.* ./nofrendo/sndhrdw/*.*)

idf_component_register(SRCS ${app_sources}
    INCLUDE_DIRS 
    "./cpu/"
    "./libsnss/"
    "./nes/"
    "./sndhrdw/"
    "./mappers/"
    "./"
)

set(CFLAGS "${CFLAGS} -Wno-error=char-subscripts -Wno-error=attributes -DNOFRENDO_DEBUG")


