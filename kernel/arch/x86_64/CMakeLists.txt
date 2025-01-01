add_executable(roanix_exe ${KERNEL_SRC})

set(
    FREESTND_X86_FLAGS
    "-m64"
    "-march=x86-64"
    "-mno-80387"
    "-mno-mmx"
    "-mno-sse"
    "-mno-sse2"
    "-mno-red-zone"
    "-mcmodel=kernel"
)

set(
    X86_LINK_FLAGS
    "-Wl,-m"
    "-Wl,elf_x86_64"
    "-Wl,-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
)

add_executable(roanix::exe ALIAS roanix_exe)
set_property(TARGET roanix_exe PROPERTY OUTPUT_NAME roanix)

target_include_directories(
    roanix_exe PRIVATE

    ${FRIGG_INCLUDE_DIRS}
    ${FREESTND_C_INCLUDE_DIRS}
    ${FREESTND_CXX_INCLUDE_DIRS}

    "\$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/vendor/limine/>"
)

target_compile_options(
    roanix_exe PRIVATE 
    ${FREESTND_FLAGS}
    ${FREESTND_X86_FLAGS}
    ${KERNEL_CXX_FLAGS}
)

target_link_options(roanix_exe PRIVATE ${KERNEL_LINK_FLAGS} ${X86_LINK_FLAGS})
target_compile_features(roanix_exe PRIVATE cxx_std_20)

install(TARGETS roanix_exe DESTINATION "/boot")