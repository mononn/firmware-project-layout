cmake_minimum_required(VERSION 3.5)

project(${OUTPUT})

# Include for ESP-IDF build system functions
include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
# Create idf::esp32 and idf::freertos static libraries
idf_build_process(esp32
	# try and trim the build; additional components
	# will be included as needed based on dependency tree
	#
	# although esptool_py does not generate static library,
	# processing the component is needed for flashing related
	# targets and file generation
	COMPONENTS
		esp32
		freertos
		esptool_py
	SDKCONFIG
		${CMAKE_CURRENT_LIST_DIR}/sdkconfig
	BUILD_DIR
		${OUTDIR}
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(${BASEDIR}/projects/version.cmake)
include(${BASEDIR}/projects/app.cmake)

set(elf_file ${CMAKE_PROJECT_NAME}.elf)
add_executable(${elf_file}
	start.c
	${APP_SRCS}
)

target_compile_options(${elf_file} PUBLIC ${compile_options})
target_include_directories(${elf_file} PRIVATE ${APP_INCS})
target_compile_definitions(${elf_file} PRIVATE ${APP_DEFS})

# Link the static libraries to the executable
target_link_libraries(${elf_file}
	idf::esp32
	idf::freertos
	idf::spi_flash
	idf::nvs_flash
)

set(mapfile "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map")
target_link_libraries(${elf_file}
	"-Wl,--cref" "-Wl,--defsym=IDF_TARGET_ESP32=0"
	"-Wl,--Map=\"${mapfile}\""
)

# Attach additional targets to the executable file for flashing,
# linker script generation, partition_table generation, etc.
idf_build_executable(${elf_file})
