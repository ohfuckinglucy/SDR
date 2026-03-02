cmake_minimum_required(VERSION 3.16)
project(PlutoSDR_Notes CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# Директории
set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/../thirdparty)
set(FUNC_DIR ${PROJECT_SOURCE_DIR}/functions)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# OpenGL/GLEW/SoapiSDR
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(SoapySDR REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(FFTW3 REQUIRED fftw3)

include_directories(${INCLUDE_DIR})
include_directories(${SDL2_INCLUDE_DIRS})

# ImGui
add_library(imgui STATIC
    ${THIRD_PARTY_DIR}/imgui/imgui.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_draw.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_demo.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_tables.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_widgets.cpp
    ${THIRD_PARTY_DIR}/imgui/backends/imgui_impl_sdl2.cpp
    ${THIRD_PARTY_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${THIRD_PARTY_DIR}/imgui)
target_compile_options(imgui PRIVATE -fPIC)
target_link_libraries(imgui PRIVATE ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

# ImPlot
add_library(implot STATIC
    ${THIRD_PARTY_DIR}/implot/implot.cpp
    ${THIRD_PARTY_DIR}/implot/implot_demo.cpp
    ${THIRD_PARTY_DIR}/implot/implot_items.cpp
)
target_include_directories(implot PUBLIC ${THIRD_PARTY_DIR}/implot)
target_link_libraries(implot PRIVATE imgui)


set(APP_SOURCES
    ${SRC_DIR}/main_gui.cpp
    ${FUNC_DIR}/modulator.cpp
    ${FUNC_DIR}/ofdm_core.cpp
    ${FUNC_DIR}/sdr_hw.cpp
    ${FUNC_DIR}/sync_freq.cpp
    ${FUNC_DIR}/sync_time.cpp
    ${FUNC_DIR}/threads.cpp
)

set(TX_SOURCES
    ${SRC_DIR}/main_tx.cpp
    ${FUNC_DIR}/modulator.cpp
    ${FUNC_DIR}/ofdm_core.cpp
    ${FUNC_DIR}/sdr_hw.cpp
    ${FUNC_DIR}/sync_freq.cpp
    ${FUNC_DIR}/sync_time.cpp
    ${FUNC_DIR}/threads.cpp
)

add_executable(rx.elf ${APP_SOURCES})
add_executable(tx.elf ${TX_SOURCES})

target_link_libraries(rx.elf PRIVATE
    imgui
    implot
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    fftw3
)

target_include_directories(rx.elf PRIVATE ${INCLUDE_DIR} fftw3)

target_link_libraries(tx.elf PRIVATE
    imgui
    implot
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${SoapySDR_LIBRARIES}
    fftw3
)

target_include_directories(tx.elf PRIVATE ${INCLUDE_DIR})