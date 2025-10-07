cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)

# Ищем библиотеку SoapySDR
find_package(SoapySDR REQUIRED)

set(MAIN_SOURCE_FILES
    src/main.cpp
)

# Добавляем исполняемый файл
add_executable(main ${MAIN_SOURCE_FILES})

# Линкуем библиотеки к исполняемому файлу
target_link_libraries(main ${SoapySDR_LIBRARIES})
