cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)

# Ищем библиотеку SoapySDR
find_package(SoapySDR REQUIRED)

# Указываем директорию с заголовочными файлами
include_directories(${CMAKE_SOURCE_DIR}/include)

# Исходные файлы
set(SOURCE_FILES
    src/main.cpp
    functions/functions.cpp
    functions/modulator.cpp
)

# Добавляем исполняемый файл
add_executable(main ${SOURCE_FILES})

# Линкуем библиотеки
target_link_libraries(main ${SoapySDR_LIBRARIES})