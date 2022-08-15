cmake_minimum_required(VERSION 3.19)
# Pull in SDK (must be before project)
set(MODEL_FOLDER .)
set(EI_SDK_FOLDER edge-impulse-sdk)

include(pico_sdk_import.cmake)

project(Pico_gesture_recognition)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

pico_sdk_init()
add_subdirectory(pico-ssd1306)
add_executable(Pico_gesture_recognition
src/mpu6050.cpp
src/ei_classifier_porting.cpp
src/main.cpp)


target_include_directories(Pico_gesture_recognition PUBLIC inc/
) 



pico_enable_stdio_usb(Pico_gesture_recognition 1)
pico_enable_stdio_uart(Pico_gesture_recognition 0)

pico_add_extra_outputs(Pico_gesture_recognition)

include(${MODEL_FOLDER}/edge-impulse-sdk/cmake/utils.cmake)



target_link_libraries(Pico_gesture_recognition pico_stdlib hardware_gpio hardware_i2c hardware_pio hardware_pwm pico_ssd1306 pico_multicore)





target_include_directories(Pico_gesture_recognition PRIVATE
  ${MODEL_FOLDER}
  ${MODEL_FOLDER}/classifer
  ${MODEL_FOLDER}/tflite-model
  ${MODEL_FOLDER}/model-parameters
  )

target_include_directories(Pico_gesture_recognition PRIVATE
  ${EI_SDK_FOLDER}
  ${EI_SDK_FOLDER}/third_party/ruy
  ${EI_SDK_FOLDER}/third_party/gemmlowp
  ${EI_SDK_FOLDER}/third_party/flatbuffers/include
  ${EI_SDK_FOLDER}/third_party
  ${EI_SDK_FOLDER}/tensorflow
  ${EI_SDK_FOLDER}/dsp
  ${EI_SDK_FOLDER}/classifier
  ${EI_SDK_FOLDER}/anomaly
  ${EI_SDK_FOLDER}/CMSIS/NN/Include
  ${EI_SDK_FOLDER}/CMSIS/DSP/PrivateInclude
  ${EI_SDK_FOLDER}/CMSIS/DSP/Include
  ${EI_SDK_FOLDER}/CMSIS/Core/Include
  )

include_directories(${INCLUDES})

# find model source files
RECURSIVE_FIND_FILE(MODEL_FILES "${MODEL_FOLDER}/tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(SOURCE_FILES "${EI_SDK_FOLDER}" "*.cpp")
RECURSIVE_FIND_FILE(CC_FILES "${EI_SDK_FOLDER}" "*.cc")
RECURSIVE_FIND_FILE(S_FILES "${EI_SDK_FOLDER}" "*.s")
RECURSIVE_FIND_FILE(C_FILES "${EI_SDK_FOLDER}" "*.c")
list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})

# add all sources to the project
target_sources(Pico_gesture_recognition PRIVATE ${SOURCE_FILES})

