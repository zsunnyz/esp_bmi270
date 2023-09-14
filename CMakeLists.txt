idf_component_register(SRCS "esp_bmi270.cpp" "include/BMI270-Sensor-API/bmi2_ois.c" "include/BMI270-Sensor-API/bmi2.c" "include/BMI270-Sensor-API/bmi270_context.c" "include/BMI270-Sensor-API/bmi270_legacy.c" "include/BMI270-Sensor-API/bmi270_maximum_fifo.c" "include/BMI270-Sensor-API/bmi270.c"
                    # SRC_DIRS "include/BMI270-Sensor-API"
                    # EXCLUDE_SRCS "include/BMI270-Sensor-API/bmi270_context_examples" "include/BMI270-Sensor-API/bmi270_examples" "include/BMI270-Sensor-API/bmi270_legacy_examples" "include/BMI270-Sensor-API/bmi270_max_fifo_examples"
                    REQUIRES driver log esp_event freertos
                    INCLUDE_DIRS "include")
