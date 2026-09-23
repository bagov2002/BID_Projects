idf_component_register(SRCS "motor test.c" "stepper.c" "profile.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_gptimer esp_rom esp_timer)
