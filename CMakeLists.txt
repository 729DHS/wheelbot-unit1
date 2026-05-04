cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unit1_dm4310_test)

target_sources(app PRIVATE
	src/main.c
	src/dm4310_motor.c
)
