#!/bin/sh
./runner.exe impulse_input.dat > impulse_output.csv
./runner.exe jittery_ramp_input.dat > jittery_ramp_output.csv
./runner.exe noisy_ramp_input.dat > noisy_ramp_output.csv
./runner.exe noisy_wavy_jittery_ramp_input.dat > noisy_wavy_jittery_ramp_output.csv
./runner.exe noisy_wavy_ramp_input.dat > noisy_wavy_ramp_output.csv
./runner.exe ramp_input.dat > ramp_output.csv
./runner.exe step_input.dat > step_output.csv
./runner.exe wavy_ramp_input.dat > wavy_ramp_output.csv
