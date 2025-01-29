"softiron_values.txt" Original dataset, iterating from -255 to 255 PWM to the magnetorquer and then reading the magnetometer IMU values. The columns are as follows: PWM, mag_x, mag_y, mag_z. For calculating softiron values in "softiron_calibration.ipynb" in the parent directory

"softiron_values_2.txt" The second dataset, the original values were off by a magnitude of 10, due to an error in the code, this fixes that.

"softiron_testing_linear.txt" Demonstration that the softiron values calculated previously worked. Using live IMU values and correcting for them with the offset as PWM goes from -255 to 255 are recorded in this file. Columns are as follows: PWM, mag_x, mag_y, mag_z, x_offset, y_offset, z_offset.

"softiron_testing_random.txt" Demonstration of the softiron values calculated previously working again. However, this time PWM values were randomly generated for. Columns are the same as in "softiron_testing_linear.txt"

The data in "voltimu_troubleshooting" all pertain to trying to understand why the IMU on the breadboard reads different values depending on the source of the voltage given to it. All of the data in this folder is the result of iterating from -255 to 255 PWM on the magnetorquer and recording on the magnetometer. The columns are the same format as "softiron_testing_linear.txt"
	"softiron_values_3.txt": 09.30.2024 test, using the battery 
	"softiron_values_3_again.txt": 09.30.2024 test, using the battery 
	"voltimugraph_4.03v": 09.30.2024 test, using the power source
	"softiron_values_4": 10.02.2024 test, using the battery
	"softiron_values_5": 10.02.2024 test, using the battery
	"voltimugraph_4.03v_again": 10.02.2024 test, using the power source
	"softiron_values_6": 10.02.2024 test, using the battery
	"voltimugraph_4.03v_again2": 10.02.2024 test, using the power source
	"softiron_values_7_bad": 10.02.2024 test, using the battery, I think I might have 		plugged in the battery but not completely in the beginning so there is a dip in readings
	"softiron_values_7": 10.02.2024 test, using the battery