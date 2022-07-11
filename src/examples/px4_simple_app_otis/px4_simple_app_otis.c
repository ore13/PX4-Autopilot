/****************************************************************************
*
*   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
* @file px4_simple_app_otis.c
*
* @author Otis Rea <otiswilsonrea@gmail.com>
*/
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_otis_main(int argc, char *argv[]);

int px4_simple_app_otis_main(int argc, char *argv[])
{
	// Notify that we are starting
	PX4_INFO("Starting Hello Sky...");

	// Subscribe to the combined sensor topic
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	// update rate is limited to 5hz
	orb_set_interval(sensor_sub_fd, 200);

	// create a polling struct. note that here only one topic is waited for.
	// multiple topics can be added with different sensor sub file descriptor
	// i.e add another {.fd = sensor_sub_XX, .events = POLLIN}
	px4_pollfd_struct_t fds[] = {
		{.fd = sensor_sub_fd, .events = POLLIN},
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		// px4_poll is 0 or negative if data is not returned
		int poll_ret = px4_poll(fds, 1, 1000);

		// handle the result of the poll
		if (poll_ret == 0) {
			PX4_ERR("Got no data within 1 second");
		} else if (poll_ret < 0) {
			/*
			This logic allows the 1st 10 errors, then will only
			print errors every 50 error occurances. This minimises
			flooding the output with error messages
			*/
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("ERRROR return value from poll(): %d", poll_ret);
			}

			error_counter++;
		} else {
			if (fds[0].revents & POLLIN) {
				struct sensor_combined_s raw;
				// copy sensor data into local buffer
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

				// Print the results
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				(double)raw.accelerometer_m_s2[0],
				(double)raw.accelerometer_m_s2[1],
				(double)raw.accelerometer_m_s2[2]);
			}
		}
	}

	PX4_INFO("Finished");

	return OK;
}

