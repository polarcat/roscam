/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <time.h>
#include <stdint.h>
#include <memory>

namespace camera {

static inline uint64_t time_ms(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return now.tv_sec * 1000 + (uint32_t) (now.tv_nsec * .000001);
}

struct image {
	uint32_t id;
	uint16_t w;
	uint16_t h;
	uint8_t *data;
	uint32_t bytes;
	struct timespec time;
#ifdef PRINT_FPS
	uint32_t idx;
	float prev_ms;
	uint32_t flags;
#endif
};

class device;

class stream {
public:
	stream(device &);
	~stream();
	device &dev_;
	bool start();
	void get_frame_size(uint16_t &w, uint16_t &h);
	bool get_frame(struct image &);
	void put_frame();
};

using stream_ptr = std::unique_ptr<stream>;
stream_ptr create_rgb_stream(const char *path, uint16_t w, uint16_t h);
stream_ptr create_jpg_stream(const char *path, uint16_t w, uint16_t h);

}

#endif // CAMERA_H
