/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <sys/mman.h>

#include "camera.h"
#define LOG_TAG "roscam"
#include "roslog.h"

namespace camera {

static constexpr int POLL_TIMEOUT_MS = 1000;
static constexpr uint8_t BUFFERS_CNT = 2;
static constexpr uint8_t RGB_PLANES = 3;
static constexpr uint8_t DEFAULT_FPS = 30;

struct buffer_view {
	void* data = nullptr;
	uint32_t size = 0;
};

struct frame {
	uint16_t w = 0;
	uint16_t h = 0;
	uint8_t bufcnt; // NOTE: force it small by intention; increase when needed
	struct buffer_view *buf;
};

class device {
public:
	device(int fd, uint32_t format) : fd(fd), format(format) {};
	~device()
	{
		for (uint8_t i = 0; i < frame.bufcnt; ++i)
			v4l2_munmap(frame.buf[i].data, frame.buf[i].size);
		nop("Closed video device %d", fd);
		v4l2_close(fd);
	};
	const int fd;
	const uint32_t format;
	struct frame frame;
	struct v4l2_buffer buf;
};

static bool dev_ioctl(int fd, long req, void *arg)
{
	int rc;
	do {
		rc = v4l2_ioctl(fd, req, arg);
	} while (rc == -1 && ((errno == EINTR) || (errno == EAGAIN)));

	if (rc < 0)
		return false;

	return true;
}

static int open_camera(const char* path)
{
	int fd;
	struct stat st;

	if (stat(path, &st) < 0) {
		ee("Failed to stat '%s'", path);
		return -1;
	}

	if (!S_ISCHR(st.st_mode))
		return -1;

	if ((fd = v4l2_open(path, O_RDWR | O_NONBLOCK, 0)) < 0) {
		ee("Failed to open v4l2_device '%s'", path);
		return -1;
	}

	return fd;
}

static void set_framerate(device &dev)
{
	const char *env = getenv("CAMERA_FPS");
	uint8_t fps;
	struct v4l2_streamparm par;

	env ? (fps = atoi(env)) : (fps = DEFAULT_FPS);
	par.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	par.parm.capture.timeperframe.numerator = 1;
	par.parm.capture.timeperframe.denominator = fps;

	if (!dev_ioctl(dev.fd, VIDIOC_S_PARM, &par)) {
		ee("Failed to request %u fps; v4l2_ioctl VIDIOC_S_PARM fd %d",
		 par.parm.capture.timeperframe.denominator, dev.fd);
		return;
	} else if (par.parm.capture.timeperframe.denominator != fps) {
		ww("Requested %u fps is not supported", fps);
	}

	ii("Streaming at %u fps", par.parm.capture.timeperframe.denominator);
}

static bool init_stream(device &dev, uint16_t w, uint16_t h)
{
	struct v4l2_format fmt;
	struct v4l2_buffer buf;
	struct v4l2_requestbuffers req;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = w; /* hint */
	fmt.fmt.pix.height = h; /* hint */
	fmt.fmt.pix.pixelformat = dev.format;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;

	if (!dev_ioctl(dev.fd, VIDIOC_S_FMT, &fmt)) {
		ee("v4l2_ioctl VIDIOC_S_FMT fd %d", dev.fd);
		return false;
	}

	if (fmt.fmt.pix.pixelformat != dev.format) {
		ee("v4l does not support RGB24");
		return false;
	}

	dev.frame.w = fmt.fmt.pix.width;
	dev.frame.h = fmt.fmt.pix.height;
	memset(&req, 0, sizeof(req));
	req.count = BUFFERS_CNT;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (!dev_ioctl(dev.fd, VIDIOC_REQBUFS, &req)) {
		ee("v4l2_ioctl VIDIOC_REQBUFS fd %d", dev.fd);
		return false;
	}

	if (!(dev.frame.buf = (struct buffer_view *) calloc(req.count,
	 sizeof(struct buffer_view)))) {
		return false;
	}

	nop("%u buffers in use fd %d", req.count, dev.fd);
	for (dev.frame.bufcnt = 0; dev.frame.bufcnt < req.count;
	 ++dev.frame.bufcnt) {
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = dev.frame.bufcnt;

		if (!dev_ioctl(dev.fd, VIDIOC_QUERYBUF, &buf)) {
			ee("v4l2_ioctl VIDIOC_QUERYBUF fd %d", dev.fd);
			return false;
		}

		dev.frame.buf[dev.frame.bufcnt].size = buf.length;
		dev.frame.buf[dev.frame.bufcnt].data = v4l2_mmap(NULL,
		 buf.length, PROT_READ, MAP_SHARED, dev.fd,
		 buf.m.offset);

		if (MAP_FAILED == dev.frame.buf[dev.frame.bufcnt].data) {
			ee("buf[%u] v4l2_mmap() failed", dev.frame.bufcnt);
			return false;
		}
	}

	for (uint8_t i = 0; i < dev.frame.bufcnt; ++i) {
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (!dev_ioctl(dev.fd, VIDIOC_QBUF, &buf)) {
			ee("v4l2_ioctl VIDIOC_QBUF fd %d", dev.fd);
			return false;
		}
	}

	set_framerate(dev);
	return true;
}

static stream_ptr create_stream(const char *path, uint16_t w, uint16_t h,
 uint32_t format)
{
	int fd;

	if (!path)
		return nullptr;
	else if ((fd = open_camera(path)) < 0)
		return nullptr;

	device *dev = new device(fd, format);
	if (!init_stream(*dev, w, h))
		return nullptr;

	stream *stream = new camera::stream(*dev);
	return std::make_unique<camera::stream>(std::move(*stream));
}

stream::stream(device &dev): dev_(dev) {}
stream::~stream() { delete &dev_; }

bool stream::start()
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (!dev_ioctl(dev_.fd, VIDIOC_STREAMON, &type)) {
		ee("v4l2_ioctl VIDIOC_STREAMON fd %d", dev_.fd);
		return false;
	}

	return true;
}

void stream::get_frame_size(uint16_t &w, uint16_t &h)
{
	w = dev_.frame.w;
	h = dev_.frame.h;
}

void stream::put_frame()
{
	if (dev_.buf.bytesused)
		dev_ioctl(dev_.fd, VIDIOC_QBUF, &dev_.buf);
	/* was not queued otherwise */
}

bool stream::get_frame(struct image &out)
{
	struct pollfd fds;

	dev_.buf.bytesused = 0; /* invalidate for put_frame call */
	fds.fd = dev_.fd;
	while (1) {
		fds.events = POLLIN;
		fds.revents = 0;

		int rc = poll(&fds, 1, POLL_TIMEOUT_MS);
		if (rc == 0) { // timeout
			break;
		} else if (rc < 0) {
			if (errno == EINTR)
				continue;

			ee("poll(%d) failed", fds.fd);
			return false;
		}

		if (!(fds.revents & POLLIN))
			continue;

		memset(&dev_.buf, 0, sizeof(dev_.buf));
		dev_.buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		dev_.buf.memory = V4L2_MEMORY_MMAP;

		if (!dev_ioctl(fds.fd, VIDIOC_DQBUF, &dev_.buf)) {
			ee("v4l2_ioctl VIDIOC_DQBUF fd %d", fds.fd);
			return false;
		}

		out.w = dev_.frame.w;
		out.h = dev_.frame.h;
		out.data = (uint8_t *) dev_.frame.buf[dev_.buf.index].data;
		out.bytes = dev_.buf.bytesused;
		out.id = dev_.buf.sequence;
		out.time.tv_sec = dev_.buf.timestamp.tv_sec;
		out.time.tv_nsec = dev_.buf.timestamp.tv_usec * 1000;
#ifdef PRINT_FPS
		out.flags = dev_.buf.flags;
		out.idx = dev_.buf.index;
#endif
		break;
	}

	return !!out.bytes;
}

stream_ptr create_rgb_stream(const char *path, uint16_t w, uint16_t h)
{
	return create_stream(path, w, h, V4L2_PIX_FMT_RGB24);
}

stream_ptr create_jpg_stream(const char *path, uint16_t w, uint16_t h)
{
	return create_stream(path, w, h, V4L2_PIX_FMT_MJPEG);
}

} // namespace camera
