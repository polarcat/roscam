/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#include "camera.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/qos.hpp"

#ifndef LOG_TAG
#define LOG_TAG "roscam"
#endif
#include "roslog.h"

#define STBI_NO_PNG
#define STBI_NO_BMP
#define STBI_NO_GIF
#define STBI_NO_PSD
#define STBI_NO_PIC
#define STBI_NO_PNM
#define STBI_NO_TGA
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

namespace {

constexpr uint8_t JPG_ON = (1 << 0);
constexpr uint8_t RGB_ON = (1 << 1);
constexpr uint8_t QOS_ON = (1 << 2);

struct params {
	const char *dev = nullptr;
	const char *topic = nullptr;
	uint16_t w = 640;
	uint16_t h = 480;
	float fps = 30.;
	bool qos = false;
	uint8_t flags = RGB_ON;
};

#ifdef PRINT_FPS
inline void print_fps(struct camera::image &img)
{
	float ms = img.time.tv_sec * 1000 + img.time.tv_nsec * .000001;
	float diff = ms - img.prev_ms;
	float fps = 1. / (diff * .001);
#if 1
	printf("\033[Gfps \033[1;33m%u\033[0m diff %u ms (%u bytes)\033[K",
	 (uint8_t) fps, (uint8_t) diff, img.bytes);
#else
	printf("[%u] fps \033[1;33m%u\033[0m diff %u ms (%u bytes) flags %#x\n",
	 img.idx, (uint8_t) fps, (uint8_t) diff, img.bytes, img.flags);
#endif
	img.prev_ms = ms;
}
#else
#define print_fps(img) ;
#endif

using std::placeholders::_1;
using image_t = sensor_msgs::msg::Image;

static constexpr uint8_t RGB_PLANES = 3;
static constexpr uint8_t MAX_SCALE_FACTOR = 5;

static inline void reset_image(camera::image *out)
{
	if (out->data) {
		free(out->data);
		out->data = nullptr;
		out->bytes = 0;
	}
}

static bool decode_image(camera::image *in, camera::image *out)
{
	int n = 0;
	int w = 0;
	int h = 0;

	out->data = stbi_load_from_memory(in->data, in->bytes, &w, &h, &n,
	 RGB_PLANES);

	if (n != RGB_PLANES) {
		ee("only RGB color scheme is supported, n=%d\n", n);
		reset_image(out);
		return false;
	} else if (!w || !h) {
		ee("invalid decoded image size wh (%d %d)\n", w, h);
		reset_image(out);
		return false;
	}

	out->w = static_cast<uint16_t>(w);
	out->h = static_cast<uint16_t>(h);
	return true;
}

inline void copy_rgb8(camera::image *in, image_t *out, uint8_t scale)
{
	uint16_t src_w = in->w * RGB_PLANES;
	uint16_t dst_w = out->width * RGB_PLANES;
	uint16_t dst_row = 0;

	for (uint16_t row = 0; row < in->h; ++row) {
		if (row % scale)
			continue;

		uint8_t *src = in->data + src_w * row;
		uint8_t *dst = out->data.data() + dst_w * dst_row++;
		for (uint16_t col = 0; col < in->w; ++col) {
			if (col % scale) {
				src += RGB_PLANES;
				continue;
			}

			*dst++ = *src++;
			*dst++ = *src++;
			*dst++ = *src++;
		}
	}
}

inline rclcpp::Time rostime(struct timespec &time)
{
	return rclcpp::Time(static_cast<int32_t>(time.tv_sec),
	 static_cast<uint32_t>(time.tv_nsec));
}

} /* namespace */

class roscam: public rclcpp::Node
{
	enum pixel_format {
		PIXEL_FMT_RGB8,
		PIXEL_FMT_JPG,
	};
public:
	roscam(std::unique_ptr<struct params>);
	~roscam();

private:
	void create_rgb_stream();
	void publish_rgb_image();
	void create_jpg_stream();
	void publish_jpg_image();
	void publish_decoded_image();
	std::thread job_;
	bool done_ = false;
	camera::stream_ptr stream_;
	uint16_t w_;
	uint16_t h_;
	int8_t scale_ = 1;
	enum pixel_format format_;
	const char *dev_;
	std::string topic_;
	rclcpp::QoS qos_ = rclcpp::QoS(1);
};

roscam::~roscam()
{
	done_ = true;
	if (job_.joinable())
		job_.join();

	ii("Stopped");
}

roscam::roscam(std::unique_ptr<struct params> p): Node(LOG_TAG)
{
	std::unique_ptr<struct params> params = std::move(p);

	dev_ = params->dev;
	topic_ = params->topic;
	w_ = params->w;
	h_ = params->h;

	ii("Open camera %s hinted geom (%u %u)", dev_, w_, h_);

	if ((params->flags & (JPG_ON | RGB_ON)) == (JPG_ON | RGB_ON))
		publish_decoded_image();
	else if (params->flags & RGB_ON)
		publish_rgb_image();
	else
		publish_jpg_image();

	ii("Topic '%s' geom (%u %u)", topic_.c_str(), w_, h_);

	if (params->flags & QOS_ON) {
		ii("Qos: volatile, reliable, depth=1\n");
		qos_.durability_volatile();
		qos_.keep_last(1);
		qos_.reliable();
	}
}

void roscam::create_jpg_stream()
{
	uint16_t w;
	uint16_t h;

	stream_ = camera::create_jpg_stream(dev_, w_, h_);
	if (!stream_.get()) {
		ee("Failed to create JPG stream");
		exit(1);
	}

	stream_->get_frame_size(w, h);
	if (w != w_ || h != h_) {
		ww("Requested geom (%u %u) is not supported;"
		 " using available geom (%u %u)", w_, h_, w, h);
		w_ = w;
		h_ = h;
		stream_.reset();
		stream_ = camera::create_jpg_stream(dev_, w_, h_);
		if (!stream_.get()) {
			ee("Failed to create JPG stream");
			exit(1);
		}
	}
}

void roscam::publish_decoded_image()
{
	create_jpg_stream();
	job_ = std::thread {
	 [=]()->void {
		camera::image in;
		rclcpp::Publisher<image_t>::SharedPtr rgb;

		if (!stream_->start()) {
			exit(1);
		} else if (!(rgb = create_publisher<image_t>(topic_, qos_))) {
			ee("Failed to create publisher for topic '%s'",
			 topic_.c_str());
			exit(1);
		}

		while (!done_) {
			stream_->get_frame(in);
			camera::image decoded;

			if (!decode_image(&in, &decoded)) {
				stream_->put_frame();
				continue;
			}

			image_t out;
			out.encoding = sensor_msgs::image_encodings::RGB8;
			out.header.stamp = rostime(in.time);
			out.header.frame_id = std::to_string(in.id);
			out.width = decoded.w;
			out.height = decoded.h;
			out.step = out.width;
			out.data.resize(out.width * out.height * RGB_PLANES);
			memcpy(out.data.data(), decoded.data, out.data.size());
			stream_->put_frame();
			rgb->publish(std::move(out));
			print_fps(in);
			reset_image(&decoded);
		}
	}};
}

void roscam::publish_jpg_image()
{
	create_jpg_stream();
	job_ = std::thread {
	 [=]()->void {
		camera::image in;
		rclcpp::Publisher<image_t>::SharedPtr jpg;

		if (!stream_->start()) {
			exit(1);
		} else if (!(jpg = create_publisher<image_t>(topic_, qos_))) {
			ee("Failed to create publisher for topic '%s'",
			 topic_.c_str());
			exit(1);
		}

		while (!done_) {
			stream_->get_frame(in);
			image_t out;
			out.encoding = "jpg";
			out.header.stamp = rostime(in.time);
			out.header.frame_id = std::to_string(in.id);
			out.data.resize(in.bytes);
			memcpy(out.data.data(), in.data, out.data.size());
			stream_->put_frame();
			jpg->publish(std::move(out));
			print_fps(in);
		}
	}};
}

void roscam::create_rgb_stream()
{
	uint16_t w;
	uint16_t h;

	stream_ = camera::create_rgb_stream(dev_, w_, h_);
	if (!stream_.get()) {
		ee("Failed to create RGB stream\n");
		exit(1);
	}

	stream_->get_frame_size(w, h);
	if (w != w_ || h != h_) {
		ww("Requested geom (%u %u) is not supported;"
		 " using available geom (%u %u)", w_, h_, w, h);
		w_ = w;
		h_ = h;
		stream_.reset();
		stream_ = camera::create_rgb_stream(dev_, w_, h_);
		if (!stream_.get()) {
			ee("Failed to create RGB stream");
			exit(1);
		}
	}

	w_ /= scale_;
	h_ /= scale_;
}

void roscam::publish_rgb_image()
{
	create_rgb_stream();
	job_ = std::thread {
	 [=]()->void {
		camera::image in;
		rclcpp::Publisher<image_t>::SharedPtr rgb;

		if (!stream_->start()) {
			exit(1);
		} else if (!(rgb = create_publisher<image_t>(topic_, qos_))) {
			ee("Failed to create publisher for topic '%s'",
			 topic_.c_str());
			exit(1);
		}

		while (!done_) {
			stream_->get_frame(in);
			image_t out;
			out.encoding = sensor_msgs::image_encodings::RGB8;
			out.header.stamp = rostime(in.time);
			out.header.frame_id = std::to_string(in.id);
			out.width = w_;
			out.height = h_;
			out.step = out.width;
			out.data.resize(out.width * out.height * RGB_PLANES);
			copy_rgb8(&in, &out, scale_);
			stream_->put_frame();
			rgb->publish(std::move(out));
			print_fps(in);
		}
	}};
}

static void help(const char *name)
{
	printf("Usage: %s <options>\n"
	 "Options:\n"
	 "\033[2m"
	 " -d, --dev <str>     video device, e.g. /dev/video0\n"
	 " -t, --topic <str>   topic name\n"
	 " -p, --params <str>  stream hints (WxH@fps), e.g. 1920x1080@30\n"
	 " -j, --jpg           select mjpeg input stream\n"
	 " -r, --rgb           publish rgb image (can be combined with -j)\n"
	 " -q, --qos           apply hardcoded QoS configuration\n"
	 "\033[0m"
	 "Examples: %s -d /dev/video0 -t /camera -p 1920x1080@30 -j -r\n",
	 name, name);
}

static int opt(const char *arg, const char *args, const char *argl)
{
	return (strcmp(arg, args) == 0 || strcmp(arg, argl) == 0);
}

static void init(int argc, char *argv[], struct params *params)
{
	const char *geom_w;
	const char *geom_h;
	const char *fps;
	const char *arg;

	for (uint8_t i = 0; i < argc; ++i) {
		arg = argv[i];
		if (opt(arg, "-d", "--dev")) {
			i++;
			params->dev = argv[i];
		} else if (opt(arg, "-t", "--topic")) {
			i++;
			params->topic = argv[i];
		} else if (opt(arg, "-p", "--params")) {
			i++;
			if (!(geom_w = argv[i])) {
				ee("malformed width, geom string e.g. 1920x1080\n");
				exit(1);
			} else if (!(geom_h = strchr(geom_w, 'x'))) {
				ee("malformed height, geom string e.g. 1920x1080\n");
				exit(1);
			} else if ((fps = strchr(geom_h, '@'))) {
				params->fps = atoi(++fps);
			}

			params->w = atoi(geom_w);
			params->h = atoi(++geom_h);
		} else if (opt(arg, "-j", "--jpg")) {
			params->flags |= JPG_ON;
		} else if (opt(arg, "-r", "--rgb")) {
			params->flags |= RGB_ON;
		} else if (opt(arg, "-q", "--qos")) {
			params->flags |= QOS_ON;
		}
	}

	if (!params->dev || !params->topic) {
		help(argv[0]);
		exit(1);
	}
}

int main(int argc, char *argv[])
{
	std::unique_ptr<struct params> params;
	params = std::make_unique<struct params>();
	init(argc, argv, params.get());
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<roscam>(std::move(params)));
	rclcpp::shutdown();
	return 0;
}
