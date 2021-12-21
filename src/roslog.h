/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#ifndef ROSLOG_H
#define ROSLOG_H

#include "rclcpp/rclcpp.hpp"

#define rlog rclcpp::get_logger(LOG_TAG)
#define ii(...) RCLCPP_INFO(rlog, __VA_ARGS__)
#define ww(...) RCLCPP_WARN(rlog, __VA_ARGS__)
#define ee(...) RCLCPP_ERROR(rlog, __VA_ARGS__)
#define nop(...) ;

#endif /* ROSLOG_H */
