/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/*!
 * @file
 * @brief  Oculus Rift S Driver Internal Interface
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#pragma once

#ifndef RIFT_S_H
#define RIFT_S_H

#include "util/u_logging.h"

struct rift_s_hmd;
struct rift_s_controller;

extern enum u_logging_level rift_s_log_level;

#define RIFT_S_TRACE(...) U_LOG_IFL_T(rift_s_log_level, __VA_ARGS__)
#define RIFT_S_DEBUG(...) U_LOG_IFL_D(rift_s_log_level, __VA_ARGS__)
#define RIFT_S_INFO(...) U_LOG_IFL_I(rift_s_log_level, __VA_ARGS__)
#define RIFT_S_WARN(...) U_LOG_IFL_W(rift_s_log_level, __VA_ARGS__)
#define RIFT_S_ERROR(...) U_LOG_IFL_E(rift_s_log_level, __VA_ARGS__)

#endif
