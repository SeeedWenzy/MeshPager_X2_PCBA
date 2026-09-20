/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * register console command: ping
 */
void ping_cmd_register_ping(void);
void ping_cmd_user(char *addr, uint8_t cnt);

#ifdef __cplusplus
}
#endif
