/*
 * Copyright (c) 2021 Matias Nitsche
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pico.h"
#include "pico/bootrom.h"

void rp2_usb_reset(void)
{
  reset_usb_boot(0, 0);
}
