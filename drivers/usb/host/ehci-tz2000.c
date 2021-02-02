/*
 * Toshiba TZ2000 USB EHCI support
 *
 * Copyright (C) 2013,2014 Toshiba Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <usb.h>

#include "ehci.h"

int ehci_hcd_init(int index, struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	*hccr = (struct ehci_hccr *)TZ2000_USBHC_BASE;
	*hcor = (struct ehci_hcor *)((uint32_t)*hccr +
			HC_LENGTH(ehci_readl(&(*hccr)->cr_capbase)));
	return 0;
}

int ehci_hcd_stop(int index)
{
	return 0;
}
