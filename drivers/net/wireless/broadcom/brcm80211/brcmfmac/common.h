// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2014 Broadcom Corporation
 */
#ifndef BRCMFMAC_COMMON_H
#define BRCMFMAC_COMMON_H

#include <linux/platform_device.h>
#include <linux/platform_data/brcmfmac.h>
#include "fwil_types.h"

#define BRCMF_FW_ALTPATH_LEN			256

/* Definitions for the module global and device specific settings are defined
 * here. Two structs are used for them. brcmf_mp_global_t and brcmf_mp_device.
 * The mp_global is instantiated once in a global struct and gets initialized
 * by the common_attach function which should be called before any other
 * (module) initiliazation takes place. The device specific settings is part
 * of the drvr struct and should be initialized on every brcmf_attach.
 */

/**
 * struct brcmf_mp_global_t - Global module paramaters.
 *
 * @firmware_path: Alternative firmware path.
 */
struct brcmf_mp_global_t {
	char	firmware_path[BRCMF_FW_ALTPATH_LEN];
};

extern struct brcmf_mp_global_t brcmf_mp_global;

/**
 * struct brcmf_module_data - Fixed settings of a module, found by its bus IDs.
 *
 * @ios_setup: Firmware set up as Apple's iOS driver does it: its iovars and
 *	Bluetooth coexistence values, no frame bursting, and wake-on-WLAN by
 *	packet filters and the host wake line (the firmware has no "wowl").
 * @roamoff: Firmware roaming off.
 * @keep_link_s2idle: Stay associated through suspend to idle.
 * @cis_mac: The MAC address is in the SDIO CIS only, not in the NVRAM.
 * @pm2_sleep_ret: PM2 return-to-sleep time (ms) when none is given; 0: the
 *	driver's default.
 * @sdio_clock: SDIO clock (Hz) for a card whose CIS gives no transfer speed.
 * @sdio_idle_ms: Idle time (ms) before the SDIO bus releases the HT clock.
 */
struct brcmf_module_data {
	bool	ios_setup;
	bool	roamoff;
	bool	keep_link_s2idle;
	bool	cis_mac;
	u32	pm2_sleep_ret;
	u32	sdio_clock;
	u32	sdio_idle_ms;
};

/**
 * struct brcmf_mp_device - Device module paramaters.
 *
 * @p2p_enable: Legacy P2P0 enable (old wpa_supplicant).
 * @feature_disable: Feature_disable bitmask.
 * @fcmode: FWS flow control.
 * @roamoff: Firmware roaming off?
 * @ignore_probe_fail: Ignore probe failure.
 * @trivial_ccode_map: Assume firmware uses ISO3166 country codes with rev 0
 * @country_codes: If available, pointer to struct for translating country codes
 * @bus: Bus specific platform data. Only SDIO at the mmoment.
 * @module: Fixed settings of this module, NULL for most.
 */
struct brcmf_mp_device {
	bool		p2p_enable;
	unsigned int	feature_disable;
	int		fcmode;
	bool		roamoff;
	bool		iapp;
	bool		ignore_probe_fail;
	bool		trivial_ccode_map;
	struct brcmfmac_pd_cc *country_codes;
	const char	*board_type;
	unsigned char	mac[ETH_ALEN];
	const char	*antenna_sku;
	const void	*cal_blob;
	int		cal_size;
	union {
		struct brcmfmac_sdio_pd sdio;
	} bus;
	const struct brcmf_module_data *module;
};

void brcmf_c_set_joinpref_default(struct brcmf_if *ifp);

struct brcmf_mp_device *brcmf_get_module_param(struct device *dev,
					       enum brcmf_bus_type bus_type,
					       u32 chip, u32 chiprev);
void brcmf_release_module_param(struct brcmf_mp_device *module_param);

/* Sets dongle media info (drv_version, mac address). */
int brcmf_c_preinit_dcmds(struct brcmf_if *ifp);
int brcmf_c_set_cur_etheraddr(struct brcmf_if *ifp, const u8 *addr);

#ifdef CONFIG_DMI
void brcmf_dmi_probe(struct brcmf_mp_device *settings, u32 chip, u32 chiprev);
#else
static inline void
brcmf_dmi_probe(struct brcmf_mp_device *settings, u32 chip, u32 chiprev) {}
#endif

#ifdef CONFIG_ACPI
void brcmf_acpi_probe(struct device *dev, enum brcmf_bus_type bus_type,
		      struct brcmf_mp_device *settings);
#else
static inline void brcmf_acpi_probe(struct device *dev,
				    enum brcmf_bus_type bus_type,
				    struct brcmf_mp_device *settings) {}
#endif

u8 brcmf_map_prio_to_prec(void *cfg, u8 prio);

u8 brcmf_map_prio_to_aci(void *cfg, u8 prio);

#endif /* BRCMFMAC_COMMON_H */
