/*
 * phylink models the MAC to optional PHY connection, supporting
 * technologies such as SFP cages where the PHY is hot-pluggable.
 *
 * Copyright (C) 2015 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/ethtool.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/phylink.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "swphy.h"

#define SUPPORTED_INTERFACES \
	(SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_FIBRE | \
	 SUPPORTED_BNC | SUPPORTED_AUI | SUPPORTED_Backplane)
#define ADVERTISED_INTERFACES \
	(ADVERTISED_TP | ADVERTISED_MII | ADVERTISED_FIBRE | \
	 ADVERTISED_BNC | ADVERTISED_AUI | ADVERTISED_Backplane)

static LIST_HEAD(phylinks);
static DEFINE_MUTEX(phylink_mutex);

enum {
	PHYLINK_DISABLE_STOPPED,
	PHYLINK_DISABLE_LINK,
};

struct phylink {
	struct list_head node;
	struct net_device *netdev;
	const struct phylink_mac_ops *ops;
	struct mutex config_mutex;

	unsigned long phylink_disable_state; /* bitmask of disables */
	struct phy_device *phydev;
	phy_interface_t link_interface;	/* PHY_INTERFACE_xxx */
	u8 link_an_mode;		/* MLO_AN_xxx */
	u8 link_port;			/* The current non-phy ethtool port */
	u32 link_port_support;		/* SUPPORTED_xxx ethtool for ports */

	/* The link configuration settings */
	struct phylink_link_state link_config;
	struct gpio_desc *link_gpio;

	struct mutex state_mutex;	/* may be taken within config_mutex */
	struct phylink_link_state phy_state;
	struct work_struct resolve;

	bool mac_link_up;
};

static const char *phylink_an_mode_str(unsigned int mode)
{
	static const char *modestr[] = {
		[MLO_AN_PHY] = "phy",
		[MLO_AN_FIXED] = "fixed",
		[MLO_AN_SGMII] = "sgmii",
		[MLO_AN_8023Z] = "802.3z",
	};

	return mode < ARRAY_SIZE(modestr) ? modestr[mode] : "unknown";
}

static int phylink_parse_fixedlink(struct phylink *pl, struct device_node *np)
{
	struct device_node *fixed_node;
	int ret, len;

	fixed_node = of_get_child_by_name(np, "fixed-link");
	if (fixed_node) {
		struct gpio_desc *desc;
		u32 speed;

		ret = of_property_read_u32(fixed_node, "speed", &speed);

		pl->link_an_mode = MLO_AN_FIXED;
		pl->link_config.link = 1;
		pl->link_config.an_complete = 1;
		pl->link_config.speed = speed;
		pl->link_config.duplex = DUPLEX_HALF;
		pl->link_config.pause = MLO_PAUSE_NONE;

		if (of_property_read_bool(fixed_node, "full-duplex"))
			pl->link_config.duplex = DUPLEX_FULL;
		if (of_property_read_bool(fixed_node, "pause"))
			pl->link_config.pause |= MLO_PAUSE_SYM;
		if (of_property_read_bool(fixed_node, "asym-pause"))
			pl->link_config.pause |= MLO_PAUSE_ASYM;

		if (ret == 0) {
			desc = fwnode_get_named_gpiod(&fixed_node->fwnode,
						      "link-gpios");

			if (!IS_ERR(desc))
				pl->link_gpio = desc;
			else if (desc == ERR_PTR(-EPROBE_DEFER))
				ret = -EPROBE_DEFER;
		}
		of_node_put(fixed_node);
	} else {
		const __be32 *fixed_prop;

		fixed_prop = of_get_property(np, "fixed-link", &len);
		if (fixed_prop && len == 5 * sizeof(*fixed_prop)) {
			pl->link_config.duplex = be32_to_cpu(fixed_prop[1]) ?
						DUPLEX_FULL : DUPLEX_HALF;
			pl->link_config.speed = be32_to_cpu(fixed_prop[2]);
			pl->link_config.pause = MLO_PAUSE_NONE;
			if (be32_to_cpu(fixed_prop[3]))
				pl->link_config.pause |= MLO_PAUSE_SYM;
			if (be32_to_cpu(fixed_prop[4]))
				pl->link_config.pause |= MLO_PAUSE_ASYM;

			pl->link_an_mode = MLO_AN_FIXED;
		}
		ret = 0;
	}

	if (pl->link_an_mode == MLO_AN_FIXED) {
		/* Generate the supported/advertising masks */
		if (pl->link_config.pause & MLO_PAUSE_SYM) {
			pl->link_config.supported |= SUPPORTED_Pause;
			pl->link_config.advertising |= ADVERTISED_Pause;
		}
		if (pl->link_config.pause & MLO_PAUSE_ASYM) {
			pl->link_config.supported |= SUPPORTED_Asym_Pause;
			pl->link_config.advertising |= ADVERTISED_Asym_Pause;
		}

		if (pl->link_config.speed > SPEED_1000 &&
		    pl->link_config.duplex != DUPLEX_FULL)
			netdev_warn(pl->netdev, "fixed link specifies half duplex for %dMbps link?\n",
				    pl->link_config.speed);

#define S(spd) \
	pl->link_config.supported |= pl->link_config.duplex ? \
		SUPPORTED_##spd##_Full : SUPPORTED_##spd##_Half
#define A(spd) \
	pl->link_config.advertising |= pl->link_config.duplex ? \
		ADVERTISED_##spd##_Full : ADVERTISED_##spd##_Half
#define C(spd, tech) \
		case spd: \
			S(spd##tech); \
			A(spd##tech); \
			break
		switch (pl->link_config.speed) {
		C(10, baseT);
		C(100, baseT);
		C(1000, baseT);
#undef S
#undef A
#define S(spd) pl->link_config.supported |= SUPPORTED_##spd##_Full
#define A(spd) pl->link_config.advertising |= ADVERTISED_##spd##_Full
		C(2500, baseX);
		C(10000, baseT);
		}
#undef S
#undef A
#undef C
	}
	return ret;
}

static int phylink_parse_managed(struct phylink *pl, struct device_node *np)
{
	const char *managed;

	if (of_property_read_string(np, "managed", &managed) == 0) {
		if (strcmp(managed, "in-band-status") == 0) {
			if (pl->link_an_mode == MLO_AN_FIXED) {
				netdev_err(pl->netdev, "can't use both fixed-link and in-band-status\n");
				return -EINVAL;
			}
			pl->link_an_mode = MLO_AN_SGMII;
			pl->link_config.an_enabled = true;
		}
	}

	return 0;
}


static int phylink_get_mac_state(struct phylink *pl, struct phylink_link_state *state)
{
	struct net_device *ndev = pl->netdev;

	state->supported = pl->link_config.supported;
	state->advertising = pl->link_config.advertising;
	state->an_enabled = pl->link_config.an_enabled;
	state->link = 1;
	state->sync = 1;

	return pl->ops->mac_link_state(ndev, state);
}

/* The fixed state is... fixed except for the link state,
 * which may be determined by a GPIO.
 */
static void phylink_get_fixed_state(struct phylink *pl, struct phylink_link_state *state)
{
	*state = pl->link_config;
	if (pl->link_gpio)
		state->link = !!gpiod_get_value(pl->link_gpio);
}

extern const char *phy_speed_to_str(int speed);

static void phylink_resolve(struct work_struct *w)
{
	struct phylink *pl = container_of(w, struct phylink, resolve);
	struct phylink_link_state link_state;
	struct net_device *ndev = pl->netdev;

	mutex_lock(&pl->state_mutex);
	if (pl->phylink_disable_state) {
		link_state.link = false;
	} else {
		switch (pl->link_an_mode) {
		case MLO_AN_PHY:
			link_state = pl->phy_state;
			break;

		case MLO_AN_FIXED:
			phylink_get_fixed_state(pl, &link_state);
			break;

		case MLO_AN_SGMII:
			/* This should be the logical and of phy up and mac up */
		case MLO_AN_8023Z:
			phylink_get_mac_state(pl, &link_state);
			if (pl->phydev)
				link_state.link = link_state.link &&
						  pl->phy_state.link;
			break;
		}
	}

	if (link_state.link != netif_carrier_ok(ndev)) {
		if (!link_state.link) {
			netif_carrier_off(ndev);
			pl->ops->mac_link_down(ndev, pl->link_an_mode);
			netdev_info(ndev, "Link is Down\n");
		} else {
			/* If we're using PHY autonegotiation, we need to keep
			 * the MAC updated with the current link parameters.
			 */
			if (pl->link_an_mode == MLO_AN_PHY)
				pl->ops->mac_config(ndev, MLO_AN_PHY, &link_state);

			pl->ops->mac_link_up(ndev, pl->link_an_mode);

			netif_carrier_on(ndev);

			netdev_info(ndev,
				    "Link is Up - %s/%s - flow control %s\n",
				    phy_speed_to_str(link_state.speed),
				    link_state.duplex ? "Full" : "Half",
				    link_state.pause ? "rx/tx" : "off");
		}
	}
	mutex_unlock(&pl->state_mutex);
}

static void phylink_run_resolve(struct phylink *pl)
{
	if (!pl->phylink_disable_state)
		queue_work(system_power_efficient_wq, &pl->resolve);
}

struct phylink *phylink_create(struct net_device *ndev, struct device_node *np,
	phy_interface_t iface, const struct phylink_mac_ops *ops)
{
	struct phylink *pl;
	int ret;

	pl = kzalloc(sizeof(*pl), GFP_KERNEL);
	if (!pl)
		return ERR_PTR(-ENOMEM);

	mutex_init(&pl->state_mutex);
	mutex_init(&pl->config_mutex);
	INIT_WORK(&pl->resolve, phylink_resolve);
	pl->netdev = ndev;
	pl->link_interface = iface;
	pl->link_port_support = SUPPORTED_MII;
	pl->link_port = PORT_MII;
	pl->ops = ops;
	__set_bit(PHYLINK_DISABLE_STOPPED, &pl->phylink_disable_state);

	ret = phylink_parse_fixedlink(pl, np);
	if (ret < 0) {
		kfree(pl);
		return ERR_PTR(ret);
	}

	ret = phylink_parse_managed(pl, np);
	if (ret < 0) {
		kfree(pl);
		return ERR_PTR(ret);
	}

	ret = pl->ops->mac_get_support(pl->netdev, pl->link_an_mode,
				       &pl->link_config);
	if (ret) {
		kfree(pl);
		return ERR_PTR(ret);
	}

	mutex_lock(&phylink_mutex);
	list_add_tail(&pl->node, &phylinks);
	mutex_unlock(&phylink_mutex);

	return pl;
}
EXPORT_SYMBOL_GPL(phylink_create);

void phylink_destroy(struct phylink *pl)
{
	mutex_lock(&phylink_mutex);
	list_del(&pl->node);
	mutex_unlock(&phylink_mutex);

	cancel_work_sync(&pl->resolve);
	kfree(pl);
}
EXPORT_SYMBOL_GPL(phylink_destroy);

void phylink_phy_change(struct phy_device *phy, bool up, bool do_carrier)
{
	struct phylink *pl = phy->phylink;

	mutex_lock(&pl->state_mutex);
	pl->phy_state.speed = phy->speed;
	pl->phy_state.duplex = phy->duplex;
	pl->phy_state.pause = MLO_PAUSE_NONE;
	if (phy->pause)
		pl->phy_state.pause |= MLO_PAUSE_SYM;
	if (phy->asym_pause)
		pl->phy_state.pause |= MLO_PAUSE_ASYM;
	pl->phy_state.link = up;
	mutex_unlock(&pl->state_mutex);

	phylink_run_resolve(pl);

	netdev_dbg(pl->netdev, "phy link %s\n", up ? "up" : "down");
}

static int phylink_bringup_phy(struct phylink *pl, struct phy_device *phy)
{
	mutex_lock(&pl->config_mutex);
	phy->phylink = pl;
	phy->phy_link_change = phylink_phy_change;

	netdev_info(pl->netdev,
		    "PHY [%s] driver [%s]\n", dev_name(&phy->dev),
		    phy->drv->name);

	mutex_lock(&pl->state_mutex);
	pl->phydev = phy;

	/* Restrict the phy advertisment to the union of the PHY and
	 * MAC-level advert.
	 */
	phy->advertising &= ADVERTISED_INTERFACES |
			    pl->link_config.advertising;
	mutex_unlock(&pl->state_mutex);

	phy_start_machine(phy);
	if (phy->irq > 0)
		phy_start_interrupts(phy);

	mutex_unlock(&pl->config_mutex);

	return 0;
}

int phylink_connect_phy(struct phylink *pl, struct phy_device *phy)
{
	int ret;

	ret = phy_attach_direct(pl->netdev, phy, 0, pl->link_interface);
	if (ret)
		return ret;

	ret = phylink_bringup_phy(pl, phy);
	if (ret)
		phy_detach(phy);

	return ret;
}
EXPORT_SYMBOL_GPL(phylink_connect_phy);

int phylink_of_phy_connect(struct phylink *pl, struct device_node *dn)
{
	struct device_node *phy_node;
	struct phy_device *phy_dev;
	int ret;

	/* Fixed links are handled without needing a PHY */
	if (pl->link_an_mode == MLO_AN_FIXED)
		return 0;

	phy_node = of_parse_phandle(dn, "phy-handle", 0);
	if (!phy_node)
		phy_node = of_parse_phandle(dn, "phy", 0);
	if (!phy_node)
		phy_node = of_parse_phandle(dn, "phy-device", 0);

	if (!phy_node) {
		if (pl->link_an_mode == MLO_AN_PHY) {
			netdev_err(pl->netdev, "unable to find PHY node\n");
			return -ENODEV;
		}
		return 0;
	}

	phy_dev = of_phy_attach(pl->netdev, phy_node, 0, pl->link_interface);
	/* We're done with the phy_node handle */
	of_node_put(phy_node);

	if (!phy_dev)
		return -ENODEV;

	ret = phylink_bringup_phy(pl, phy_dev);
	if (ret)
		phy_detach(phy_dev);

	return ret;
}
EXPORT_SYMBOL_GPL(phylink_of_phy_connect);

void phylink_disconnect_phy(struct phylink *pl)
{
	struct phy_device *phy;

	mutex_lock(&pl->config_mutex);
	phy = pl->phydev;

	mutex_lock(&pl->state_mutex);
	pl->phydev = NULL;
	mutex_unlock(&pl->state_mutex);
	flush_work(&pl->resolve);

	if (phy)
		phy_disconnect(phy);

	mutex_unlock(&pl->config_mutex);
}
EXPORT_SYMBOL_GPL(phylink_disconnect_phy);

void phylink_mac_change(struct phylink *pl, bool up)
{
	phylink_run_resolve(pl);
	netdev_dbg(pl->netdev, "mac link %s\n", up ? "up" : "down");
}
EXPORT_SYMBOL_GPL(phylink_mac_change);

void phylink_start(struct phylink *pl)
{
	mutex_lock(&pl->config_mutex);

	netdev_info(pl->netdev, "configuring for link AN mode %s\n",
		    phylink_an_mode_str(pl->link_an_mode));

	/* Apply the link configuration to the MAC when starting. This allows
	 * a fixed-link to start with the correct parameters, and also
	 * ensures that we set the appropriate advertisment for Serdes links.
	 */
	pl->ops->mac_config(pl->netdev, pl->link_an_mode, &pl->link_config);

	clear_bit(PHYLINK_DISABLE_STOPPED, &pl->phylink_disable_state);
	phylink_run_resolve(pl);

	if (pl->phydev)
		phy_start(pl->phydev);

	mutex_unlock(&pl->config_mutex);
}
EXPORT_SYMBOL_GPL(phylink_start);

void phylink_stop(struct phylink *pl)
{
	mutex_lock(&pl->config_mutex);

	if (pl->phydev)
		phy_stop(pl->phydev);

	set_bit(PHYLINK_DISABLE_STOPPED, &pl->phylink_disable_state);
	flush_work(&pl->resolve);

	pl->mac_link_up = false;

	mutex_unlock(&pl->config_mutex);
}
EXPORT_SYMBOL_GPL(phylink_stop);

static void phylink_get_ethtool(const struct phylink_link_state *state,
				struct ethtool_cmd *cmd)
{
	cmd->supported &= SUPPORTED_INTERFACES;
	cmd->supported |= state->supported;
	cmd->advertising &= ADVERTISED_INTERFACES;
	cmd->advertising |= state->advertising;
	ethtool_cmd_speed_set(cmd, state->speed);
	cmd->duplex = state->duplex;

	cmd->autoneg = state->an_enabled ? AUTONEG_ENABLE : AUTONEG_DISABLE;
}

static int phylink_ethtool_gset(struct phylink *pl, struct ethtool_cmd *cmd)
{
	struct phylink_link_state link_state;
	int ret;

	if (pl->phydev) {
		ret = phy_ethtool_gset(pl->phydev, cmd);
		if (ret)
			return ret;

		cmd->supported &= SUPPORTED_INTERFACES |
				  pl->link_config.supported;
	} else {
		cmd->supported = pl->link_port_support;
		cmd->transceiver = XCVR_EXTERNAL;
		cmd->port = pl->link_port;
	}

	switch (pl->link_an_mode) {
	case MLO_AN_FIXED:
		/* We are using fixed settings. Report these as the
		 * current link settings - and note that these also
		 * represent the supported speeds/duplex/pause modes.
		 */
		phylink_get_fixed_state(pl, &link_state);
		phylink_get_ethtool(&link_state, cmd);
		break;

	case MLO_AN_SGMII:
		/* If there is a phy attached, then use the reported
		 * settings from the phy with no modification.
		 */
		if (pl->phydev)
			break;

	case MLO_AN_8023Z:
		phylink_get_mac_state(pl, &link_state);

		/* The MAC is reporting the link results from its own PCS
		 * layer via in-band status. Report these as the current
		 * link settings.
		 */
		phylink_get_ethtool(&link_state, cmd);
		break;
	}

	return 0;
}

int phylink_ethtool_get_settings(struct phylink *pl, struct ethtool_cmd *cmd)
{
	int ret;

	mutex_lock(&pl->config_mutex);
	ret = phylink_ethtool_gset(pl, cmd);
	mutex_unlock(&pl->config_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(phylink_ethtool_get_settings);

static int phylink_ethtool_sset(struct phylink *pl, struct ethtool_cmd *cmd)
{
	u32 supported;
	int ret;

	/* Calculate the union of the MAC support and attached phy support */
	supported = pl->link_config.supported;
	if (pl->phydev)
		supported &= pl->phydev->supported;

	/* Mask out unsupported advertisments */
	cmd->advertising &= supported;

	/* FIXME: should we reject autoneg if phy/mac does not support it? */

	if (cmd->autoneg == AUTONEG_DISABLE) {
		/* Autonegotiation disabled, validate speed and duplex */
		if (cmd->duplex != DUPLEX_HALF && cmd->duplex != DUPLEX_FULL)
			return -EINVAL;

		/* FIXME: validate speed/duplex against supported */

		cmd->advertising &= ~ADVERTISED_Autoneg;
	} else {
		/* Autonegotiation enabled, validate advertisment */
		/* FIXME: shouldn't we ensure there's some duplex/speeds set */
		if (cmd->advertising == 0)
			return -EINVAL;

		cmd->advertising |= ADVERTISED_Autoneg;
	}

	/* If we have a fixed link (as specified by firmware), refuse
	 * to enable autonegotiation, or change link parameters.
	 */
	if (pl->link_an_mode == MLO_AN_FIXED) {
		if (cmd->autoneg != AUTONEG_DISABLE ||
		    ethtool_cmd_speed(cmd) != pl->link_config.speed ||
		    cmd->duplex != pl->link_config.duplex)
			return -EINVAL;
	}

	/* If we have a PHY, configure the phy */
	if (pl->phydev) {
		ret = phy_ethtool_sset(pl->phydev, cmd);
		if (ret)
			return ret;
	}

	mutex_lock(&pl->state_mutex);
	/* Configure the MAC to match the new settings */
	pl->link_config.advertising = cmd->advertising;
	pl->link_config.speed = cmd->speed;
	pl->link_config.duplex = cmd->duplex;
	pl->link_config.an_enabled = cmd->autoneg != AUTONEG_DISABLE;

	pl->ops->mac_config(pl->netdev, pl->link_an_mode, &pl->link_config);
	pl->ops->mac_an_restart(pl->netdev, pl->link_an_mode);
	mutex_unlock(&pl->state_mutex);

	return ret;
}

int phylink_ethtool_set_settings(struct phylink *pl, struct ethtool_cmd *cmd)
{
	int ret;

	if (cmd->autoneg != AUTONEG_DISABLE && cmd->autoneg != AUTONEG_ENABLE)
		return -EINVAL;

	mutex_lock(&pl->config_mutex);
	ret = phylink_ethtool_sset(pl, cmd);
	mutex_unlock(&pl->config_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(phylink_ethtool_set_settings);

/* This emulates MII registers for a fixed-mode phy operating as per the
 * passed in state. "aneg" defines if we report negotiation is possible.
 *
 * FIXME: should deal with negotiation state too.
 */
static int phylink_mii_emul_read(struct net_device *ndev, unsigned int reg,
				 struct phylink_link_state *state, bool aneg)
{
	struct fixed_phy_status fs;
	int val;

	fs.link = state->link;
	fs.speed = state->speed;
	fs.duplex = state->duplex;
	fs.pause = state->pause & MLO_PAUSE_SYM;
	fs.asym_pause = state->pause & MLO_PAUSE_ASYM;

	val = swphy_read_reg(reg, &fs);
	if (reg == MII_BMSR) {
		if (!state->an_complete)
			val &= ~BMSR_ANEGCOMPLETE;
		if (!aneg)
			val &= ~BMSR_ANEGCAPABLE;
	}
	return val;
}

static int phylink_mii_read(struct phylink *pl, unsigned int phy_id,
			    unsigned int reg)
{
	struct phylink_link_state state;
	int val = 0xffff;

	if (pl->phydev && pl->phydev->addr != phy_id)
		return mdiobus_read(pl->phydev->bus, phy_id, reg);

	if (!pl->phydev && phy_id != 0)
		return val;

	switch (pl->link_an_mode) {
	case MLO_AN_FIXED:
		phylink_get_fixed_state(pl, &state);
		val = phylink_mii_emul_read(pl->netdev, reg, &state, true);
		break;

	case MLO_AN_PHY:
		val = mdiobus_read(pl->phydev->bus, phy_id, reg);
		break;

	case MLO_AN_SGMII:
		if (pl->phydev) {
			val = mdiobus_read(pl->phydev->bus, pl->phydev->addr,
					   reg);
			break;
		}
		/* No phy, fall through to reading the MAC end */
	case MLO_AN_8023Z:
		val = phylink_get_mac_state(pl, &state);
		if (val < 0)
			return val;

		val = phylink_mii_emul_read(pl->netdev, reg, &state, true);
		break;
	}

	return val & 0xffff;
}

static void phylink_mii_write(struct phylink *pl, unsigned int phy_id,
			      unsigned int reg, unsigned int val)
{
	if (pl->phydev && pl->phydev->addr != phy_id) {
		mdiobus_write(pl->phydev->bus, phy_id, reg, val);
		return;
	}

	if (!pl->phydev && phy_id != 0)
		return;

	switch (pl->link_an_mode) {
	case MLO_AN_FIXED:
		break;

	case MLO_AN_PHY:
		mdiobus_write(pl->phydev->bus, pl->phydev->addr, reg, val);
		break;

	case MLO_AN_SGMII:
		if (pl->phydev) {
			mdiobus_write(pl->phydev->bus, phy_id, reg, val);
			break;
		}
		/* No phy, fall through to reading the MAC end */
	case MLO_AN_8023Z:
		break;
	}
}

int phylink_mii_ioctl(struct phylink *pl, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *mii_data = if_mii(ifr);
	int val, ret;

	mutex_lock(&pl->config_mutex);

	switch (cmd) {
	case SIOCGMIIPHY:
		mii_data->phy_id = pl->phydev ? pl->phydev->addr : 0;
		/* fallthrough */

	case SIOCGMIIREG:
		val = phylink_mii_read(pl, mii_data->phy_id, mii_data->reg_num);
		if (val < 0) {
			ret = val;
		} else {
			mii_data->val_out = val;
			ret = 0;
		}
		break;

	case SIOCSMIIREG:
		phylink_mii_write(pl, mii_data->phy_id, mii_data->reg_num,
				  mii_data->val_in);
		ret = 0;
		break;

	default:
		ret = -EOPNOTSUPP;
		if (pl->phydev)
			ret = phy_mii_ioctl(pl->phydev, ifr, cmd);
		break;
	}

	mutex_unlock(&pl->config_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(phylink_mii_ioctl);



void phylink_disable(struct phylink *pl)
{
	set_bit(PHYLINK_DISABLE_LINK, &pl->phylink_disable_state);
	flush_work(&pl->resolve);

	netif_carrier_off(pl->netdev);
}
EXPORT_SYMBOL_GPL(phylink_disable);

void phylink_enable(struct phylink *pl)
{
	clear_bit(PHYLINK_DISABLE_LINK, &pl->phylink_disable_state);
	phylink_run_resolve(pl);
}
EXPORT_SYMBOL_GPL(phylink_enable);

void phylink_set_link_port(struct phylink *pl, u32 support, u8 port)
{
	WARN_ON(support & ~SUPPORTED_INTERFACES);

	mutex_lock(&pl->config_mutex);
	pl->link_port_support = support;
	pl->link_port = port;
	mutex_unlock(&pl->config_mutex);
}
EXPORT_SYMBOL_GPL(phylink_set_link_port);

int phylink_set_link_an_mode(struct phylink *pl, unsigned int mode)
{
	struct phylink_link_state state;
	int ret = 0;

	mutex_lock(&pl->config_mutex);
	if (pl->link_an_mode != mode) {
		netdev_info(pl->netdev, "switching to link AN mode %s\n",
			    phylink_an_mode_str(mode));

		state = pl->link_config;
		ret = pl->ops->mac_get_support(pl->netdev, mode, &state);
		if (ret == 0) {
			pl->link_an_mode = mode;
			pl->link_config = state;

			if (!test_bit(PHYLINK_DISABLE_STOPPED,
				      &pl->phylink_disable_state))
				pl->ops->mac_config(pl->netdev,
						    pl->link_an_mode,
						    &pl->link_config);
		}
	}
	mutex_unlock(&pl->config_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(phylink_set_link_an_mode);

struct phylink *phylink_lookup_by_netdev(struct net_device *ndev)
{
	struct phylink *pl, *found = NULL;

	mutex_lock(&phylink_mutex);
	list_for_each_entry(pl, &phylinks, node)
		if (pl->netdev == ndev) {
			found = pl;
			break;
		}

	mutex_unlock(&phylink_mutex);

	return found;
}
EXPORT_SYMBOL_GPL(phylink_lookup_by_netdev);
