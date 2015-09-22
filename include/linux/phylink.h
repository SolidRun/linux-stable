#ifndef NETDEV_PCS_H
#define NETDEV_PCS_H

#include <linux/phy.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

struct device_node;
struct ethtool_cmd;
struct net_device;

enum {
	MLO_PAUSE_NONE,
	MLO_PAUSE_ASYM = BIT(0),
	MLO_PAUSE_SYM = BIT(1),

	MLO_AN_PHY = 0,
	MLO_AN_FIXED,
	MLO_AN_SGMII,
	MLO_AN_8023Z,
};

struct phylink_link_state {
	u32 supported;
	u32 advertising;
	u32 lp_advertising;
	int speed;
	int duplex;
	int pause;
	unsigned int link:1;
	unsigned int sync:1;
	unsigned int an_enabled:1;
	unsigned int an_complete:1;
};

struct phylink_mac_ops {
	/* Get the ethtool supported mask for the indicated mode */
	int (*mac_get_support)(struct net_device *, unsigned int mode,
			       struct phylink_link_state *);

	/* Read the current link state from the hardware */
	int (*mac_link_state)(struct net_device *, struct phylink_link_state *);

	/* Configure the MAC */
	void (*mac_config)(struct net_device *, unsigned int mode,
			   const struct phylink_link_state *);
	void (*mac_an_restart)(struct net_device *, unsigned int mode);

	void (*mac_link_down)(struct net_device *, unsigned int mode);
	void (*mac_link_up)(struct net_device *, unsigned int mode);
};

struct phylink *phylink_create(struct net_device *, struct device_node *,
	phy_interface_t iface, const struct phylink_mac_ops *ops);
void phylink_destroy(struct phylink *);

int phylink_connect_phy(struct phylink *, struct phy_device *);
int phylink_of_phy_connect(struct phylink *, struct device_node *);
void phylink_disconnect_phy(struct phylink *);

void phylink_mac_change(struct phylink *, bool up);

void phylink_start(struct phylink *);
void phylink_stop(struct phylink *);

int phylink_ethtool_get_settings(struct phylink *, struct ethtool_cmd *);
int phylink_ethtool_set_settings(struct phylink *, struct ethtool_cmd *);
int phylink_mii_ioctl(struct phylink *, struct ifreq *, int);

#endif
