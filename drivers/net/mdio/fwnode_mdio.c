// SPDX-License-Identifier: GPL-2.0-only
/*
 * fwnode helpers for the MDIO (Ethernet PHY) API
 *
 * This file provides helper functions for extracting PHY device information
 * out of the fwnode and using it to populate an mii_bus.
 */

#include <linux/acpi.h>
#include <linux/acpi_mdio.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>

MODULE_AUTHOR("Calvin Johnson <calvin.johnson@oss.nxp.com>");
MODULE_LICENSE("GPL");

int fwnode_mdiobus_register_phy(struct mii_bus *bus,
				struct fwnode_handle *child, u32 addr)
{
	struct mii_timestamper *mii_ts = NULL;
	struct phy_device *phy;
	bool is_c45 = false;
	u32 phy_id;
	int rc;

	if (is_of_node(child)) {
		mii_ts = of_find_mii_timestamper(to_of_node(child));
		if (IS_ERR(mii_ts))
			return PTR_ERR(mii_ts);
	}

	rc = fwnode_property_match_string(child, "compatible", "ethernet-phy-ieee802.3-c45");
	if (rc >= 0)
		is_c45 = true;

	if (is_c45 || fwnode_get_phy_id(child, &phy_id))
		phy = get_phy_device(bus, addr, is_c45);
	else
		phy = phy_device_create(bus, addr, phy_id, 0, NULL);
	if (IS_ERR(phy)) {
		unregister_mii_timestamper(mii_ts);
		return PTR_ERR(phy);
	}

	if (is_acpi_node(child)) {
		phy->irq = bus->irq[addr];

		/* Associate the fwnode with the device structure so it
		 * can be looked up later.
		 */
		phy->mdio.dev.fwnode = child;

		/* All data is now stored in the phy struct, so register it */
		rc = phy_device_register(phy);
		if (rc) {
			phy_device_free(phy);
			fwnode_handle_put(phy->mdio.dev.fwnode);
			return rc;
		}
	} else if (is_of_node(child)) {
		rc = of_mdiobus_phy_device_register(bus, phy, to_of_node(child), addr);
		if (rc) {
			unregister_mii_timestamper(mii_ts);
			phy_device_free(phy);
			return rc;
		}
	}

	/* phy->mii_ts may already be defined by the PHY driver. A
	 * mii_timestamper probed via the device tree will still have
	 * precedence.
	 */
	if (mii_ts)
		phy->mii_ts = mii_ts;
	return 0;
}
EXPORT_SYMBOL(fwnode_mdiobus_register_phy);

/**
 * fwnode_mdiobus_register - Register mii_bus and create PHYs from fwnode
 * @mdio: pointer to mii_bus structure
 * @fwnode: pointer to fwnode of MDIO bus.
 *
 * This function returns of_mdiobus_register() for DT and
 * acpi_mdiobus_register() for ACPI.
 */
int fwnode_mdiobus_register(struct mii_bus *mdio, struct fwnode_handle *fwnode)
{
	if (is_of_node(fwnode))
		return of_mdiobus_register(mdio, to_of_node(fwnode));

	if (is_acpi_node(fwnode))
		return acpi_mdiobus_register(mdio, fwnode);

	return -EINVAL;
}
EXPORT_SYMBOL(fwnode_mdiobus_register);
