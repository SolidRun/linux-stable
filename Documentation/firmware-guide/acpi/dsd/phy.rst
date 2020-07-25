.. SPDX-License-Identifier: GPL-2.0

=========================
MDIO bus and PHYs in ACPI
=========================

The PHYs on an mdiobus are probed and registered using mdiobus_register().
Later, for connecting these PHYs to MAC, the PHYs registered on the
mdiobus have to be referenced.

mdio-handle
-----------
For each MAC node, a property "mdio-handle" is used to reference the
MDIO bus on which the PHYs are registered. On getting hold of the MDIO
bus, use find_phy_device() to get the PHY connected to the MAC.

phy-channel
-----------
Property "phy-channel" defines the address of the PHY on the mdiobus.

phy-mode
--------
Property "phy-mode" defines the type of PHY interface.

An example of this is shown below::

DSDT entry for MAC where MDIO node is referenced
------------------------------------------------
	Scope(\_SB.MCE0.PR17) // 1G
	{
	  Name (_DSD, Package () {
	     ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
		 Package () {
		     Package () {"phy-channel", 1},
		     Package () {"phy-mode", "rgmii-id"},
		     Package () {"mdio-handle", Package (){\_SB.MDI0}}
	      }
	   })
	}

	Scope(\_SB.MCE0.PR18) // 1G
	{
	  Name (_DSD, Package () {
	    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
		Package () {
		    Package () {"phy-channel", 2},
		    Package () {"phy-mode", "rgmii-id"},
		    Package () {"mdio-handle", Package (){\_SB.MDI0}}
	    }
	  })
	}

DSDT entry for MDIO node
------------------------
a) Silicon Component
--------------------
	Scope(_SB)
	{
	  Device(MDI0) {
	    Name(_HID, "NXP0006")
	    Name(_CCA, 1)
	    Name(_UID, 0)
	    Name(_CRS, ResourceTemplate() {
	      Memory32Fixed(ReadWrite, MDI0_BASE, MDI_LEN)
	      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared)
	       {
		 MDI0_IT
	       }
	    }) // end of _CRS for MDI0
	    Name (_DSD, Package () {
	      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
	      Package () {
		 Package () {"little-endian", 1},
	      }
	    })
	  } // end of MDI0
	}

b) Platform Component
---------------------
	Scope(\_SB.MDI0)
	{
	  Device(PHY1) {
	    Name (_ADR, 0x1)
	  } // end of PHY1

	  Device(PHY2) {
	    Name (_ADR, 0x2)
	  } // end of PHY2
	}
