// SPDX-License-Identifier: GPL-2.0
/*
 * Synopsys DesignWare XPCS regmap device driver
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/pcs/pcs-xpcs-regmap.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/sizes.h>

#include "pcs-xpcs.h"

/* Page select register for the indirect MMIO CSRs access */
#define DW_VR_CSR_VIEWPORT		0xff

struct dw_xpcs_regmap {
	struct device *dev;
	struct mii_bus *bus;
	struct regmap *regmap;
	bool reg_indir;
	int reg_width;
};

static ptrdiff_t xpcs_regmap_addr_format(int dev, int reg)
{
	return FIELD_PREP(0x1f0000, dev) | FIELD_PREP(0xffff, reg);
}

static u16 xpcs_regmap_addr_page(ptrdiff_t csr)
{
	return FIELD_GET(0x1fff00, csr);
}

static ptrdiff_t xpcs_regmap_addr_offset(ptrdiff_t csr)
{
	return FIELD_GET(0xff, csr);
}

static int xpcs_regmap_read_reg_indirect(struct dw_xpcs_regmap *pxpcs, int dev,
					 int reg)
{
	unsigned int shift, val;
	ptrdiff_t csr, ofs;
	u16 page;
	int res;

	shift = pxpcs->reg_width == 4 ? 2 : 1;
	csr = xpcs_regmap_addr_format(dev, reg);
	page = xpcs_regmap_addr_page(csr);
	ofs = xpcs_regmap_addr_offset(csr);

	res = regmap_write(pxpcs->regmap, DW_VR_CSR_VIEWPORT << shift, page);
	if (res < 0)
		return res;

	res = regmap_read(pxpcs->regmap, ofs << shift, &val);
	if (res < 0)
		return res;

	return val & 0xffff;
}

static int xpcs_regmap_write_reg_indirect(struct dw_xpcs_regmap *pxpcs, int dev,
					  int reg, u16 val)
{
	unsigned int shift;
	ptrdiff_t csr, ofs;
	u16 page;
	int res;

	shift = pxpcs->reg_width == 4 ? 2 : 1;
	csr = xpcs_regmap_addr_format(dev, reg);
	page = xpcs_regmap_addr_page(csr);
	ofs = xpcs_regmap_addr_offset(csr);

	res = regmap_write(pxpcs->regmap, DW_VR_CSR_VIEWPORT << shift, page);
	if (res < 0)
		return res;

	return regmap_write(pxpcs->regmap, ofs << shift, val);
}

static int xpcs_regmap_read_reg_direct(struct dw_xpcs_regmap *pxpcs, int dev,
				       int reg)
{
	unsigned int val;
	ptrdiff_t csr;
	int res;

	csr = xpcs_regmap_addr_format(dev, reg);

	res = regmap_read(pxpcs->regmap, csr << (pxpcs->reg_width == 4 ? 2 : 1),
			  &val);
	if (res < 0)
		return res;

	return val & 0xffff;
}

static int xpcs_regmap_write_reg_direct(struct dw_xpcs_regmap *pxpcs, int dev,
					int reg, u16 val)
{
	ptrdiff_t csr = xpcs_regmap_addr_format(dev, reg);
	return regmap_write(pxpcs->regmap,
			    csr << (pxpcs->reg_width == 4 ? 2 : 1), val);
}

static int xpcs_regmap_read_c22(struct mii_bus *bus, int addr, int reg)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	if (pxpcs->reg_indir)
		return xpcs_regmap_read_reg_indirect(pxpcs, MDIO_MMD_VEND2, reg);
	else
		return xpcs_regmap_read_reg_direct(pxpcs, MDIO_MMD_VEND2, reg);
}

static int xpcs_regmap_write_c22(struct mii_bus *bus, int addr, int reg, u16 val)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	if (pxpcs->reg_indir)
		return xpcs_regmap_write_reg_indirect(pxpcs, MDIO_MMD_VEND2, reg, val);
	else
		return xpcs_regmap_write_reg_direct(pxpcs, MDIO_MMD_VEND2, reg, val);
}

static int xpcs_regmap_read_c45(struct mii_bus *bus, int addr, int dev, int reg)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	if (pxpcs->reg_indir)
		return xpcs_regmap_read_reg_indirect(pxpcs, dev, reg);
	else
		return xpcs_regmap_read_reg_direct(pxpcs, dev, reg);
}

static int xpcs_regmap_write_c45(struct mii_bus *bus, int addr, int dev,
			       int reg, u16 val)
{
	struct dw_xpcs_regmap *pxpcs = bus->priv;

	if (addr != 0)
		return -ENODEV;

	if (pxpcs->reg_indir)
		return xpcs_regmap_write_reg_indirect(pxpcs, dev, reg, val);
	else
		return xpcs_regmap_write_reg_direct(pxpcs, dev, reg, val);
}

static int xpcs_regmap_init_bus(struct dw_xpcs_regmap *pxpcs)
{
	struct device *dev = pxpcs->dev;
	static atomic_t id = ATOMIC_INIT(-1);
	int ret;

	pxpcs->bus = devm_mdiobus_alloc_size(dev, 0);
	if (!pxpcs->bus)
		return -ENOMEM;

	pxpcs->bus->name = "DW XPCS MCI/APB3";
	pxpcs->bus->read = xpcs_regmap_read_c22;
	pxpcs->bus->write = xpcs_regmap_write_c22;
	pxpcs->bus->read_c45 = xpcs_regmap_read_c45;
	pxpcs->bus->write_c45 = xpcs_regmap_write_c45;
	pxpcs->bus->phy_mask = ~0;
	pxpcs->bus->parent = dev;
	pxpcs->bus->priv = pxpcs;

	snprintf(pxpcs->bus->id, MII_BUS_ID_SIZE,
		 "dwxpcs-%x", atomic_inc_return(&id));

	/* MDIO-bus here serves as just a back-end engine abstracting out
	 * the MDIO and MCI/APB3 IO interfaces utilized for the DW XPCS CSRs
	 * access.
	 */
	ret = devm_mdiobus_register(dev, pxpcs->bus);
	if (ret) {
		dev_err(dev, "Failed to create MDIO bus\n");
		return ret;
	}

	return 0;
}

static int xpcs_regmap_init_dev(struct dw_xpcs_regmap *pxpcs)
{
	struct device *dev = pxpcs->dev;
	struct mdio_device *mdiodev;
	int ret;

	/* There is a single memory-mapped DW XPCS device */
	mdiodev = mdio_device_create(pxpcs->bus, 0);
	if (IS_ERR(mdiodev))
		return PTR_ERR(mdiodev);

	/* Associate the FW-node with the device structure so it can be looked
	 * up later. Make sure DD-core is aware of the OF-node being re-used.
	 */
	device_set_node(&mdiodev->dev, fwnode_handle_get(dev_fwnode(dev)));
	mdiodev->dev.of_node_reused = true;

	/* Pass the data further so the DW XPCS driver core could use it */
	mdiodev->dev.platform_data = (void *)device_get_match_data(dev);

	ret = mdio_device_register(mdiodev);
	if (ret) {
		dev_err(dev, "Failed to register MDIO device\n");
		goto err_clean_data;
	}

	return 0;

err_clean_data:
	mdiodev->dev.platform_data = NULL;

	mdio_device_free(mdiodev);

	return ret;
}

struct dw_xpcs *devm_xpcs_regmap_register(struct device *dev,
 					  const struct xpcs_regmap_config *config)
{
	struct dw_xpcs_regmap *pxpcs;
	int ret;

	pxpcs = devm_kzalloc(dev, sizeof(*pxpcs), GFP_KERNEL);
	if (!pxpcs)
		return ERR_PTR(-ENOMEM);

	pxpcs->dev = dev;
	pxpcs->regmap = config->regmap;
	pxpcs->reg_indir = config->reg_indir;
	pxpcs->reg_width = config->reg_width;

	ret = xpcs_regmap_init_bus(pxpcs);
	if (ret)
		return ERR_PTR(ret);

#if 0
	ret = xpcs_regmap_init_dev(pxpcs);
	if (ret)
		return ERR_PTR(ret);

	return pxpcs->bus;
#else
	return xpcs_create_mdiodev(pxpcs->bus, 0);
#endif
}
