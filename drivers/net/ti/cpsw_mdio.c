// SPDX-License-Identifier: GPL-2.0+
/*
 * CPSW MDIO generic driver for TI AMxx/K2x/EMAC devices.
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <common.h>
#include <log.h>
#include <malloc.h>
#include <asm/io.h>
#include <miiphy.h>
#include <wait_bit.h>
#include <linux/bitops.h>
#include <linux/delay.h>

struct cpsw_mdio_regs {
	u32	version;
	u32	control;
#define CONTROL_IDLE		BIT(31)
#define CONTROL_ENABLE		BIT(30)
#define CONTROL_FAULT		BIT(19)
#define CONTROL_FAULT_ENABLE	BIT(18)
#define CONTROL_DIV_MASK	GENMASK(15, 0)

#define MDIO_MAN_MDCLK_O        BIT(2)
#define MDIO_MAN_OE             BIT(1)
#define MDIO_MAN_PIN            BIT(0)
#define MDIO_MANUALMODE         BIT(31)

	u32	alive;
	u32	link;
	u32	linkintraw;
	u32	linkintmasked;
	u32	__reserved_0[2];
	u32	userintraw;
	u32	userintmasked;
	u32	userintmaskset;
	u32	userintmaskclr;
	u32	manualif;
	u32	poll;
	u32	__reserved_1[18];

	struct {
		u32		access;
		u32		physel;
#define USERACCESS_GO		BIT(31)
#define USERACCESS_WRITE	BIT(30)
#define USERACCESS_ACK		BIT(29)
#define USERACCESS_READ		(0)
#define USERACCESS_PHY_REG_SHIFT	(21)
#define USERACCESS_PHY_ADDR_SHIFT	(16)
#define USERACCESS_DATA		GENMASK(15, 0)
	} user[2];
};

#define CPSW_MDIO_DIV_DEF	0xff
#define PHY_REG_MASK		0x1f
#define PHY_ID_MASK		0x1f

#define MDIO_BITRANGE           0x8000
#define C22_READ_PATTERN        0x6
#define C22_WRITE_PATTERN       0x5
#define C22_BITRANGE            0x8
#define PHY_BITRANGE            0x10
#define PHY_DATA_BITRANGE       0x8000

/*
 * This timeout definition is a worst-case ultra defensive measure against
 * unexpected controller lock ups.  Ideally, we should never ever hit this
 * scenario in practice.
 */
#define CPSW_MDIO_TIMEOUT            100 /* msecs */

enum cpsw_mdio_manual {
	MDIO_PIN = 0,
	MDIO_OE,
	MDIO_MDCLK,
};

struct cpsw_mdio {
	struct cpsw_mdio_regs *regs;
	struct mii_dev *bus;
	int div;
};

static void cpsw_mdio_disable(struct cpsw_mdio *mdio)
{
	u32 reg;
	/* Disable MDIO state machine */
	reg = readl(&mdio->regs->control);
	reg &= ~CONTROL_ENABLE;

	writel(reg, &mdio->regs->control);
}

static void cpsw_mdio_enable_manual_mode(struct cpsw_mdio *mdio)
{
	u32 reg;

	/* set manual mode */
	reg = readl(&mdio->regs->poll);
	reg |= MDIO_MANUALMODE;

	writel(reg, &mdio->regs->poll);
}

static void set_mdc(struct cpsw_mdio *mdio, int level)
{
	u32 reg;

	reg = readl(&mdio->regs->manualif);
	if (level)
		reg |= MDIO_MAN_MDCLK_O;
	else
		reg &= ~MDIO_MAN_MDCLK_O;

	writel(reg, &mdio->regs->manualif);
}

static void set_mdio_dir(struct cpsw_mdio *mdio, int output)
{
	u32 reg;

	reg = readl(&mdio->regs->manualif);
	if (output)
		reg |= MDIO_MAN_OE;
	else
		reg &= ~MDIO_MAN_OE;

	writel(reg, &mdio->regs->manualif);
}

static void set_mdio_data(struct cpsw_mdio *mdio, int value)
{
	u32 reg;

	reg = readl(&mdio->regs->manualif);
	if (value)
		reg |= MDIO_MAN_PIN;
	else
		reg &= ~MDIO_MAN_PIN;

	writel(reg, &mdio->regs->manualif);
}

static int get_mdio_data(struct cpsw_mdio *mdio)
{
	u32 reg;

	reg = readl(&mdio->regs->manualif);

	return test_bit(MDIO_PIN, &reg);
}

#define MDIO_READ 2
#define MDIO_WRITE 1

#define MDIO_C45 (1<<15)

/* Minimum MDC period is 400 ns, plus some margin for error.  MDIO_DELAY
 * is done twice per period.
 */
#define MDIO_DELAY 250

/* The PHY may take up to 300 ns to produce data, plus some margin
 * for error.
 */
#define MDIO_READ_DELAY 350

/* MDIO must already be configured as output. */
static void mdiobb_send_bit(struct cpsw_mdio *ctrl, int val)
{
        set_mdio_data(ctrl, val);
        ndelay(MDIO_DELAY);
        set_mdc(ctrl, 1);
        ndelay(MDIO_DELAY);
        set_mdc(ctrl, 0);
}

/* MDIO must already be configured as input. */
static int mdiobb_get_bit(struct cpsw_mdio *ctrl)
{
        ndelay(MDIO_DELAY);
        set_mdc(ctrl, 1);
        ndelay(MDIO_READ_DELAY);
        set_mdc(ctrl, 0);

        return get_mdio_data(ctrl);
}

/* MDIO must already be configured as output. */
static void mdiobb_send_num(struct cpsw_mdio *ctrl, u16 val, int bits)
{
        int i;

        for (i = bits - 1; i >= 0; i--)
                mdiobb_send_bit(ctrl, (val >> i) & 1);
}

/* MDIO must already be configured as input. */
static u16 mdiobb_get_num(struct cpsw_mdio *ctrl, int bits)
{
        int i;
        u16 ret = 0;

        for (i = bits - 1; i >= 0; i--) {
                ret <<= 1;
                ret |= mdiobb_get_bit(ctrl);
        }

        return ret;
}

static void mdiobb_cmd(struct cpsw_mdio *ctrl, int op, u8 phy, u8 reg)
{
	int i;

	set_mdio_dir(ctrl, 1);

	/*
	 * Send a 32 bit preamble ('1's) with an extra '1' bit for good
	 * measure.  The IEEE spec says this is a PHY optional
	 * requirement.  The AMD 79C874 requires one after power up and
	 * one after a MII communications error.  This means that we are
	 * doing more preambles than we need, but it is safer and will be
	 * much more robust.
	 */

	for (i = 0; i < 32; i++)
		mdiobb_send_bit(ctrl, 1);

	/* send the start bit (01) and the read opcode (10) or write (01).
	   Clause 45 operation uses 00 for the start and 11, 10 for
	   read/write */
	mdiobb_send_bit(ctrl, 0);
/*
	if (op & MDIO_C45)
		mdiobb_send_bit(ctrl, 0);
	else
*/
		mdiobb_send_bit(ctrl, 1);
	mdiobb_send_bit(ctrl, (op >> 1) & 1);
	mdiobb_send_bit(ctrl, (op >> 0) & 1);

	mdiobb_send_num(ctrl, phy, 5);
	mdiobb_send_num(ctrl, reg, 5);
}

static int mdiobb_read_common(struct cpsw_mdio *ctrl, int phy)
{
        int ret, i;

        set_mdio_dir(ctrl, 0);

        /* check the turnaround bit: the PHY should be driving it to zero, if this
         * PHY is listed in phy_ignore_ta_mask as having broken TA, skip that
         */
        if (mdiobb_get_bit(ctrl) != 0) {
                /* PHY didn't drive TA low -- flush any bits it
                 * may be trying to send.
                 */
                for (i = 0; i < 32; i++)
                        mdiobb_get_bit(ctrl);

                return 0xffff;
        }

        ret = mdiobb_get_num(ctrl, 16);
        mdiobb_get_bit(ctrl);

        return ret;
}

static int mdiobb_write_common(struct cpsw_mdio *ctrl, u16 val)
{
        /* send the turnaround (10) */
        mdiobb_send_bit(ctrl, 1);
        mdiobb_send_bit(ctrl, 0);

        mdiobb_send_num(ctrl, val, 16);

        set_mdio_dir(ctrl, 0);
        mdiobb_get_bit(ctrl);

        return 0;
}

static int cpsw_mdio_sw_read(struct mii_dev *bus, int phy_id,
			     int dev_addr, int phy_reg)
{
	struct cpsw_mdio *mdio = bus->priv;
	int reg;

//	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
//		return -EINVAL;

        mdiobb_cmd(mdio, MDIO_READ, phy_id, phy_reg);

        reg = mdiobb_read_common(mdio, phy_id);
//	printf("read %d %d %d = 0x%x\n", phy_id, dev_addr, phy_reg, reg);
	return reg;
}

static int cpsw_mdio_sw_write(struct mii_dev *bus, int phy_id,
			      int dev_addr, int phy_reg, u16 phy_data)
{
	struct cpsw_mdio *mdio = bus->priv;

//	if ((phy_reg & ~PHY_REG_MASK) || (phy_id & ~PHY_ID_MASK))
//		return -EINVAL;

        mdiobb_cmd(mdio, MDIO_WRITE, phy_id, phy_reg);

//	printf("write %d %d %d = 0x%x\n", phy_id, dev_addr, phy_reg, phy_data);
        return mdiobb_write_common(mdio, phy_data);
}

/* wait until hardware is ready for another user access */
static int cpsw_mdio_wait_for_user_access(struct cpsw_mdio *mdio)
{
	return wait_for_bit_le32(&mdio->regs->user[0].access,
				 USERACCESS_GO, false,
				 CPSW_MDIO_TIMEOUT, false);
}

static int cpsw_mdio_read(struct mii_dev *bus, int phy_id,
			  int dev_addr, int phy_reg)
{
	struct cpsw_mdio *mdio = bus->priv;
	int data, ret;
	u32 reg;

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	ret = cpsw_mdio_wait_for_user_access(mdio);
	if (ret)
		return ret;
	reg = (USERACCESS_GO | USERACCESS_READ |
	       (phy_reg << USERACCESS_PHY_REG_SHIFT) |
	       (phy_id << USERACCESS_PHY_ADDR_SHIFT));
	writel(reg, &mdio->regs->user[0].access);
	ret = cpsw_mdio_wait_for_user_access(mdio);
	if (ret)
		return ret;

	reg = readl(&mdio->regs->user[0].access);
	data = (reg & USERACCESS_ACK) ? (reg & USERACCESS_DATA) : -1;
	return data;
}

static int cpsw_mdio_write(struct mii_dev *bus, int phy_id, int dev_addr,
			   int phy_reg, u16 data)
{
	struct cpsw_mdio *mdio = bus->priv;
	u32 reg;
	int ret;

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	ret = cpsw_mdio_wait_for_user_access(mdio);
	if (ret)
		return ret;
	reg = (USERACCESS_GO | USERACCESS_WRITE |
	       (phy_reg << USERACCESS_PHY_REG_SHIFT) |
	       (phy_id << USERACCESS_PHY_ADDR_SHIFT) |
	       (data & USERACCESS_DATA));
	writel(reg, &mdio->regs->user[0].access);

	return cpsw_mdio_wait_for_user_access(mdio);
}

u32 cpsw_mdio_get_alive(struct mii_dev *bus)
{
	struct cpsw_mdio *mdio = bus->priv;
	u32 val;

	val = readl(&mdio->regs->alive);
	return val & GENMASK(7, 0);
}

struct mii_dev *cpsw_mdio_init(const char *name, phys_addr_t mdio_base,
			       u32 bus_freq, int fck_freq, bool manual_mode)
{
	struct cpsw_mdio *cpsw_mdio;
	int ret;

	cpsw_mdio = calloc(1, sizeof(*cpsw_mdio));
	if (!cpsw_mdio) {
		debug("failed to alloc cpsw_mdio\n");
		return NULL;
	}

	cpsw_mdio->bus = mdio_alloc();
	if (!cpsw_mdio->bus) {
		debug("failed to alloc mii bus\n");
		free(cpsw_mdio);
		return NULL;
	}

	cpsw_mdio->regs = (struct cpsw_mdio_regs *)(uintptr_t)mdio_base;

	if (!bus_freq || !fck_freq)
		cpsw_mdio->div = CPSW_MDIO_DIV_DEF;
	else
		cpsw_mdio->div = (fck_freq / bus_freq) - 1;
	cpsw_mdio->div &= CONTROL_DIV_MASK;

	/* set enable and clock divider */
/*
	writel(cpsw_mdio->div | CONTROL_ENABLE | CONTROL_FAULT |
	       CONTROL_FAULT_ENABLE, &cpsw_mdio->regs->control);
	wait_for_bit_le32(&cpsw_mdio->regs->control,
			  CONTROL_IDLE, false, CPSW_MDIO_TIMEOUT, true);
*/

	/*
	 * wait for scan logic to settle:
	 * the scan time consists of (a) a large fixed component, and (b) a
	 * small component that varies with the mii bus frequency.  These
	 * were estimated using measurements at 1.1 and 2.2 MHz on tnetv107x
	 * silicon.  Since the effect of (b) was found to be largely
	 * negligible, we keep things simple here.
	 */
	mdelay(1);

	if (manual_mode) {
		cpsw_mdio->bus->read = cpsw_mdio_sw_read;
		cpsw_mdio->bus->write = cpsw_mdio_sw_write;
		/* idle state */
		set_mdc(cpsw_mdio, 0);
		set_mdio_data(cpsw_mdio, 1);
		set_mdio_dir(cpsw_mdio, 0);
		cpsw_mdio_disable(cpsw_mdio);
		cpsw_mdio_enable_manual_mode(cpsw_mdio);
	} else {
		cpsw_mdio->bus->read = cpsw_mdio_read;
		cpsw_mdio->bus->write = cpsw_mdio_write;
	}

	cpsw_mdio->bus->priv = cpsw_mdio;
	snprintf(cpsw_mdio->bus->name, sizeof(cpsw_mdio->bus->name), name);

	ret = mdio_register(cpsw_mdio->bus);
	if (ret < 0) {
		debug("failed to register mii bus\n");
		goto free_bus;
	}

	return cpsw_mdio->bus;

free_bus:
	mdio_free(cpsw_mdio->bus);
	free(cpsw_mdio);
	return NULL;
}

void cpsw_mdio_free(struct mii_dev *bus)
{
	struct cpsw_mdio *mdio = bus->priv;
	u32 reg;

	/* disable mdio */
	reg = readl(&mdio->regs->control);
	reg &= ~CONTROL_ENABLE;
	writel(reg, &mdio->regs->control);

	mdio_unregister(bus);
	mdio_free(bus);
	free(mdio);
}
