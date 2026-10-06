// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner A13 (sun5i) suspend to RAM support
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * The A13 has no deep sleep state of its own - suspend to RAM works by
 * putting the DRAM into self-refresh (its rail stays up), latching the
 * DRAM pads (SDR_DPCR bit 0, retained in the DRAM pad power domain) and
 * then cutting the SoC power rails via the AXP209 PMIC. Waking up is a
 * PMIC power-on event: the SoC boots through BROM into the SPL, which
 * recognizes the pending resume (RTC general purpose registers + the pad
 * hold flag), recovers the DRAM from self-refresh without destroying its
 * contents and jumps to cpu_resume. This mirrors the "super standby"
 * mechanism of the vendor BSP, with U-Boot SPL taking the role of boot0.
 *
 * Everything on the SoC loses state, so the critical always-on blocks
 * (CCU, INTC, PIO, timers, SRAM controller) are saved and restored raw
 * here. Peripherals are expected to reinitialize their hardware in their
 * driver resume callbacks.
 *
 * The handoff protocol with the SPL uses the battery backed RTC general
 * purpose registers:
 *   GP0: DRAM ZQ calibration result (SDR_ZQSR & 0xfffff)
 *   GP1: physical address of cpu_resume
 *   GP2: 0x53555350 ("SUSP") when a resume image is armed
 *   GP3: SDR_DCR value (DRAM geometry)
 */

#include <linux/cpu_pm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/suspend.h>

#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/suspend.h>

#define SUN5I_SRAM_A1_BASE		0x00000000
#define SUN5I_DRAMC_BASE		0x01c01000
#define SUN5I_CCU_BASE			0x01c20000
#define SUN5I_INTC_BASE			0x01c20400
#define SUN5I_PIO_BASE			0x01c20800
#define SUN5I_TIMER_BASE		0x01c20c00
#define SUN5I_SRAMC_BASE		0x01c00000
#define SUN5I_TWI0_BASE			0x01c2ac00
#define SUN5I_HSTIMER_BASE		0x01c60000
#define SUN5I_DRAM_BASE			0x40000000

/* battery backed general purpose registers in the RTC block */
#define SUN5I_RTC_GP_REG(n)		(0x120 + 4 * (n))
#define SUN5I_STANDBY_GP_ZQ		SUN5I_RTC_GP_REG(0)
#define SUN5I_STANDBY_GP_RESUME		SUN5I_RTC_GP_REG(1)
#define SUN5I_STANDBY_GP_MAGIC		SUN5I_RTC_GP_REG(2)
#define SUN5I_STANDBY_GP_DCR		SUN5I_RTC_GP_REG(3)
#define SUN5I_STANDBY_MAGIC		0x53555350

#define SUN5I_DRAMC_SDR_DCR		0x004
#define SUN5I_DRAMC_SDR_ZQSR		0x0b0

#define SUN5I_TIMER_WDOG_MODE		0x094

/* register file sizes for the raw state save/restore */
#define SUN5I_CCU_WORDS			(0x164 / 4)
#define SUN5I_INTC_WORDS		(0x98 / 4)
#define SUN5I_PIO_WORDS			(0x21c / 4)
#define SUN5I_TIMER_WORDS		(0x28 / 4 + 1)
#define SUN5I_SRAMC_WORDS		(0x98 / 4)
#define SUN5I_HSTIMER_WORDS		(0x24 / 4)

/* the SPL DQS gate training pattern clobbers the start of DRAM */
#define SUN5I_TRAINING_WORDS		16

/* mv64xxx style TWI controller, polled */
#define SUN5I_TWI_DATA			0x08
#define SUN5I_TWI_CNTR			0x0c
#define SUN5I_TWI_STAT			0x10
#define SUN5I_TWI_CCR			0x14
#define SUN5I_TWI_SRST			0x18
#define SUN5I_TWI_EFR			0x1c
#define SUN5I_TWI_CNTR_INT_FLAG		BIT(3)
#define SUN5I_TWI_CNTR_M_STP		BIT(4)
#define SUN5I_TWI_CNTR_M_STA		BIT(5)
#define SUN5I_TWI_CNTR_BUS_EN		BIT(6)

#define AXP209_I2C_ADDR			0x34
#define AXP209_POWER_OUT_CTRL		0x12
#define AXP209_VOFF_CTRL		0x31
#define AXP209_VOFF_SLEEP_ARM		BIT(3)
/* rails cut for sleep: EXTEN, DCDC3, LDO4, DCDC2, LDO3 (LDO2/avcc stays) */
#define AXP209_SLEEP_OFF_MASK		0x5b

struct sun5i_pm {
	void __iomem *sram;
	void __iomem *dramc;
	void __iomem *ccu;
	void __iomem *intc;
	void __iomem *pio;
	void __iomem *timer;
	void __iomem *sramc;
	void __iomem *twi;
	void __iomem *hstimer;
	u32 *dram_start;

	u32 ccu_save[SUN5I_CCU_WORDS];
	u32 intc_save[SUN5I_INTC_WORDS];
	u32 pio_save[SUN5I_PIO_WORDS];
	u32 timer_save[SUN5I_TIMER_WORDS];
	u32 sramc_save[SUN5I_SRAMC_WORDS];
	u32 hstimer_save[SUN5I_HSTIMER_WORDS];
	u32 training_save[SUN5I_TRAINING_WORDS];

	u32 axp_sleep_val;
	int (*standby)(void __iomem *dramc, void __iomem *twi, u32 axp_val);
};

static struct sun5i_pm pm;

extern int sun5i_standby_blob(void __iomem *dramc, void __iomem *twi,
			      u32 axp_val);
extern u32 sun5i_standby_blob_sz;

/*
 * Polled access to the AXP209 over TWI0, for use when the i2c subsystem
 * is already suspended. The controller is reset and reconfigured behind
 * the i2c driver's back; on the resume/abort path the driver reinits the
 * hardware itself (pm_runtime_force_resume), so nothing is saved here.
 */
static int sun5i_twi_wait_flag(u8 *stat)
{
	int timeout = 100000;

	while (!(readl(pm.twi + SUN5I_TWI_CNTR) & SUN5I_TWI_CNTR_INT_FLAG)) {
		if (!--timeout)
			return -ETIMEDOUT;
		udelay(1);
	}

	*stat = readl(pm.twi + SUN5I_TWI_STAT);
	return 0;
}

static int sun5i_twi_step(u32 data, bool start, u8 expect)
{
	u32 val;
	u8 stat;
	int ret;

	if (start) {
		val = readl(pm.twi + SUN5I_TWI_CNTR);
		writel(val | SUN5I_TWI_CNTR_M_STA, pm.twi + SUN5I_TWI_CNTR);
	} else {
		writel(data, pm.twi + SUN5I_TWI_DATA);
		val = readl(pm.twi + SUN5I_TWI_CNTR);
		writel(val & ~SUN5I_TWI_CNTR_INT_FLAG,
		       pm.twi + SUN5I_TWI_CNTR);
	}

	ret = sun5i_twi_wait_flag(&stat);
	if (ret)
		return ret;

	return stat == expect ? 0 : -EIO;
}

static void sun5i_twi_stop(void)
{
	u32 val;

	val = readl(pm.twi + SUN5I_TWI_CNTR) & 0xc0;
	writel(val | SUN5I_TWI_CNTR_M_STP, pm.twi + SUN5I_TWI_CNTR);
	udelay(100);
}

static int sun5i_twi_xfer(u8 reg, u8 *val, bool write)
{
	int ret;

	ret = sun5i_twi_step(0, true, 0x08);
	if (ret)
		goto out;
	ret = sun5i_twi_step(AXP209_I2C_ADDR << 1, false, 0x18);
	if (ret)
		goto out;
	ret = sun5i_twi_step(reg, false, 0x28);
	if (ret)
		goto out;

	if (write) {
		ret = sun5i_twi_step(*val, false, 0x28);
		goto out;
	}

	/* repeated start, SLA+R, single byte with NAK */
	ret = sun5i_twi_step(0, true, 0x10);
	if (ret)
		goto out;
	ret = sun5i_twi_step((AXP209_I2C_ADDR << 1) | 1, false, 0x40);
	if (ret)
		goto out;
	ret = sun5i_twi_step(0, false, 0x58);
	if (ret == -EIO)
		ret = 0;
	if (!ret)
		*val = readl(pm.twi + SUN5I_TWI_DATA);
out:
	sun5i_twi_stop();
	return ret;
}

static int sun5i_axp_prepare(void)
{
	u8 val;
	int ret;

	/* APB1 to OSC24M and TWI0 clocked, to get a known 400kHz setup */
	writel(0, pm.ccu + 0x58);
	writel(readl(pm.ccu + 0x6c) | BIT(0), pm.ccu + 0x6c);

	writel(1, pm.twi + SUN5I_TWI_SRST);
	udelay(100);
	writel((5 << 3) | 0, pm.twi + SUN5I_TWI_CCR);
	writel(0, pm.twi + SUN5I_TWI_EFR);
	writel(SUN5I_TWI_CNTR_BUS_EN, pm.twi + SUN5I_TWI_CNTR);

	/* arm the PMIC wakeup: outputs come back on the next power event */
	ret = sun5i_twi_xfer(AXP209_VOFF_CTRL, &val, false);
	if (ret)
		return ret;
	val |= AXP209_VOFF_SLEEP_ARM;
	ret = sun5i_twi_xfer(AXP209_VOFF_CTRL, &val, true);
	if (ret)
		return ret;

	ret = sun5i_twi_xfer(AXP209_POWER_OUT_CTRL, &val, false);
	if (ret)
		return ret;
	pm.axp_sleep_val = val & ~AXP209_SLEEP_OFF_MASK;

	return 0;
}

static void save_regs(void __iomem *base, u32 *buf, int words)
{
	int i;

	for (i = 0; i < words; i++)
		buf[i] = readl(base + 4 * i);
}

static void restore_regs(void __iomem *base, const u32 *buf, int words)
{
	int i;

	for (i = 0; i < words; i++)
		writel(buf[i], base + 4 * i);
}

/*
 * The SPL leaves the clock tree in its early boot state (CPU on PLL1 at
 * the U-Boot frequency, APB1 on OSC24M, most gates closed). Move the CPU
 * to OSC24M, bring the PLLs back to their pre-suspend configuration and
 * then restore dividers and gates. PLL5 feeds the running DRAM and is
 * owned by the SPL - it is deliberately left alone.
 */
static void sun5i_ccu_restore(void)
{
	int i;

	writel((readl(pm.ccu + 0x54) & ~(0x3 << 16)) | (0x1 << 16),
	       pm.ccu + 0x54);

	for (i = 0; i <= 0x50 / 4; i++) {
		if (i == 0x20 / 4 || i == 0x24 / 4)
			continue;
		writel(pm.ccu_save[i], pm.ccu + 4 * i);
	}

	/* PLL stabilization, sun5i has no lock flags */
	udelay(500);

	writel(pm.ccu_save[0x54 / 4], pm.ccu + 0x54);
	udelay(20);

	for (i = 0x58 / 4; i < SUN5I_CCU_WORDS; i++)
		writel(pm.ccu_save[i], pm.ccu + 4 * i);
}

static void sun5i_pio_restore(void)
{
	int port, i;
	u32 off;

	/* levels, drive and bias first, then the pin functions */
	for (port = 0; port < 7; port++) {
		off = 0x24 * port;

		writel(pm.pio_save[(off + 0x10) / 4], pm.pio + off + 0x10);
		for (i = 0x14; i <= 0x20; i += 4)
			writel(pm.pio_save[(off + i) / 4], pm.pio + off + i);
		for (i = 0x00; i <= 0x0c; i += 4)
			writel(pm.pio_save[(off + i) / 4], pm.pio + off + i);
	}

	/* external interrupt config, clear pending, debounce */
	for (i = 0x200; i <= 0x210; i += 4)
		writel(pm.pio_save[i / 4], pm.pio + i);
	writel(~0, pm.pio + 0x214);
	writel(pm.pio_save[0x218 / 4], pm.pio + 0x218);
}

static void sun5i_timer_restore(void)
{
	u32 ctl;

	writel(0x3, pm.timer + 0x04);

	writel(pm.timer_save[0x14 / 4], pm.timer + 0x14);
	ctl = pm.timer_save[0x10 / 4];
	writel(ctl | ((ctl & BIT(0)) ? BIT(1) : 0), pm.timer + 0x10);

	writel(pm.timer_save[0x24 / 4], pm.timer + 0x24);
	ctl = pm.timer_save[0x20 / 4];
	writel(ctl | ((ctl & BIT(0)) ? BIT(1) : 0), pm.timer + 0x20);

	writel(pm.timer_save[0], pm.timer + 0x00);
}

static void sun5i_hstimer_restore(void)
{
	u32 ctl;

	writel(0x1, pm.hstimer + 0x04);

	writel(pm.hstimer_save[0x14 / 4], pm.hstimer + 0x14);
	writel(pm.hstimer_save[0x18 / 4], pm.hstimer + 0x18);
	ctl = pm.hstimer_save[0x10 / 4];
	writel(ctl | ((ctl & BIT(0)) ? BIT(1) : 0), pm.hstimer + 0x10);

	writel(pm.hstimer_save[0], pm.hstimer + 0x00);
}

static int sun5i_pm_finish(unsigned long arg)
{
	/* mark the resume image as armed */
	writel(SUN5I_STANDBY_MAGIC, pm.timer + SUN5I_STANDBY_GP_MAGIC);

	flush_cache_all();

	/* never returns unless the PMIC write failed */
	return pm.standby(pm.dramc, pm.twi, pm.axp_sleep_val);
}

static int sun5i_pm_enter(suspend_state_t state)
{
	int ret;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;

	save_regs(pm.ccu, pm.ccu_save, SUN5I_CCU_WORDS);
	save_regs(pm.sramc, pm.sramc_save, SUN5I_SRAMC_WORDS);
	save_regs(pm.intc, pm.intc_save, SUN5I_INTC_WORDS);
	save_regs(pm.pio, pm.pio_save, SUN5I_PIO_WORDS);
	save_regs(pm.timer, pm.timer_save, SUN5I_TIMER_WORDS);
	save_regs(pm.hstimer, pm.hstimer_save, SUN5I_HSTIMER_WORDS);
	memcpy(pm.training_save, pm.dram_start, sizeof(pm.training_save));

	writel(readl(pm.dramc + SUN5I_DRAMC_SDR_ZQSR) & 0xfffff,
	       pm.timer + SUN5I_STANDBY_GP_ZQ);
	writel(virt_to_phys(cpu_resume), pm.timer + SUN5I_STANDBY_GP_RESUME);
	writel(readl(pm.dramc + SUN5I_DRAMC_SDR_DCR),
	       pm.timer + SUN5I_STANDBY_GP_DCR);

	ret = sun5i_axp_prepare();
	if (ret) {
		pr_err("sun5i-pm: PMIC not reachable (%d)\n", ret);
		return ret;
	}

	pm.standby = fncpy(pm.sram, &sun5i_standby_blob,
			   sun5i_standby_blob_sz);

	cpu_pm_enter();
	ret = cpu_suspend(0, sun5i_pm_finish);
	cpu_pm_exit();

	/* back from the SPL (or a failed power-down) */
	writel(0, pm.timer + SUN5I_STANDBY_GP_MAGIC);

	if (!ret) {
		/* stop the rescue watchdog armed by the SPL */
		writel(0, pm.timer + SUN5I_TIMER_WDOG_MODE);

		sun5i_ccu_restore();
		restore_regs(pm.sramc, pm.sramc_save, SUN5I_SRAMC_WORDS);
		restore_regs(pm.intc, pm.intc_save, SUN5I_INTC_WORDS);
		sun5i_pio_restore();
		sun5i_timer_restore();
		sun5i_hstimer_restore();
		memcpy(pm.dram_start, pm.training_save,
		       sizeof(pm.training_save));
	} else {
		pr_err("sun5i-pm: power-down failed, suspend aborted\n");
		ret = -EIO;
	}

	return ret;
}

static int sun5i_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

static const struct platform_suspend_ops sun5i_pm_ops = {
	.valid = sun5i_pm_valid,
	.enter = sun5i_pm_enter,
};

static int __init sun5i_pm_init(void)
{
	struct device_node *np;

	if (!of_machine_is_compatible("allwinner,sun5i-a13"))
		return 0;

	/* without the PMIC there is no way to power down or wake up */
	np = of_find_compatible_node(NULL, NULL, "x-powers,axp209");
	if (!np)
		return 0;
	of_node_put(np);

	pm.sram = __arm_ioremap_exec(SUN5I_SRAM_A1_BASE, SZ_16K, false);
	pm.dramc = ioremap(SUN5I_DRAMC_BASE, SZ_4K);
	pm.ccu = ioremap(SUN5I_CCU_BASE, SZ_1K);
	pm.intc = ioremap(SUN5I_INTC_BASE, SZ_1K);
	pm.pio = ioremap(SUN5I_PIO_BASE, SZ_1K);
	pm.timer = ioremap(SUN5I_TIMER_BASE, SZ_1K);
	pm.sramc = ioremap(SUN5I_SRAMC_BASE, SZ_1K);
	pm.twi = ioremap(SUN5I_TWI0_BASE, SZ_1K);
	pm.hstimer = ioremap(SUN5I_HSTIMER_BASE, SZ_4K);
	pm.dram_start = memremap(SUN5I_DRAM_BASE,
				 sizeof(pm.training_save), MEMREMAP_WB);

	if (!pm.sram || !pm.dramc || !pm.ccu || !pm.intc || !pm.pio ||
	    !pm.timer || !pm.sramc || !pm.twi || !pm.hstimer ||
	    !pm.dram_start) {
		pr_err("sun5i-pm: mapping failed\n");
		return -ENOMEM;
	}

	suspend_set_ops(&sun5i_pm_ops);
	pr_info("sun5i-pm: suspend to RAM available\n");

	return 0;
}
late_initcall(sun5i_pm_init);
