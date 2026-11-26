// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 I2C controller driver
 */

#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define S5L8702_I2C_CON   0x0  /* Control register */
#define S5L8702_I2C_STAT  0x4  /* Control/status register */
#define S5L8702_I2C_ADD   0x8  /* Bus address register */
#define S5L8702_I2C_DS    0xc  /* Transmit/receive data shift register */
#define S5L8702_I2C_BUSY  0x10
#define S5L8702_I2C_UNK14 0x14
#define S5L8702_I2C_UNK18 0x18
#define S5L8702_I2C_INT   0x20 /* Interrupt status register */
#define S5L8702_I2C_UNK28 0x28

#define S5L8702_I2C_CON_CK_REG(x)		((x) & 0xf)
#define S5L8702_I2C_CON_BUSHOLD  		BIT(4)
#define S5L8702_I2C_CON_CKSEL16  		(0 << 6)
#define S5L8702_I2C_CON_CKSEL512 		BIT(6)
#define S5L8702_I2C_CON_ACKGEN   		BIT(7)
#define S5L8702_I2C_CON_INTEN_BUSHOLD 	BIT(8)
#define S5L8702_I2C_CON_INTEN_TIMEOUT 	BIT(9)
#define S5L8702_I2C_CON_INTEN_RX      	BIT(10)
#define S5L8702_I2C_CON_INTEN_TX      	BIT(11)
#define S5L8702_I2C_CON_INTEN_START   	BIT(12)
#define S5L8702_I2C_CON_INTEN_STOP    	BIT(13)
#define S5L8702_I2C_CON_INTEN_ALL 		(0x3f00)

#define S5L8702_I2C_STAT_LRB    BIT(0)
// The missing bits probably match S5L8700X datasheet ADDR_ZERO, AAS, LBA
#define S5L8702_I2C_STAT_SOE    BIT(4) // Serial Output Enable
#define S5L8702_I2C_STAT_BB     BIT(5)
#define S5L8702_I2C_STAT_TX     BIT(6)
#define S5L8702_I2C_STAT_MASTER BIT(7)

#define S5L8702_I2C_INT_BUSHOLD BIT(8)
#define S5L8702_I2C_INT_TIMEOUT BIT(9)
#define S5L8702_I2C_INT_RX      BIT(10)
#define S5L8702_I2C_INT_TX      BIT(11)
#define S5L8702_I2C_INT_START   BIT(12)
#define S5L8702_I2C_INT_STOP    BIT(13)
#define S5L8702_I2C_INT_ALL     (0x3f00)

#define S5L8702_I2C_XFER_TIMEOUT	(msecs_to_jiffies(100))

#define S5L8702_I2C_BUSY_LOOPS		5000

/* i2c controller state */
enum s5l8702_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_PREPARE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct s5l8702_i2c_dev {
	struct device *dev;
	void __iomem *regs;
	int irq;
	bool write_busy_poll;
	bool reinit;		/* suspended: program the controller before every transfer */
	/*
	 * The controller's clock gates (S5L8720: "clocks"), on only while a
	 * transfer runs, as iOS does: a clocked idle block costs power. A
	 * gated block must not be read (the bus hangs), so the interrupt
	 * handler only looks at the registers while clk_on is set.
	 *
	 * The interrupt line is level: a status bit still latched when the
	 * gate closes keeps it asserted, and only a clocked write clears it.
	 * So the gate is closed only after the status is cleared, and an
	 * interrupt taken while gated is acked with the clock on briefly. An
	 * "unhandled" return there would let the kernel disable the line, and
	 * every PMU transfer after that would time out (s2idle freeze).
	 */
	struct clk_bulk_data *clks;
	int nclks;
	bool clk_on;
	bool busy;		/* a transfer owns the state machine */
	bool s5l8720;
	u32 bus_freq;
	u32 clk_bits;
	u32 stop_delay_us;
	enum s5l8702_i2c_state state;
	struct i2c_msg	*msg;
	unsigned int msg_pos;
	unsigned int nmsgs;
	int msg_ret;
	unsigned int iiccon;
	unsigned int pending_irq;
	struct completion msg_complete;
	struct i2c_adapter adapter;
};

static inline u32 s5l8702_i2c_readl(struct s5l8702_i2c_dev *i2c_dev, u32 reg)
{
	return readl(i2c_dev->regs + reg);
}

static void s5l8702_i2c_writel(struct s5l8702_i2c_dev *i2c_dev, u32 reg, u32 val)
{
	if (i2c_dev->write_busy_poll) {
		unsigned int n = S5L8702_I2C_BUSY_LOOPS;

		while (readl(i2c_dev->regs + S5L8702_I2C_BUSY) && --n)
			cpu_relax();
	}
	writel(val, i2c_dev->regs + reg);
}

static void s5l8702_i2c_state_machine(struct s5l8702_i2c_dev *i2c_dev) {
	uint32_t stat;

again:
	switch ( i2c_dev->state )
	{

	  case STATE_START:
		i2c_dev->pending_irq = S5L8702_I2C_INT_BUSHOLD;
		if (i2c_dev->msg->flags & I2C_M_RD) {
			stat = S5L8702_I2C_STAT_SOE | S5L8702_I2C_STAT_MASTER;
		} else {
		  	stat = S5L8702_I2C_STAT_SOE | S5L8702_I2C_STAT_TX | S5L8702_I2C_STAT_MASTER;
		}
		i2c_dev->iiccon &= ~S5L8702_I2C_CON_BUSHOLD;
		i2c_dev->iiccon |= S5L8702_I2C_CON_ACKGEN;
		if (i2c_dev->s5l8720) {
			/*
			 * The S5L8720 only generates START when mode and START
			 * are written together, after the address.
			 */
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, i2c_8bit_addr_from_msg(i2c_dev->msg));
			stat |= S5L8702_I2C_STAT_BB;
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, stat);
		} else {
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, stat);
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, i2c_8bit_addr_from_msg(i2c_dev->msg));
			stat |= S5L8702_I2C_STAT_BB;
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, stat);
		}
		if (i2c_dev->msg->flags & I2C_M_RD) {
		  	i2c_dev->state = STATE_PREPARE_READ;
		}
		else {
			i2c_dev->state = STATE_WRITE;
		}
		break;

	  case STATE_WRITE: // Write
		if ( s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) & S5L8702_I2C_STAT_LRB ) { // Did we receive ACK?
		  	i2c_dev->msg_ret = -EIO;
		  	goto generate_stop;
		}
		if ( i2c_dev->msg_pos == i2c_dev->msg->len ) { // is the end of the msg
		  	goto generate_stop;
		}
		i2c_dev->pending_irq = S5L8702_I2C_INT_BUSHOLD;
		i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, i2c_dev->msg->buf[i2c_dev->msg_pos++]);
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		break;

	  case STATE_READ: // Read
		i2c_dev->msg->buf[i2c_dev->msg_pos++] = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_DS);
		fallthrough;
	  case STATE_PREPARE_READ: // Prepare read
		if ( !i2c_dev->msg_pos && (s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) & S5L8702_I2C_STAT_LRB) ) {
			i2c_dev->msg_ret = -EIO;
			goto generate_stop;
		}

		if ( i2c_dev->msg_pos == i2c_dev->msg->len ) { // is the end of the msg
			goto generate_stop;
		}

		if ( (i2c_dev->msg->len - i2c_dev->msg_pos) == 1 ) { // last byte of msg NACK
			i2c_dev->iiccon &= ~S5L8702_I2C_CON_ACKGEN;
		}
		i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
		i2c_dev->pending_irq = S5L8702_I2C_INT_BUSHOLD;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		i2c_dev->state = STATE_READ;
		break;

	  case STATE_STOP: // Generate Stop
generate_stop:
		/*
		 * S5L8720: between messages to the same device use a repeated
		 * START. A STOP followed straight away by a new START can be
		 * dropped by the controller, which then clocks the next
		 * address byte out as data (overwriting the selected reg).
		 */
		if (i2c_dev->s5l8720 && !i2c_dev->msg_ret && i2c_dev->nmsgs > 1 &&
		    i2c_dev->msg[1].addr == i2c_dev->msg[0].addr) {
			i2c_dev->pending_irq = S5L8702_I2C_INT_STOP;
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT,
					   S5L8702_I2C_STAT_MASTER | S5L8702_I2C_STAT_BB);
			i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
			s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
			i2c_dev->nmsgs--;
			i2c_dev->msg++;
			i2c_dev->msg_pos = 0;
			i2c_dev->state = STATE_START;
			break;
		}

		i2c_dev->pending_irq = S5L8702_I2C_INT_STOP;

		/* stat is only set by STATE_START; read the current value. */
		stat = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT);
		if (i2c_dev->msg->flags & I2C_M_RD) {
			stat &= ~S5L8702_I2C_STAT_BB;
		}
		else {
			stat &= ~( S5L8702_I2C_STAT_BB | S5L8702_I2C_STAT_TX );
		}
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, stat);
		i2c_dev->iiccon &= ~S5L8702_I2C_CON_ACKGEN;
		i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		i2c_dev->nmsgs--;
		i2c_dev->msg++;
		i2c_dev->msg_pos = 0;

		// If we have an error or we processed all messages then we are done
		if (i2c_dev->msg_ret || (i2c_dev->nmsgs == 0)) {
			i2c_dev->state = STATE_IDLE;
		} else {
			i2c_dev->state = STATE_START;
		}
		/*
		 * The S5L8720 raises no STOP interrupt. Give the STOP condition
		 * well over a bit time to finish before anything else: the bus
		 * runs at a few tens of kHz, and a START requested while STOP is
		 * still on the wire is dropped, so the next address byte goes
		 * out as data (a write to the selected register).
		 */
		if (i2c_dev->s5l8720) {
			i2c_dev->pending_irq = 0;
			if (irqs_disabled())
				udelay(i2c_dev->stop_delay_us);
			else
				usleep_range(i2c_dev->stop_delay_us,
					     i2c_dev->stop_delay_us + 20);
			goto again;
		}
		break;
		
	  case STATE_IDLE: // We are done
		i2c_dev->pending_irq = 0;
		complete(&i2c_dev->msg_complete);
		break;
	}
}

static irqreturn_t s5l8702_i2c_isr(int this_irq, void *data)
{
	struct s5l8702_i2c_dev *i2c_dev = data;
	u32 val;

	if (!READ_ONCE(i2c_dev->clk_on)) {
		/* latched before the gate closed: clear it with the clock on */
		if (clk_bulk_enable(i2c_dev->nclks, i2c_dev->clks))
			return IRQ_NONE;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_INT, S5L8702_I2C_INT_ALL);
		clk_bulk_disable(i2c_dev->nclks, i2c_dev->clks);
		return IRQ_HANDLED;
	}

	val = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_INT, val);

	/* stray status outside a transfer: acked, nothing to advance */
	if (!READ_ONCE(i2c_dev->busy))
		return IRQ_HANDLED;

	dev_dbg(i2c_dev->dev, "%s state=0x%04x msg_ret=0x%04x pending_irq=0x%04x val=0x%04x",
		__func__, i2c_dev->state, i2c_dev->msg_ret, i2c_dev->pending_irq, val);

	i2c_dev->pending_irq &= ~val;

	// [TODO] Is this the best way due to us getting other interrupts?
	if (!i2c_dev->pending_irq) {
		s5l8702_i2c_state_machine(i2c_dev);
	}

	return IRQ_HANDLED;
}

static int s5l8702_i2c_init(struct s5l8702_i2c_dev *i2c_dev);

static int s5l8702_i2c_do_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
				int num, bool poll)
{
	unsigned long time_left;
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);

	dev_dbg(i2c_dev->dev, "%s start", __func__);

	/*
	 * Back in its reset state (CON has no interrupt enables) after a suspend
	 * that cut the SoC's power, and before resume_noirq has run: the n72
	 * sleep code talks to the PMU that early. Program it again first.
	 */
	if (i2c_dev->reinit ||
	    !(s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON) & S5L8702_I2C_CON_INTEN_BUSHOLD))
		s5l8702_i2c_init(i2c_dev);

	// [TODO] implement clocks this is equivalent to set controller active and clear interrupts
	// but we are missing clock enable and disable
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK14, 1);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_INT, S5L8702_I2C_INT_ALL);

	int i;

	for (i = 0; i < num; i++) {
		dev_dbg(i2c_dev->dev, "%s addr=0x%04x flags=0x%04x len=%u buf=%02x",
			__func__, msgs[i].addr, msgs[i].flags, msgs[i].len, msgs[i].buf[0]);
	}

	i2c_dev->msg     = msgs;
	i2c_dev->nmsgs   = num;
	i2c_dev->msg_ret = 0;
	i2c_dev->msg_pos = 0;
	i2c_dev->state   = STATE_START;

	reinit_completion(&i2c_dev->msg_complete);
	WRITE_ONCE(i2c_dev->busy, true);
	s5l8702_i2c_state_machine(i2c_dev);

	if (poll || i2c_dev->irq < 0 || irqs_disabled()) {
		/*
		 * No interrupt wired up, or interrupts off (atomic transfers at
		 * power-off, the n72 sleep code on the way down and just after
		 * the wake): poll the interrupt status register and run the
		 * handler from here.
		 * Count iterations rather than jiffies, which do not advance
		 * with interrupts off.
		 */
		unsigned int loops = 100000 / 5;	/* about 100 ms */

		time_left = 1;
		while (!completion_done(&i2c_dev->msg_complete)) {
			if (s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT) &
			    S5L8702_I2C_INT_ALL)
				s5l8702_i2c_isr(0, i2c_dev);
			else
				udelay(5);
			if (!--loops) {
				time_left = 0;
				break;
			}
		}
	} else {
		time_left = wait_for_completion_timeout(&i2c_dev->msg_complete,
							S5L8702_I2C_XFER_TIMEOUT);
	}

	WRITE_ONCE(i2c_dev->busy, false);
	dev_dbg(i2c_dev->dev, "%s done time_left=0x%04lx msg_ret=0x%04x", __func__, time_left, i2c_dev->msg_ret);
	if (time_left == 0) {
		if (!poll && i2c_dev->irq >= 0 && !irqs_disabled())
			synchronize_irq(i2c_dev->irq);
		dev_warn_ratelimited(i2c_dev->dev,
			"timeout: addr 0x%02x state %d pending 0x%x msg %u/%d pos %u, CON 0x%x STAT 0x%x INT 0x%x UNK14 0x%x%s\n",
			msgs[0].addr, i2c_dev->state, i2c_dev->pending_irq,
			num - i2c_dev->nmsgs, num, i2c_dev->msg_pos,
			s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON),
			s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT),
			s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT),
			s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_UNK14),
			poll ? " (polled)" : "");
		/* leave the controller idle and programmed for the next one */
		s5l8702_i2c_init(i2c_dev);
		return -ETIMEDOUT;
	}

	return i2c_dev->msg_ret ? : num;
}

/* clk_enable() only takes a spinlock: fine in the atomic (power-off) path */
static int s5l8702_i2c_clk_on(struct s5l8702_i2c_dev *i2c_dev)
{
	int ret = clk_bulk_enable(i2c_dev->nclks, i2c_dev->clks);

	if (!ret)
		WRITE_ONCE(i2c_dev->clk_on, true);
	return ret;
}

static void s5l8702_i2c_clk_off(struct s5l8702_i2c_dev *i2c_dev)
{
	unsigned long flags;

	/* clear the level line while clocked, then gate (see clk_on) */
	local_irq_save(flags);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_INT, S5L8702_I2C_INT_ALL);
	WRITE_ONCE(i2c_dev->clk_on, false);
	clk_bulk_disable(i2c_dev->nclks, i2c_dev->clks);
	local_irq_restore(flags);
}

static int s5l8702_i2c_gated_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
				  int num, bool poll)
{
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret = s5l8702_i2c_clk_on(i2c_dev);

	if (ret)
		return ret;
	ret = s5l8702_i2c_do_xfer(adap, msgs, num, poll);
	s5l8702_i2c_clk_off(i2c_dev);
	return ret;
}

static int s5l8702_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{
	return s5l8702_i2c_gated_xfer(adap, msgs, num, false);
}

/* Used for PMIC writes at power-off, with interrupts disabled. */
static int s5l8702_i2c_xfer_atomic(struct i2c_adapter *adap,
				   struct i2c_msg msgs[], int num)
{
	return s5l8702_i2c_gated_xfer(adap, msgs, num, true);
}

static u32 s5l8702_i2c_func(struct i2c_adapter *adap)
{
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);

	dev_dbg(i2c_dev->dev, "%s", __func__);

	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm s5l8702_i2c_algo = {
	.xfer = s5l8702_i2c_xfer,
	.xfer_atomic = s5l8702_i2c_xfer_atomic,
	.functionality = s5l8702_i2c_func,
};

static int s5l8702_i2c_init(struct s5l8702_i2c_dev *i2c_dev) {
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_ADD, 0x40); // [TODO] Get slave address from DT

	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK14, 1);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK18, 0);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, S5L8702_I2C_STAT_MASTER);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, 0);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, 0); 
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, 0x40); // [TODO] Get slave address from DT

	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON,
		S5L8702_I2C_CON_INTEN_BUSHOLD | S5L8702_I2C_CON_ACKGEN |
		i2c_dev->clk_bits);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, S5L8702_I2C_STAT_SOE); 
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK28, 0);

	i2c_dev->iiccon = S5L8702_I2C_CON_INTEN_STOP | S5L8702_I2C_CON_INTEN_BUSHOLD |
		i2c_dev->clk_bits;
	/* Keep every status interrupt source enabled on the S5L8720. */
	if (i2c_dev->s5l8720)
		i2c_dev->iiccon |= S5L8702_I2C_CON_INTEN_ALL;

	return 0;
}

static void s5l8702_i2c_unprepare(void *data)
{
	struct s5l8702_i2c_dev *i2c_dev = data;

	clk_bulk_unprepare(i2c_dev->nclks, i2c_dev->clks);
}

static int s5l8702_i2c_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s", __func__);
	struct s5l8702_i2c_dev *i2c_dev;
	int ret;
	struct i2c_adapter *adap;

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;

	i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	i2c_dev->s5l8720 = of_device_is_compatible(pdev->dev.of_node,
						   "apple,s5l8720-i2c");
	i2c_dev->write_busy_poll = of_property_read_bool(pdev->dev.of_node,
							  "samsung,write-busy-poll");

	i2c_dev->nclks = devm_clk_bulk_get_all(&pdev->dev, &i2c_dev->clks);
	if (i2c_dev->nclks < 0)
		return dev_err_probe(&pdev->dev, i2c_dev->nclks, "clocks\n");
	ret = clk_bulk_prepare(i2c_dev->nclks, i2c_dev->clks);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, s5l8702_i2c_unprepare, i2c_dev);
	if (ret)
		return ret;

	{
		u32 freq = 100000;

		of_property_read_u32(pdev->dev.of_node, "clock-frequency", &freq);
		freq = clamp_t(u32, freq, 10000, 400000);
		i2c_dev->bus_freq = freq;
		if (i2c_dev->s5l8720) {
			unsigned long src_rate = i2c_dev->nclks > 0 ?
						 clk_get_rate(i2c_dev->clks[0].clk) : 0;
			u32 ticks, cksel, div, actual_div, pre, actual_hz;

			if (!src_rate)
				src_rate = 24000000;
			ticks = DIV_ROUND_UP(src_rate, freq);
			if (ticks <= 512) {
				cksel = S5L8702_I2C_CON_CKSEL16;
				div = DIV_ROUND_UP(ticks, 32);
				actual_div = 32;
			} else {
				cksel = S5L8702_I2C_CON_CKSEL512;
				div = DIV_ROUND_UP(ticks, 512);
				actual_div = 512;
			}
			pre = clamp_t(u32, div > 0 ? div - 1 : 1, 1, 15);
			i2c_dev->clk_bits = cksel | S5L8702_I2C_CON_CK_REG(pre);
			actual_hz = src_rate / (actual_div * (pre + 1));
			i2c_dev->stop_delay_us = max_t(u32, 50,
						       DIV_ROUND_UP(12000000U, actual_hz));
		} else {
			i2c_dev->clk_bits = S5L8702_I2C_CON_CKSEL512 |
					    S5L8702_I2C_CON_CK_REG(0);
			i2c_dev->stop_delay_us = 500;
		}
	}

	ret = s5l8702_i2c_clk_on(i2c_dev);
	if (ret)
		return ret;
	ret = s5l8702_i2c_init(i2c_dev);
	s5l8702_i2c_clk_off(i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could initialize I2C controller\n");
		goto err;
	}

	init_completion(&i2c_dev->msg_complete);

	/* Without an "interrupts" property the controller is polled. */
	i2c_dev->irq = platform_get_irq_optional(pdev, 0);
	if (i2c_dev->irq == -EPROBE_DEFER) {
		ret = i2c_dev->irq;
		goto err;
	}
	if (i2c_dev->irq >= 0) {
		/*
		 * Kept enabled through the noirq phases: the bus carries the
		 * PMU, which late users still reach there (the RTC core arms
		 * the next alarm from a work item that suspend does not freeze).
		 * With the line disabled under a running transfer, that
		 * transfer timed out and the wake alarm was lost.
		 */
		ret = devm_request_threaded_irq(&pdev->dev, i2c_dev->irq, NULL,
						s5l8702_i2c_isr,
						IRQF_ONESHOT | IRQF_NO_SUSPEND,
						dev_name(&pdev->dev), i2c_dev);
		if (ret) {
			dev_err(&pdev->dev, "Could not request IRQ\n");
			goto err;
		}
	} else {
		dev_info(&pdev->dev, "no interrupt, polling\n");
	}

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "s5l8702 (%s)",
		 dev_name(&pdev->dev));
	adap->algo = &s5l8702_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	ret = devm_i2c_add_adapter(&pdev->dev, adap);
	if (ret)
		goto err;

	return 0;

err:
	return ret;
}

/*
 * After a suspend that cut the SoC's power (n72 suspend to RAM) the
 * controller is back in its reset state: program it again. Harmless
 * otherwise, and early (noirq), before any client resumes.
 */
static int s5l8702_i2c_resume_noirq(struct device *dev)
{
	struct s5l8702_i2c_dev *i2c_dev = dev_get_drvdata(dev);

	int ret;

	i2c_dev->reinit = false;
	ret = s5l8702_i2c_clk_on(i2c_dev);
	if (ret)
		return ret;
	ret = s5l8702_i2c_init(i2c_dev);
	s5l8702_i2c_clk_off(i2c_dev);
	return ret;
}

/*
 * The boot loader may use the controller on the way back (LLB talks to the
 * PMU) and leave it set up its own way. Until resume_noirq, every transfer
 * programs it again first: the n72 sleep code writes the PMU before then,
 * both on the way down and just after the wake.
 */
static int s5l8702_i2c_suspend_noirq(struct device *dev)
{
	struct s5l8702_i2c_dev *i2c_dev = dev_get_drvdata(dev);

	i2c_dev->reinit = true;
	return 0;
}

static const struct dev_pm_ops s5l8702_i2c_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(s5l8702_i2c_suspend_noirq, s5l8702_i2c_resume_noirq)
};

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_i2c_of_match[] = {
	{ .compatible = "samsung,s5l8702-i2c" },
	{ .compatible = "apple,s5l8720-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, s5l8702_i2c_of_match);
#endif

static struct platform_driver s5l8702_i2c_driver = {
	.probe		= s5l8702_i2c_probe,
	.driver		= {
		.name	= "i2c-s5l8702",
		.of_match_table = of_match_ptr(s5l8702_i2c_of_match),
		.pm	= pm_sleep_ptr(&s5l8702_i2c_pm_ops),
	},
};
module_platform_driver(s5l8702_i2c_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 I2C bus adapter");
MODULE_LICENSE("GPL v2");
