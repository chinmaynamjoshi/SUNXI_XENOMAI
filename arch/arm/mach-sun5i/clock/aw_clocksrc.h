/*
 * arch/arm/mach-sun5i/clock/ccmu/aw_clocksrc.h
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Kevin Zhang <kevin@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __AW_CLOCKSRC_H__
#define __AW_CLOCKSRC_H__

#ifndef __tmr_reg
    #define __tmr_reg(x)    (*(volatile __u32 *)(x))
#endif  /*#ifndef __tmr_reg */


/* define timer io base on aw chips */
#define AW_TMR_IO_BASE          SW_VA_TIMERC_IO_BASE
/* define timer io register address */
#define TMR_REG_o_IRQ_EN        (AW_TMR_IO_BASE + 0x0000)
#define TMR_REG_o_IRQ_STAT      (AW_TMR_IO_BASE + 0x0004)
#define TMR_REG_o_TMR0_CTL      (AW_TMR_IO_BASE + 0x0010)
#define TMR_REG_o_TMR0_INTV     (AW_TMR_IO_BASE + 0x0014)
#define TMR_REG_o_TMR0_CUR      (AW_TMR_IO_BASE + 0x0018)
#define TMR_REG_o_TMR1_CTL      (AW_TMR_IO_BASE + 0x0020)
#define TMR_REG_o_TMR1_INTV     (AW_TMR_IO_BASE + 0x0024)
#define TMR_REG_o_TMR1_CUR      (AW_TMR_IO_BASE + 0x0028)
#define TMR_REG_o_TMR2_CTL      (AW_TMR_IO_BASE + 0x0030)
#define TMR_REG_o_TMR2_INTV     (AW_TMR_IO_BASE + 0x0034)
#define TMR_REG_o_TMR2_CUR      (AW_TMR_IO_BASE + 0x0038)
#define TMR_REG_o_TMR3_CTL      (AW_TMR_IO_BASE + 0x0040)
#define TMR_REG_o_TMR3_INTV     (AW_TMR_IO_BASE + 0x0044)
#define TMR_REG_o_TMR3_CUR      (AW_TMR_IO_BASE + 0x0048)
#define TMR_REG_o_CNT64_CTL     (AW_TMR_IO_BASE + 0x00A0)
#define TMR_REG_o_CNT64_LO      (AW_TMR_IO_BASE + 0x00A4)
#define TMR_REG_o_CNT64_HI      (AW_TMR_IO_BASE + 0x00A8)
/* define timer io register value */
#define TMR_REG_IRQ_EN          __tmr_reg(TMR_REG_o_IRQ_EN   )
#define TMR_REG_IRQ_STAT        __tmr_reg(TMR_REG_o_IRQ_STAT )
#define TMR_REG_TMR0_CTL        __tmr_reg(TMR_REG_o_TMR0_CTL )
#define TMR_REG_TMR0_INTV       __tmr_reg(TMR_REG_o_TMR0_INTV)
#define TMR_REG_TMR0_CUR        __tmr_reg(TMR_REG_o_TMR0_CUR )
#define TMR_REG_TMR1_CTL        __tmr_reg(TMR_REG_o_TMR1_CTL )
#define TMR_REG_TMR1_INTV       __tmr_reg(TMR_REG_o_TMR1_INTV)
#define TMR_REG_TMR1_CUR        __tmr_reg(TMR_REG_o_TMR1_CUR )
#define TMR_REG_TMR2_CTL        __tmr_reg(TMR_REG_o_TMR2_CTL )
#define TMR_REG_TMR2_INTV       __tmr_reg(TMR_REG_o_TMR2_INTV)
#define TMR_REG_TMR2_CUR        __tmr_reg(TMR_REG_o_TMR2_CUR )
#define TMR_REG_TMR3_CTL        __tmr_reg(TMR_REG_o_TMR3_CTL )
#define TMR_REG_TMR3_INTV       __tmr_reg(TMR_REG_o_TMR3_INTV)
#define TMR_REG_TMR3_CUR        __tmr_reg(TMR_REG_o_TMR3_CUR )
#define TMR_REG_CNT64_CTL       __tmr_reg(TMR_REG_o_CNT64_CTL)
#define TMR_REG_CNT64_LO        __tmr_reg(TMR_REG_o_CNT64_LO )
#define TMR_REG_CNT64_HI        __tmr_reg(TMR_REG_o_CNT64_HI )


/* define timer clock source */
#define TMR_CLK_SRC_32KLOSC     (0)
#define TMR_CLK_SRC_24MHOSC     (1)
#define TMR_CLK_SRC_PLL         (2)


/* config clock frequency   */
#define AW_HPET_CLK_SRC     TMR_CLK_SRC_24MHOSC
#define AW_HPET_CLK_EVT     TMR_CLK_SRC_24MHOSC


/* aw HPET clock source frequency */
#ifndef AW_HPET_CLK_SRC
    #error "AW_HPET_CLK_SRC is not define!!"
#endif
#if(AW_HPET_CLK_SRC == TMR_CLK_SRC_24MHOSC)
    #define AW_HPET_CLOCK_SOURCE_HZ         (24000000)
#else
    #error "AW_HPET_CLK_SRC config is invalid!!"
#endif


/* Setup IPIPE Freq */
#ifdef CONFIG_IPIPE
  /* http://linux-sunxi.org/images/3/33/A10_Datasheet.pdf p.53 PLL6 fixed at 1.2GHz */
  /* Set TMR_REG_TMR1_CTL |= (2<<4); in aw_clocksrc.c for PLL/4 for 300MHz ticks */
  #define TIMER2_HPET_CLOCK_EVENT_HZ          (300000000)
#endif /* CONFIG_IPIPE */


/* aw HPET clock eventy frequency */
#ifndef AW_HPET_CLK_EVT
    #error "AW_HPET_CLK_EVT is not define!!"
#endif
#if(AW_HPET_CLK_EVT == TMR_CLK_SRC_32KLOSC)
    #define AW_HPET_CLOCK_EVENT_HZ          (32768)
#elif(AW_HPET_CLK_EVT == TMR_CLK_SRC_24MHOSC)
    #define AW_HPET_CLOCK_EVENT_HZ          (24000000)
#else
    #error "AW_HPET_CLK_EVT config is invalid!!"
#endif


#endif  /* #ifndef __AW_CLOCKSRC_H__ */

