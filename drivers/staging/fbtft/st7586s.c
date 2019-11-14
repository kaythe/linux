// SPDX-License-Identifier: GPL-2.0+
/*
 * FB driver for the st7586s LCD Controller
 *
 * Copyright (C) 2015 Dennis Menschel
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "fbtft.h"

#define DRVNAME "fb_st7586s"

void write_data_command(struct fbtft_par *par, unsigned dc, u32 val)
{
	int ret;

	if (par->gpio.dc != -1)
		gpio_set_value(par->gpio.dc, dc);

	*par->buf = (u8)val;

	ret = par->fbtftops.write(par, par->buf, 1);
}

/**
 * init_display() - initialize the display controller
 *
 * @par: FBTFT parameter object
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int init_display(struct fbtft_par *par)
{
  fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	par->fbtftops.reset(par);

	mdelay(550);
  
  gpio_set_value(par->gpio.dc, 0);
  
	/* turn off sleep mode */
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);

	/* set pixel format to RGB-565 */
	write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

	write_reg(par, PORCTRL, 0x08, 0x08, 0x00, 0x22, 0x22);

	/*
	 * VGH = 13.26V
	 * VGL = -10.43V
	 */
	write_reg(par, GCTRL, 0x35);

	/*
	 * VDV and VRH register values come from command write
	 * (instead of NVM)
	 */
	write_reg(par, VDVVRHEN, 0x01, 0xFF);

	/*
	 * VAP =  4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 * VAN = -4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 */
	write_reg(par, VRHS, 0x0B);

	/* VDV = 0V */
	write_reg(par, VDVS, 0x20);

	/* VCOM = 0.9V */
	write_reg(par, VCOMS, 0x20);

	/* VCOM offset = 0V */
	write_reg(par, VCMOFSET, 0x20);

	/*
	 * AVDD = 6.8V
	 * AVCL = -4.8V
	 * VDS = 2.3V
	 */
	write_reg(par, PWCTRL1, 0xA4, 0xA1);

	write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

/**
 * set_var() - apply LCD properties like rotation and BGR mode
 *
 * @par: FBTFT parameter object
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_var(struct fbtft_par *par)
{
  u16 *vmem16 = (u16 *)par->info->screen_base;
	u8 *buf = par->txbuf.buf;
	u8 *p_buf = par->txbuf.buf;
	int x, y, i;
	int ret = 0;
	char p, c;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s()\n", __func__);

	for (y=0;y<8;y++) {
	for (x=0;x<132;x++) {
			*buf = 0x00;
			for (i=0;i<8;i++) {
				*buf |= (vmem16[(y*8+i)*132+x] ? 1 : 0) << i;
			}
			buf++;
		}
	}

	for(p = 0; p < 8; p++) {
		write_data_command(par,0 ,CMD_SET_PAGE | p);
		write_data_command(par,0 ,CMD_SET_COLUMN_LOWER | ((1) & 0xf));
		write_data_command(par,0 ,CMD_SET_COLUMN_UPPER | (((1) >> 4) & 0x0F));
		write_data_command(par,0 ,CMD_RMW);
	    for(c = 0; c < 132; c++) {
			write_data_command(par,1, *p_buf);
	    	p_buf++;
	    }
	  }

	return ret;
}

/**
 * blank() - blank the display
 *
 * @par: FBTFT parameter object
 * @on: whether to enable or disable blanking the display
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int blank(struct fbtft_par *par, bool on)
{
	if (on)
		write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
	else
		write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par,
		"%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

	// TODO : implement set_addr_win

}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 240,
	.height = 320,
	.gamma_num = 1,
	.gamma_len = 1,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = init_display,
    .set_addr_win = set_addr_win,
		.set_var = set_var,
    .write_vmem = write_vmem,
		.set_gamma = set_gamma,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, &display);

MODULE_ALIAS("spi:" DRVNAME);

MODULE_DESCRIPTION("FB driver for the ST7586S LCD Controller");
MODULE_AUTHOR("Kayla Theil");
MODULE_LICENSE("GPL");
