#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdio_wrap.h"


#include <generated/csr.h>
#include <generated/mem.h>
#include <time.h>

#include "hdmi_in0.h"
#include "mmcm.h"
#include "processor.h"
#include "edid.h"

/*
 ----------------->>> Time ----------->>>

                  +-------------------+
   Video          |  Blanking         |  Video
                  |                   |
 ----(a)--------->|<-------(b)------->|
                  |                   |
                  |       +-------+   |
                  |       | Sync  |   |
                  |       |       |   |
                  |<-(c)->|<-(d)->|   |
                  |       |       |   |
 ----(1)--------->|       |       |   |
 ----(2)----------------->|       |   |
 ----(3)------------------------->|   |
 ----(4)----------------------------->|
                  |       |       |   |

 (a) - h_active ==(1)
 (b) - h_blanking
 (c) - h_sync_offset
 (d) - h_sync_width
 (1) - HDisp / width == (a)
 (2) - HSyncStart
 (3) - HSyncEnd
 (4) - HTotal

 Modeline "String description" Dot-Clock HDisp HSyncStart HSyncEnd HTotal VDisp VSyncStart VSyncEnd VTotal [options]

                            (1|a)    (2)    (3)  (4)
 ModeLine "640x480"    31.5  640    664    704  832    480  489  491  520
                              |\-(c)-/\-(d)-/    |
                              |   24    40       |
                              |                  |
                              \--------(b)-------/
                                       192

References:
 * http://www.arachnoid.com/modelines/
 * http://martin.hinner.info/vga/timing.html
 * VESA Modes - http://cvsweb.xfree86.org/cvsweb/xc/programs/Xserver/hw/xfree86/etc/vesamodes
 * 720p and TV modes - https://www.mythtv.org/wiki/Modeline_Database

*/
const struct video_timing video_modes[PROCESSOR_MODE_COUNT] = {
	// 640x480 @ 72Hz (VESA) hsync: 37.9kHz
	// ModeLine "640x480"    31.5  640  664  704  832    480  489  491  520
	//                                24   40  <192          9   2   <40
	{
		.pixel_clock = 3150,

		.h_active = 640,
		.h_blanking = 192,
		.h_sync_offset = 24,
		.h_sync_width = 40,

		.v_active = 480,
		.v_blanking = 40,
		.v_sync_offset = 9,
		.v_sync_width = 3,

		.established_timing = 0x0800
	},
	// 640x480 @ 75Hz (VESA) hsync: 37.5kHz
	// ModeLine "640x480"    31.5  640  656  720  840    480  481  484  500
	//                                16   64  <200         1    3   <20
	{
		.pixel_clock = 3150,

		.h_active = 640,
		.h_blanking = 200,
		.h_sync_offset = 16,
		.h_sync_width = 64,

		.v_active = 480,
		.v_blanking = 20,
		.v_sync_offset = 1,
		.v_sync_width = 3,

		.established_timing = 0x0400
	},
	// 800x600 @ 56Hz (VESA) hsync: 35.2kHz
	// ModeLine "800x600"    36.0  800  824  896 1024    600  601  603  625
	{
		.pixel_clock = 3600,

		.h_active = 800,
		.h_blanking = 224,
		.h_sync_offset = 24,
		.h_sync_width = 72,

		.v_active = 600,
		.v_blanking = 25,
		.v_sync_offset = 1,
		.v_sync_width = 2,

		.established_timing = 0x0200
	},
	// 800x600 @ 60Hz (VESA) hsync: 37.9kHz
	// ModeLine "800x600"    40.0  800  840  968 1056    600  601  605  628
	{
		.pixel_clock = 4000,

		.h_active = 800,
		.h_blanking = 256,
		.h_sync_offset = 40,
		.h_sync_width = 128,

		.v_active = 600,
		.v_blanking = 28,
		.v_sync_offset = 1,
		.v_sync_width = 4,

		.established_timing = 0x0100
	},
	// 800x600 @ 72Hz (VESA) hsync: 48.1kHz
	// ModeLine "800x600"    50.0  800  856  976 1040    600  637  643  666
	{
		.pixel_clock = 5000,

		.h_active = 800,
		.h_blanking = 240,
		.h_sync_offset = 56,
		.h_sync_width = 120,

		.v_active = 600,
		.v_blanking = 66,
		.v_sync_offset = 37,
		.v_sync_width = 6,

		.established_timing = 0x0080
	},
	// 800x600 @ 75Hz (VESA) hsync: 46.9kHz
	// ModeLine "800x600"    49.5  800  816  896 1056    600  601  604  625
	{
		.pixel_clock = 4950,

		.h_active = 800,
		.h_blanking = 256,
		.h_sync_offset = 16,
		.h_sync_width = 80,

		.v_active = 600,
		.v_blanking = 25,
		.v_sync_offset = 1,
		.v_sync_width = 3,

		.established_timing = 0x0040
	},
	// 1024x768 @ 60Hz (VESA) hsync: 48.4kHz
	// ModeLine "1024x768"   65.0 1024 1048 1184 1344    768  771  777  806
	{
		.pixel_clock = 6500,

		.h_active = 1024,
		.h_blanking = 320,
		.h_sync_offset = 24,
		.h_sync_width = 136,

		.v_active = 768,
		.v_blanking = 38,
		.v_sync_offset = 3,
		.v_sync_width = 6,

		.established_timing = 0x0008
	},
	// 1024x768 @ 70Hz (VESA) hsync: 56.5kHz
	// ModeLine "1024x768"   75.0 1024 1048 1184 1328    768  771  777  806
	{
		.pixel_clock = 7500,

		.h_active = 1024,
		.h_blanking = 304,
		.h_sync_offset = 24,
		.h_sync_width = 136,

		.v_active = 768,
		.v_blanking = 38,
		.v_sync_offset = 3,
		.v_sync_width = 6,

		.established_timing = 0x0004
	},
	// 1024x768 @ 75Hz (VESA) hsync: 60.0kHz
	// ModeLine "1024x768"   78.8 1024 1040 1136 1312    768  769  772  800
	{
		.pixel_clock = 7880,

		.h_active = 1024,
		.h_blanking = 288,
		.h_sync_offset = 16,
		.h_sync_width = 96,

		.v_active = 768,
		.v_blanking = 32,
		.v_sync_offset = 1,
		.v_sync_width = 3,

		.established_timing = 0x0002
	},
	// 720p @ 60Hz
	//1280	720	60 Hz	45 kHz		ModeLine "1280x720"		74.25  1280 1390 1430 1650 720 725 730 750 +HSync +VSync
	{
		.pixel_clock = 7425,

		.h_active = 1280,
		.h_blanking = 370,
		.h_sync_offset = 220,
		.h_sync_width = 40,

		.v_active = 720,
		.v_blanking = 30,
		.v_sync_offset = 20,
		.v_sync_width = 5
	},
	// Other 720p60 modes not enabled...
	//1280	720	60 Hz	44.9576 kHz	ModeLine "1280x720"		74.18  1280 1390 1430 1650 720 725 730 750 +HSync +VSync
	//1280	720	59.94			ModeLine "ATSC-720-59.94p"	74.176 1280 1320 1376 1650 720 722 728 750
	//1280	720	60 Hz			ModeLine "ATSC-720-60p"		74.25  1280 1320 1376 1650 720 722 728 750

	// 720p @ 50Hz
	// 19 720p50    16:9     1:1        1280x720p @ 50 Hz
	//1280	720	50 Hz	37.5 kHz	ModeLine "1280x720"		74.25  1280 1720 1760 1980 720 725 730 750 +HSync +VSync
	//                                                                                440   40  <700      5   5   <30
	{
		.pixel_clock = 7425,

		.h_active = 1280,
		.h_blanking = 700,
		.h_sync_offset = 440,
		.h_sync_width = 40,

		.v_active = 720,
		.v_blanking = 30,
		.v_sync_offset = 5,
		.v_sync_width = 5
	},
	// 1920x1080 @ 60.00 Hz    Modeline: "1920x1080" 148.500 1920 2008 2052 2200 1080 1084 1089 1125 +hsync +vsync
	{
		.pixel_clock = 14850,

		.h_active = 1920,
		.h_blanking = 280,
		.h_sync_offset = 88,
		.h_sync_width = 44,

		.v_active = 1080,
		.v_blanking = 45,
		.v_sync_offset = 4,
		.v_sync_width = 5
	},
	// "fake" 1920x1080 @ 60.00 Hz    Modeline: "1920x1080" 148.500 1920 2008 2052 2200 1080 1084 1089 1125 +hsync +vsync
	{
		.pixel_clock = 7425,

		.h_active = 1920,
		.h_blanking = 280,
		.h_sync_offset = 88,
		.h_sync_width = 44,

		.v_active = 1080,
		.v_blanking = 45,
		.v_sync_offset = 4,
		.v_sync_width = 5
	},
	// 720x480 @ 60.00 Hz    Modeline "720x480" 26.72 720 736 808 896 480 481 484 497 -HSync +Vsync
	{
		.pixel_clock = 2672,

		.h_active = 720,
		.h_blanking = 176,
		.h_sync_offset = 16,
		.h_sync_width = 72,

		.v_active = 480,
		.v_blanking = 17,
		.v_sync_offset = 1,
		.v_sync_width = 3,
		.comment = "(HV20/HV30 in NTSC mode)"
	},
	// 720x576 @ 50.00 Hz    Modeline 720x576" 32.67 720 744 816 912 576 577 580 597 -HSync +Vsyncc
	{
		.pixel_clock = 3267,

		.h_active = 720,
		.h_blanking = 192,
		.h_sync_offset = 24,
		.h_sync_width = 72,

		.v_active = 576,
		.v_blanking = 21,
		.v_sync_offset = 1,
		.v_sync_width = 3,
		.comment = "(HV20/HV30 in PAL mode)"
	}

};

static void fb_clkgen_write(int m, int d)
{
	/* clkfbout_mult = m */
	if(m%2)
		hdmi_out0_mmcm_write(0x14, 0x1000 | ((m/2)<<6) | (m/2 + 1));
	else
		hdmi_out0_mmcm_write(0x14, 0x1000 | ((m/2)<<6) | m/2);
	/* divclk_divide = d */
	if (d == 1)
		hdmi_out0_mmcm_write(0x16, 0x1000);
	else if(d%2)
		hdmi_out0_mmcm_write(0x16, ((d/2)<<6) | (d/2 + 1));
	else
		hdmi_out0_mmcm_write(0x16, ((d/2)<<6) | d/2);
	/* clkout0_divide = 10 */
	hdmi_out0_mmcm_write(0x8, 0x1000 | (5<<6) | 5);
	/* clkout1_divide = 2 */
	hdmi_out0_mmcm_write(0xa, 0x1000 | (1<<6) | 1);
}

/* FIXME: add vco frequency check */
static void fb_get_clock_md(unsigned int pixel_clock, unsigned int *best_m, unsigned int *best_d)
{
	unsigned int ideal_m, ideal_d;
	unsigned int bm, bd;
	unsigned int m, d;
	unsigned int diff_current;
	unsigned int diff_tested;

	ideal_m = pixel_clock;
	ideal_d = 10000;

	bm = 1;
	bd = 0;
	for(d=1;d<=128;d++)
		for(m=2;m<=128;m++) {
			/* common denominator is d*bd*ideal_d */
			diff_current = abs(d*ideal_d*bm - d*bd*ideal_m);
			diff_tested = abs(bd*ideal_d*m - d*bd*ideal_m);
			if(diff_tested < diff_current) {
				bm = m;
				bd = d;
			}
		}
	*best_m = bm;
	*best_d = bd;
}

static void fb_set_mode(const struct video_timing *mode)
{
	unsigned int clock_m, clock_d;

	fb_get_clock_md(10*(mode->pixel_clock), &clock_m, &clock_d);

	unsigned int hdmi_out0_enabled;
	if (hdmi_out0_core_initiator_enable_read()) {
		hdmi_out0_enabled = 1;
		hdmi_out0_core_initiator_enable_write(0);
	}
	hdmi_out0_core_initiator_hres_write(mode->h_active);
	hdmi_out0_core_initiator_hsync_start_write(mode->h_active + mode->h_sync_offset);
	hdmi_out0_core_initiator_hsync_end_write(mode->h_active + mode->h_sync_offset + mode->h_sync_width);
	hdmi_out0_core_initiator_hscan_write(mode->h_active + mode->h_blanking);
	hdmi_out0_core_initiator_vres_write(mode->v_active);
	hdmi_out0_core_initiator_vsync_start_write(mode->v_active + mode->v_sync_offset);
	hdmi_out0_core_initiator_vsync_end_write(mode->v_active + mode->v_sync_offset + mode->v_sync_width);
	hdmi_out0_core_initiator_vscan_write(mode->v_active + mode->v_blanking);

	hdmi_out0_core_initiator_length_write(mode->h_active*mode->v_active*2);

	hdmi_out0_core_initiator_enable_write(hdmi_out0_enabled);

	fb_clkgen_write(clock_m, clock_d);
}

void hdmi_out_setup(int mode)
{
	const struct video_timing *m = &video_modes[mode];

	hdmi_out0_core_initiator_base_write(0x42000000);

	hdmi_out0_core_initiator_enable_write(0);
	hdmi_out0_driver_clocking_mmcm_reset_write(1);

       	mmcm_config_for_clock(m->pixel_clock);

	fb_set_mode(m);

	hdmi_out0_driver_clocking_mmcm_reset_write(0);
	hdmi_out0_core_initiator_enable_write(1);
}

int hdmi_in_mode = 0;

void hdmi_in_setup(int mode)
{
	const struct video_timing *m = &video_modes[mode];

	hdmi_in_mode = mode;

	hdmi_in0_edid_hpd_en_write(0);

	hdmi_in0_disable();

	mmcm_config_for_clock(m->pixel_clock);

	hdmi_in0_init_video(m->h_active, m->v_active);

	hdmi_in0_edid_hpd_en_write(1);

	hdmi_in0_service(m->pixel_clock);
}

void hdmi_service(void)
{
	const struct video_timing *m = &video_modes[hdmi_in_mode];
	hdmi_in0_service(m->pixel_clock);
}
