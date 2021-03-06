/*
 *	include/linux/cl2n_board_version.h
 */

#ifndef _LINUX_CL2N_BOARD_VERSION_H
#define _LINUX_CL2N_BOARD_VERSION_H

// arch/arm/mach-tegra/board.h
extern int cl2n_get_board_strap(void);

enum cl2n_board_version
{
	CL2N_BOARD_VER_UNKNOW = 0,
	CL2N_BOARD_VER_A00    = 0x7,
	CL2N_BOARD_VER_B00    = 0x5,
	CL2N_BOARD_VER_C00    = 0x3,
	CL2N_BOARD_VER_D00    = 0x1,
};

#endif
