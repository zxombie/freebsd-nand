/*
 * Copyright (C) 2009 Andrew Turner
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>

#include <geom/geom.h>
#include <geom/geom_disk.h>
#include <geom/geom_vfs.h>

#include "nandreg.h"
#include "nandvar.h"

#define CHECK_STATE()						\
do {								\
	KASSERT(nand_chip.incmd != 0xFF, 			\
	    ("NANDSIM: Command state was not cleared"));	\
	KASSERT(nand_chip.inaddr != 0xFF,			\
	    ("NANDSIM: Address state was not cleared"));	\
	KASSERT(nand_chip.inread != 0xFF, 			\
	    ("NANSIM: Read state was not cleared"));		\
	KASSERT(nand_chip.inwrite != 0xFF,			\
	    ("NANDSIM: Write state was not cleated"));		\
} while(0)

#define RESET_STATE()			\
do {					\
	nand_chip.startcmd = 1;		\
	nand_chip.incmd = 0;		\
	nand_chip.inaddr = 0;		\
	nand_chip.inread = 0;		\
	nand_chip.inwrite = 0;		\
					\
	nand_chip.cmd_len = 0;		\
	nand_chip.address = 0;		\
	nand_chip.address_len = 0;	\
} while (0)

#define CLEAR_IN_STATE()		\
do {					\
	nand_chip.incmd = 0xFF;		\
	nand_chip.inaddr = 0xFF;	\
	nand_chip.inread = 0xFF;	\
	nand_chip.inwrite = 0xFF;	\
} while (0)

#define GET_OFFSET(dest_addr, len)			\
do {							\
	dest_addr = nand_chip.address +			\
	    ((nand_chip.address / 256) * 16);		\
							\
	if (dest_addr + len > nand_chip.size) {	\
		printf("NANDSIM: %s: Attempt to access past end of data\n", \
		    __func__);				\
		RESET_STATE();				\
		return (EIO);				\
	}						\
} while (0)

static struct {
	int		startcmd;	/* Can we start a new command */

	/* These tell us what to expect next */
	int		incmd;		/* Are we processing a command */
	int		inaddr;		/* Are we able to get the address */
	int		inread;		/* Are we able to read data from NAND*/
	int		inwrite;	/* Are we able to write data to NAND*/

	int		read_start;	/* Expect a NAND_CMD_READ_START */

	int		cmd_len;
	uint8_t		cmd[2];

	int		address_len;
	uint32_t	address;

	size_t		data_len;
	uint8_t		*data;

	uint8_t		manuf;
	uint8_t		device;

	size_t		size;
} nand_chip;

static int nandsim_command(nand_device_t, uint8_t);
static int nandsim_address(nand_device_t, uint8_t);
static int nandsim_read(nand_device_t, size_t, uint8_t *);
static int nandsim_write(nand_device_t, size_t, uint8_t *);

static struct nand_driver nandsim_dri = {
	.ndri_command = nandsim_command,
	.ndri_address = nandsim_address,
	.ndri_read = nandsim_read,
	.ndri_write = nandsim_write,
};

static struct nand_device nandsim_dev = {
	.ndev_driver = &nandsim_dri,
};

MALLOC_DEFINE(M_NANDSIM, "nandsimdisk", "nandsim virtual disk buffers");

/* TODO: Set the in* state correctly before returning from the functions */
static int
nandsim_command(nand_device_t ndev, uint8_t cmd)
{
	/* Some commands may be sent with the LUN in any state */
	if (cmd == NAND_CMD_RESET) {
		printf("NANDSIM: nandsim_command: Reset chip\n");
		RESET_STATE();
		return (0);
	}

	if (nand_chip.startcmd != 0) {
		/*
		 * New command, we have already called RESET_STATE()
		 * as that is the only way to get here
		 */
		nand_chip.startcmd = 0;
		nand_chip.incmd = 1;
	}

	/* Check if we are not able to handle a command */
	if (nand_chip.incmd == 0) {
		printf("NANDSIM: nandsim_command: "
		    "Got a command when we were not expecting it: 0x%X\n", cmd);
		RESET_STATE();
		return (EIO);
	}

	/* Store the commnad */
	switch(nand_chip.cmd_len) {
	case 0:
	case 1:
		nand_chip.cmd[nand_chip.cmd_len] = cmd;
		nand_chip.cmd_len++;
		break;
	default:
		printf("NANDSIM: nandsim_command: "
		    "Attempting to write too many commands\n");
		RESET_STATE();
		return (EIO);
	}

	CLEAR_IN_STATE();

	/* Which command are we in */
	switch (nand_chip.cmd[0]) {
	case NAND_CMD_PROGRAM:
		switch (nand_chip.cmd_len) {
		case 1:
			/* We can send the address now */
			nand_chip.incmd = 0;
			nand_chip.inaddr = 1;
			nand_chip.inread = 0;
			nand_chip.inwrite = 0;
			break;
		case 2:
			/* We have finished the program sysle */
			RESET_STATE();
			if (nand_chip.cmd[1] != NAND_CMD_PROGRAM_END) {
				printf("NANDSIM: nandsim_command: "
				    "Unknown command after NAND_CMD_PROGRAM\n");
				return (EIO);
			}
			break;
		}
		break;

	case NAND_CMD_READ:
		switch (nand_chip.cmd_len) {
		case 1:
			/* We can send the address now */
			nand_chip.incmd = 0;
			nand_chip.inaddr = 1;
			nand_chip.inread = 0;
			nand_chip.inwrite = 0;
			break;
		case 2:
			if (nand_chip.cmd[1] != NAND_CMD_READ_START) {
				printf("NANDSIM: nandsim_command: "
				    "Unknown command after NAND_CMD_READ\n");
				RESET_STATE();
				return (EIO);
			}
			if (!nand_chip.read_start) {
				printf("NANDSIM: nandsim_command: "
				    "Received NAND_CMD_READ_START when we "
				    "didn't expect it\n");
				RESET_STATE();
				return (EIO);
			}
			nand_chip.incmd = 0;
			nand_chip.inaddr = 0;
			nand_chip.inread = 1;
			nand_chip.inwrite = 0;
			break;
		}
		break;

	case NAND_CMD_ERASE:
		switch (nand_chip.cmd_len) {
		case 1:
			/* We can send the address now */
			nand_chip.incmd = 0;
			nand_chip.inaddr = 1;
			nand_chip.inread = 0;
			nand_chip.inwrite = 0;
			break;
		case 2:
			RESET_STATE();
			if (nand_chip.cmd[1] != NAND_CMD_ERASE_END) {
				printf("NANDSIM: nandsim_command: "
				    "Unknown command after NAND_CMD_ERASE\n");
				return (EIO);
			}
			break;
		}
		break;

	case NAND_CMD_READID:
		/* If we are reading the manifest ID we can move to read */
		if (nand_chip.cmd_len > 1) {
			printf("NANDSIM: nandsim_command: "
			    "NAND_CMD_READID only supports 1 command\n");
			RESET_STATE();
			return (EIO);
		}
		nand_chip.incmd = 0;
		nand_chip.inaddr = 1;
		nand_chip.inread = 0;
		nand_chip.inwrite = 0;
		break;

	default:
		printf("NANDSIM: nandsim_command: "
		    "Unknown or unimplemented command\n");
		RESET_STATE();
		return (EIO);
	}

	CHECK_STATE();

	return (0);
}

static int
nandsim_address(nand_device_t ndev, uint8_t address)
{
	if (nand_chip.inaddr == 0) {
		printf("NANDSIM: nandsim_address: "
		    "Got an address when we were not expecting it\n");
		RESET_STATE();
		return (EIO);
	}

	CLEAR_IN_STATE();

	/* The address is too long */
	if (nand_chip.address_len == 4) {
		printf("NANDSIM: nandsim_address: Address too long\n");
		RESET_STATE();
		return (EIO);
	}

	switch (nand_chip.cmd[0]) {
	case NAND_CMD_READID:
		nand_chip.incmd = 0;
		nand_chip.inaddr = 0;
		nand_chip.inread = 1;
		nand_chip.inwrite = 0;
		break;

	case NAND_CMD_READ:
		nand_chip.incmd = 0;
		nand_chip.inaddr = 1;
		nand_chip.inread = 1;
		nand_chip.inwrite = 0;
		break;

	case NAND_CMD_PROGRAM:
		/* We can enter the end command */
		nand_chip.incmd = 1;
		nand_chip.inaddr = 1;
		nand_chip.inread = 0;
		nand_chip.inwrite = 1;
		break;

	default:
		printf("NANDSIM: nandsim_address: "
		    "Invalid command when writing the address\n");
		RESET_STATE();
		return (EIO);
	}

	nand_chip.address |= address << (8 * nand_chip.address_len);
	nand_chip.address_len++;

	CHECK_STATE();

	return (0);
}

static int
nandsim_read(nand_device_t ndev, size_t len, uint8_t *data)
{
	off_t real_address;
	int i;

	if (nand_chip.inread == 0) {
		printf("NANDSIM: nandsim_read: "
		    "Attempting to read when we can't read\n");
		RESET_STATE();
		return (EIO);
	}

	CLEAR_IN_STATE();

	switch(nand_chip.cmd[0]) {
	case NAND_CMD_READID:
		switch(nand_chip.address) {
		case NAND_READID_MANFID:
			/* Read the Manufacturer ID */
			if (len > 0) {
				data[0] = nand_chip.manuf;
				if (len > 1)
					data[1] = nand_chip.device;

				printf("NANDSIM: nandsim_read: Read chip ID (");
				for (i = 0; i < len; i++)
					printf("%.2X ", data[i]);
				printf(")\n");
			} else {
				printf("NANDSIM: nandsim_read: "
				    "Read chip ID length too short\n");
				RESET_STATE();
				return (EIO);
			}
			break;

		default:
			printf("NANDSIM: nandsim_read: "
			    "Unknown or unimplemented address %X "
			    "after NAND_READID_MANFID\n", nand_chip.address);
			RESET_STATE();
			return (EIO);
		}
		/* TODO: Allow this to be read with 2 calls to read */
		RESET_STATE();
		break;

	case NAND_CMD_READ:
		switch(ndev->ndev_cell_size) {
		case 8:
		case 16:
			printf("NANDSIM: nandsim_read: Read page %X\n",
			    nand_chip.address);

			/* Copy the data to NAND */
			GET_OFFSET(real_address, len);
			printf("NANDSIM: nandsim_read: Reading offset %X\n",
			    (unsigned int)real_address);

			/* The length is in terms of ndev->ndev_width bits */
			len = len * ndev->ndev_cell_size / 8;
			memcpy(data, &nand_chip.data[real_address], len);
			break;
		default:
			printf("NANDSIM: nandsim_read: Unknown bus width %d\n",
			    ndev->ndev_cell_size);
			RESET_STATE();
			return (EIO);
		}

		RESET_STATE();
		break;

	default:
		printf("NANDSIM: nandsim_read: Unknown command:");
		for (i = 0; i < nand_chip.cmd_len; i++)
			printf(" %.2X", nand_chip.cmd[i]);
		printf("\n");
		RESET_STATE();
		return (EIO);
	}

	CHECK_STATE();

	return (0);
}

static int
nandsim_write(nand_device_t ndev, size_t len, uint8_t *data)
{
	off_t real_address;
	int i;

	CLEAR_IN_STATE();

	if (nand_chip.inwrite == 0) {
		printf("NANDSIM: nandsim_read: "
		    "Attempting to write when we can't write\n");
		RESET_STATE();
		return (EIO);
	}

	switch (nand_chip.cmd[0]) {
	case NAND_CMD_PROGRAM:
		switch(ndev->ndev_cell_size) {
		case 8:
		case 16:
			GET_OFFSET(real_address, len);
			printf("NANDSIM: nandsim_write: "
			    "Programming offset %X\n",
			    (unsigned int)real_address);

			/*
			 * The length is in terms of ndev->ndev_width bits.
			 * Adjust the length to be in terms of 8 bits.
			 */
			len = len * ndev->ndev_cell_size / 8;

			/*
			 * Iterate through each byte anding in the new
			 * data to ensure we only move from 1 -> 0
			 */
			for (i = 0; i < len; i++) {
				nand_chip.data[real_address + i] &= data[i];
			}
			RESET_STATE();
			break;
		default:
			printf("NANDSIM: nandsim_write: "
			    "Write of unknown bus width %d\n",
			    ndev->ndev_cell_size);
			RESET_STATE();
			return (EIO);
		}
		break;

	default:
		printf("NANDSIM: nandsim_write: Unknown command:");
		for (i = 0; i < nand_chip.cmd_len; i++)
			printf(" %.2X", nand_chip.cmd[i]);
		printf("\n");
		RESET_STATE();
		return (EIO);
	}

	CHECK_STATE();

	return (0);
}

static int
nandsim_probe(void)
{
	return nand_probe(&nandsim_dev);
}

static int
nandsim_attach(void)
{
	return nand_attach(&nandsim_dev);
}

static int
nandsim_detach(void)
{
	return nand_detach(&nandsim_dev);
}

static int
nandsim_load(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		/* Samsung 64MiB chip, eg. K9F1208U0B */
		nand_chip.manuf = 0xEC;
		nand_chip.device = 0x76;
		nand_chip.read_start = 0;

		nand_chip.size = (512 + 16) * 32 * 4096;
		nand_chip.data = malloc(nand_chip.size, M_NANDSIM, M_WAITOK);
		/* Erase the NAND chip */
		memset(nand_chip.data, 0xFF, nand_chip.size);

		if (nandsim_probe() != 0) {
			printf("nandsim: Error in nandsim_probe()\n");
			return (ENXIO);
		}
		if (nandsim_attach() != 0) {
			printf("nandsim: Error in nandsim_attach()\n");
			return (ENXIO);
		}
		return (0);

	case MOD_UNLOAD:
		nandsim_detach();
		free(nand_chip.data, M_NANDSIM);
		return (0);

	default:
		return (ENOTSUP);
	}
}

DEV_MODULE(nandsim, nandsim_load, NULL);
MODULE_DEPEND(nandsim, nand, 1, 1, 1);

