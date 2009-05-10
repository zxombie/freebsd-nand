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

/* TODO: Support 16bit NAND */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/module.h>
#include <sys/bio.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <geom/geom_disk.h>

#include "nandreg.h"
#include "nandvar.h"

static struct nand_device_info nand_chips[] = {
	{
	    NAND_MANF_SAMSUNG, NAND_DEV_SAMSUNG_256MB,
	    64, 2048, 64, 2048, 1,
	    8, 2, 3, 1, "Samsung 256MiB 8bit Nand Flash",
	},
	{
	    NAND_MANF_SAMSUNG, NAND_DEV_SAMSUNG_64MB,
	    16, 512, 32, 4096, 1,
	    8, 1, 3, 0, "Samsung 64MiB 8bit Nand Flash",
	},
	{
	    NAND_MANF_SAMSUNG, NAND_DEV_SAMSUNG_32MB,
	    16, 512, 32, 2048, 1,
	    8, 1, 2, 0, "Samsung 32MiB 8bit Nand Flash",
	},

	{ .ndi_name = NULL, }
};

MALLOC_DECLARE(M_NAND);
MALLOC_DEFINE(M_NAND, "NAND", "Memory for the NAND flash driver");

uma_zone_t nand_device_zone;
unsigned int next_unit = 0;

static int nand_readid(nand_device_t);
static int nand_read_data(nand_device_t, off_t, size_t, uint8_t *);

static d_strategy_t nand_strategy;

#if 0
static inline int
nand_correct_ecc(device_t dev, uint8_t *data, uint8_t *read_ecc,
    uint8_t *calc_ecc)
{
	return NAND_CORRECT_ECC(device_get_parent(dev), data, read_ecc,
	    calc_ecc);
}
#endif

/*
 * Reads the device ID into the softc
 */
static int
nand_readid(nand_device_t ndev)
{
	nand_wait_select(ndev, 1);
	nand_command(ndev, NAND_CMD_READID);
	nand_address(ndev, NAND_READID_MANFID);
	nand_read_8(ndev, &ndev->ndev_manf_id);
	nand_read_8(ndev, &ndev->ndev_dev_id);
	nand_wait_select(ndev, 0);

	return (0);
}

/*
 * Writes the address to read/write to the bus
 */
static inline void
nand_write_address(nand_device_t ndev, off_t page, int with_column)
{
	off_t i;

	if (with_column) {
		/* We always want the start of the page */
		for (i = 0; i < ndev->ndev_column_cycles; i++)
			nand_address(ndev, 0x00);
	}
	/* Write the page address */
	for (i = 0; i < ndev->ndev_row_cycles; i++, page >>= 8)
		nand_address(ndev, page & 0xFF);
}

/*
 * Reads the data including spare if len is large enough from the NAND flash
 */
static int
nand_read_data(nand_device_t ndev, off_t page, size_t len, uint8_t *data)
{
	int err = 0;

	nand_command(ndev, NAND_CMD_READ);
	nand_write_address(ndev, page, 1);

	/* XXX: ONFI 1.0 says we need this but some Samsung parts don't */
	if (ndev->ndev_read_start)
		nand_command(ndev, NAND_CMD_READ_START);

	/* Wait for data to be read */
	nand_wait_rnb(ndev);

	nand_read(ndev, len, data);

#if 0
	if (ndev->ecc_len > 0) {
		/* Correct if there was a bit error */
		nand_calc_ecc(sc->dev, sc->calc_ecc);
		nand_read(dev, sc->spare_size, sc->spare_tmp);
		err = nand_correct_ecc(sc->dev, data, sc->read_ecc, sc->calc_ecc);
	}
#endif

	return (err);
}

static int
nand_write_data(nand_device_t ndev, off_t page, size_t len, uint8_t *data)
{
	nand_command(ndev, NAND_CMD_PROGRAM);
	nand_write_address(ndev, page, 1);
	nand_write(ndev, len, data);
	nand_command(ndev, NAND_CMD_PROGRAM_END);
	nand_wait_rnb(ndev);

	/* TODO: Check SR[0] */

	return (0);
}

static void
nand_erase_data(nand_device_t ndev, off_t block)
{
	nand_command(ndev, NAND_CMD_ERASE);
	nand_write_address(ndev, block, 0);
	nand_command(ndev, NAND_CMD_PROGRAM_END);
	nand_wait_rnb(ndev);
}

static void
nand_strategy(struct bio *bp)
{
	uint32_t block_size;
	nand_device_t ndev;
	off_t block, page;
	uint8_t *data;
	int cnt, err;

	ndev = bp->bio_disk->d_drv1;

	bp->bio_resid = bp->bio_bcount;
	switch(bp->bio_cmd) {
	case BIO_READ:
	case BIO_WRITE:
		page = bp->bio_offset / ndev->ndev_page_size;
		cnt = bp->bio_bcount / ndev->ndev_page_size;
		data = bp->bio_data;

		nand_wait_select(ndev, 1);
		while (cnt > 0) {
			if (bp->bio_cmd == BIO_READ)
				err = nand_read_data(ndev, page,
				    ndev->ndev_page_size, data);
			else
				err = nand_write_data(ndev, page,
				    ndev->ndev_page_size, data);

			if (err != 0) {
				bp->bio_error = err;
				bp->bio_flags |= BIO_ERROR;
				break;
			}

			bp->bio_resid -= ndev->ndev_page_size;
			data += ndev->ndev_page_size;
			page++;
			cnt--;
		}
		nand_wait_select(ndev, 0);
		break;

	case BIO_DELETE:
		block_size = ndev->ndev_page_cnt * ndev->ndev_page_size;
		block = bp->bio_offset / block_size;
		cnt = bp->bio_bcount / block_size;

		/*
		 * Deletes must be on a block boundry
		 * and be the size of a block
		 */
		if (((block % block_size) != 0) ||
		    ((bp->bio_bcount % block_size) != 0)) {
			bp->bio_error = ENOTSUP;
			bp->bio_flags |= BIO_ERROR;
			break;
		}

		nand_wait_select(ndev, 1);
		while (cnt > 0) {
			nand_erase_data(ndev, block);
			bp->bio_resid -= block_size;
			block++;
			cnt--;
		}
		nand_wait_select(ndev, 0);
		break;

	default:
		bp->bio_error = ENOTSUP;
		bp->bio_flags |= BIO_ERROR;
		break;
	}

	biodone(bp);
}

int
nand_probe(nand_device_t ndev)
{
	int err, i;

	if (ndev->ndev_driver->ndri_command == NULL ||
	    ndev->ndev_driver->ndri_address == NULL ||
	    ndev->ndev_driver->ndri_read == NULL ||
	    ndev->ndev_driver->ndri_read_8 == NULL ||
	    ndev->ndev_driver->ndri_write == NULL)
		return (EDOOFUS);

	/* Set to an 8 bit bus until we know the correct size */
	ndev->ndev_cell_size = 8;

	err = nand_command(ndev, NAND_CMD_RESET);
	nand_wait_rnb(ndev);
	if (err != 0)
		return (EIO);

	/* Find which part we have */
	err = nand_readid(ndev);
	if (err != 0)
		return (EIO);

	/* Find if we know about this part */
	for (i = 0; nand_chips[i].ndi_name != NULL; i++)
		if (nand_chips[i].ndi_manf_id == ndev->ndev_manf_id &&
		    nand_chips[i].ndi_dev_id == ndev->ndev_dev_id) {
			memcpy(&ndev->ndev_info, &nand_chips[i],
			    sizeof(struct nand_device_info));
			break;
		}

	if (nand_chips[i].ndi_name == NULL) {
		printf("nand: manufacturer 0x%x device 0x%x is not supported\n",
		    ndev->ndev_manf_id, ndev->ndev_dev_id);
		return (ENODEV);
	}

	return (0);
}

int
nand_attach(nand_device_t ndev)
{
	int err;

	err = nand_command(ndev, NAND_CMD_RESET);
	nand_wait_rnb(ndev);
	if (err != 0)
		goto out;

	//ndev->spare_tmp = malloc(ndev->spare_size, M_NAND, M_WAITOK);

	ndev->ndev_disk = disk_alloc();
	ndev->ndev_disk->d_name = "nand";
	ndev->ndev_disk->d_unit = next_unit++;
	ndev->ndev_disk->d_flags = DISKFLAG_CANDELETE;

	ndev->ndev_disk->d_strategy = nand_strategy;

	ndev->ndev_disk->d_sectorsize = ndev->ndev_page_size;
	/* Limit to 1 block */
	ndev->ndev_disk->d_maxsize = ndev->ndev_page_size * ndev->ndev_page_cnt;

	/* We ignore the spare as it is out of band data */
	ndev->ndev_disk->d_mediasize = ndev->ndev_lun_cnt *
	    ndev->ndev_block_cnt * ndev->ndev_page_cnt * ndev->ndev_page_size;

	ndev->ndev_disk->d_drv1 = ndev;
	disk_create(ndev->ndev_disk, DISK_VERSION);

out:
	if (err != 0) {
		nand_detach(ndev);
	}
	return (err);
}

int
nand_detach(nand_device_t ndev)
{
	/* TODO */
	if (ndev->ndev_disk != NULL) {
		disk_destroy(ndev->ndev_disk);
		ndev->ndev_disk = NULL;
	}

	//free(ndev->spare_tmp, M_NAND);
	//ndev->spare_tmp = NULL;

	return (0);
}

static int
nand_load(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		nand_device_zone = uma_zcreate("nand_device",
		    sizeof(struct nand_device), NULL, NULL, NULL, NULL, 0, 0);
		return (0);
	case MOD_UNLOAD:
		uma_zdestroy(nand_device_zone);
		return (0);
	default:
		return (ENOTSUP);
	}
}

DEV_MODULE(nand, nand_load, NULL);
MODULE_VERSION(nand, 1);

