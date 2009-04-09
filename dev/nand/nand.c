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

#include <vm/uma.h>

#include <geom/geom_disk.h>

#include "nandreg.h"
#include "nandvar.h"

static struct {
	uint8_t		manf_id;
	uint8_t		dev_id;

	uint16_t	spare_size;	/* Spare bytes prt page */
	uint32_t	page_size;	/* Bytes per page (not spare) */
	uint32_t	page_cnt;	/* Pages per block */
	uint32_t	block_cnt;	/* Total blocks */

	unsigned int	bus_width;	/* Number of bits in the bus */

	char		read_start;	/* Do we need to issue a read start? */
	const char	*name;
} nand_chips[] = {
	{ 0xEC, 0x76, 16, 512, 32, 4096, 8, 0, "Samsung 64MB Nand Flash" },

	{ 0x00, 0x00, 0, 0, 0, 0, 0, 0, NULL },
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
	int err;
	uint8_t data[2];

	err = nand_command(ndev, NAND_CMD_READID);
	err += nand_address(ndev, NAND_READID_MANFID);
	err += nand_read(ndev, 2, data);

	if (err != 0)
		return (EIO);

	ndev->manf_id = data[0];
	ndev->dev_id = data[1];

	printf("NAND: Read ID found chip %X %X\n", ndev->manf_id, ndev->dev_id);

	return 0;
}

/*
 * Reads the data including spare if len is large enough from the NAND flash
 */
static int
nand_read_data(nand_device_t ndev, off_t page, size_t len, uint8_t *data)
{
	int err = 0;

	nand_command(ndev, NAND_CMD_READ);

	/* This will only work with 512 byte pages */
	nand_address(ndev, 0x00);
	nand_address(ndev, (page >> 0) & 0xFF);
	nand_address(ndev, (page >> 8) & 0xFF);

	/* XXX: ONFI 1.0 says we need this but some Samsung parts don't */
	if (ndev->read_start)
		nand_command(ndev, NAND_CMD_READ_START);

	/* Wait for data to be read */
	DELAY(20);

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
	int err = 0;

	nand_command(ndev, NAND_CMD_PROGRAM);

	/* This will only work with 512 byte pages */
	nand_address(ndev, 0x00);
	nand_address(ndev, (page >> 0) & 0xFF);
	nand_address(ndev, (page >> 8) & 0xFF);

	DELAY(20);

	nand_write(ndev, len, data);

	nand_command(ndev, NAND_CMD_PROGRAM_END);

	return (err);
}

static void
nand_erase_data(nand_device_t ndev, off_t block)
{
	nand_command(ndev, NAND_CMD_ERASE);

	nand_address(ndev, (block >> 0) & 0xFF);
	nand_address(ndev, (block >> 8) & 0xFF);

	nand_command(ndev, NAND_CMD_PROGRAM_END);

	DELAY(20);
}

static void
nand_strategy(struct bio *bp)
{
	nand_device_t ndev;
	off_t block, page;
	uint8_t *data;
	int cnt, err;

	ndev = bp->bio_disk->d_drv1;

	bp->bio_resid = bp->bio_bcount;
	switch(bp->bio_cmd) {
	case BIO_READ:
		page = bp->bio_offset / ndev->page_size;
		cnt = bp->bio_bcount / ndev->page_size;
		data = bp->bio_data;

		printf("NAND: Reading %d (0x%X) pages starting at page"
		    " %d (0x%X)\n", cnt, cnt, (int)page, (int)page);

		while (cnt > 0) {
			err = nand_read_data(ndev, page, ndev->page_size, data);
			if (err != 0) {
				bp->bio_error = err;
				bp->bio_flags |= BIO_ERROR;
				break;
			}

			bp->bio_resid -= ndev->page_size;
			data += ndev->page_size;
			page++;
			cnt--;
		}
		break;

	case BIO_WRITE:
		page = bp->bio_offset / ndev->page_size;
		cnt = bp->bio_bcount / ndev->page_size;
		data = bp->bio_data;

		printf("NAND: Writing %d (0x%X) pages starting at page"
		    " %d (0x%X)\n", cnt, cnt, (int)page, (int)page);

		while (cnt > 0) {
			err = nand_write_data(ndev, page, ndev->page_size,data);
			if (err != 0) {
				bp->bio_error = err;
				bp->bio_flags |= BIO_ERROR;
				break;
			}

			bp->bio_resid -= ndev->page_size;
			data += ndev->page_size;
			page++;
			cnt--;
		}
		break;

	case BIO_DELETE:
		block = bp->bio_offset / ndev->block_size;
		cnt = bp->bio_bcount / ndev->block_size;

		/*
		 * Deletes must be on a block boundry
		 * and be the size of a block
		 */
		if (((block % ndev->block_size) != 0) ||
		    ((bp->bio_bcount % ndev->block_size) != 0)) {
			bp->bio_error = ENOTSUP;
			bp->bio_flags |= BIO_ERROR;
			break;
		}

		while (cnt > 0) {
			nand_erase_data(ndev, block);
			bp->bio_resid -= ndev->block_size;
			block++;
			cnt--;
		}
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
	if (ndev->ndev_driver->ndri_command == NULL ||
	    ndev->ndev_driver->ndri_address == NULL ||
	    ndev->ndev_driver->ndri_read == NULL ||
	    ndev->ndev_driver->ndri_write == NULL)
		return (EDOOFUS);

	return (0);
}

int
nand_attach(nand_device_t ndev)
{
	int err, i;

	printf("NAND: Reseting chip\n");
	err = nand_command(ndev, NAND_CMD_RESET);
	if (err != 0)
		goto out;

	/* Find which part we have */
	/* TODO: Move this to probe? */
	printf("NAND: Reading NAND ID\n");
	err = nand_readid(ndev);
	if (err != 0)
		goto out;
	for (i = 0; nand_chips[i].name != NULL; i++) {
		if (nand_chips[i].manf_id == ndev->manf_id &&
		    nand_chips[i].dev_id == ndev->dev_id) {
			ndev->spare_size = nand_chips[i].spare_size;
			ndev->page_size = nand_chips[i].page_size;
			ndev->page_cnt = nand_chips[i].page_cnt;
			ndev->block_size = ndev->page_size * ndev->page_cnt;
			ndev->block_cnt = nand_chips[i].block_cnt;
			ndev->ndev_width = nand_chips[i].bus_width;
			ndev->read_start = nand_chips[i].read_start;
			break;
		}
	}
	if (nand_chips[i].name == NULL) {
		printf("NAND: manufacturer 0x%x device 0x%x is not supported\n",
		    ndev->manf_id, ndev->dev_id);
		err = ENODEV;

		goto out;
	}

	ndev->spare_tmp = malloc(ndev->spare_size, M_NAND, M_WAITOK);

	ndev->disk = disk_alloc();
	ndev->disk->d_name = "nand";
	ndev->disk->d_unit = next_unit++;

	ndev->disk->d_strategy = nand_strategy;

	ndev->disk->d_sectorsize = ndev->page_size;
	ndev->disk->d_maxsize = ndev->block_size;

	/* We ignore the spare as it is or out of band data */
	ndev->disk->d_mediasize = ndev->block_cnt * ndev->block_size;

	ndev->disk->d_drv1 = ndev;
	disk_create(ndev->disk, DISK_VERSION);

out:
	if (err != 0) {
		free(ndev->spare_tmp, M_NAND);
		ndev->spare_tmp = NULL;
	}
	return (err);
}

int
nand_detach(nand_device_t ndev)
{
	/* TODO */
	if (ndev->disk != NULL)
		disk_destroy(ndev->disk);

	free(ndev->spare_tmp, M_NAND);
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

