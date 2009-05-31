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

#ifndef DEV_NAND_NANDVAR_H
#define DEV_NAND_NANDVAR_H

#include <vm/uma.h>

struct nand_driver;
struct nand_device;

typedef struct nand_driver* nand_driver_t;
typedef struct nand_device* nand_device_t;

/*
 * Used to hold callbacks to the NAND controller.
 * Not all functions need to be implemented.
 * (R) Reqired
 * (O) Optional
 */
struct nand_driver {
	int (*ndri_select)(nand_device_t, int);			/* (O) */
	int (*ndri_command)(nand_device_t, uint8_t);		/* (R) */
	int (*ndri_address)(nand_device_t, uint8_t);		/* (R) */
	int (*ndri_read)(nand_device_t, size_t, uint8_t *);	/* (R) */
	int (*ndri_read_8)(nand_device_t, uint8_t *);		/* (R) */
	int (*ndri_write)(nand_device_t, size_t, uint8_t *);	/* (R) */
	int (*ndri_read_rnb)(nand_device_t);			/* (O) */
	int (*ndri_calc_ecc)(nand_device_t, uint8_t *);		/* (O) */
};

struct nand_device_info {
	uint8_t		ndi_manf_id;
	uint8_t		ndi_dev_id;	/* Device ID */

	uint16_t	ndi_spare_size;	/* Spare bytes per page */
	uint32_t	ndi_page_size;	/* Bytes per page (not spare) */
	uint32_t	ndi_page_cnt;	/* Pages per block */
	uint32_t	ndi_block_cnt;	/* Total blocks per LUN */
	uint8_t		ndi_lun_cnt;	/* Total number of LUNs */

	uint8_t		ndi_cell_size;	/* Bits per cell */
	uint8_t		ndi_column_cycles; /* Column address cycle count */
	uint8_t		ndi_row_cycles;	/* Row address cycle count */

	char		ndi_read_start;	/* Do we need to issue a read start */
	const char	*ndi_name;	/* The name of the device */
};

#define NAND_ECC_MAX 16
struct nand_device {
	/* Set by the NAND controller */
	nand_driver_t	ndev_driver;

	/* Device info. Set by the NAND device */
	struct nand_device_info ndev_info;
#define ndev_manf_id	ndev_info.ndi_manf_id
#define ndev_dev_id	ndev_info.ndi_dev_id
#define ndev_spare_size	ndev_info.ndi_spare_size
#define ndev_page_size	ndev_info.ndi_page_size
#define ndev_page_cnt	ndev_info.ndi_page_cnt
#define ndev_block_cnt	ndev_info.ndi_block_cnt
#define ndev_lun_cnt	ndev_info.ndi_lun_cnt
#define ndev_cell_size	ndev_info.ndi_cell_size
#define ndev_column_cycles ndev_info.ndi_column_cycles
#define ndev_row_cycles	ndev_info.ndi_row_cycles
#define ndev_read_start	ndev_info.ndi_read_start
#define ndev_name	ndev_info.ndi_name

	device_t	ndev_dev;
	struct disk	*ndev_disk;

	//uint8_t		*spare_tmp;	/* Used for the spare data on read */
};

extern uma_zone_t nand_device_zone;
#define nand_alloc_device(ndev, driver) \
do { \
	ndev = uma_zalloc(nand_device_zone, M_ZERO); \
	ndev->ndev_driver = driver; \
} while (0)

#define nand_free_device(ndev) uma_zfree(ata_request_zone, ndev)

#define nand_wait_select(ndev, enable)				\
do {								\
	if (ndev->ndev_driver->ndri_select != NULL)		\
		ndev->ndev_driver->ndri_select(ndev, enable);	\
} while (0)
#define nand_command(ndev, data) ndev->ndev_driver->ndri_command(ndev, data)
#define nand_address(ndev, data) ndev->ndev_driver->ndri_address(ndev, data)
#define nand_read(ndev, len, data) \
    ndev->ndev_driver->ndri_read(ndev, len, data)
#define nand_read_8(ndev, data) ndev->ndev_driver->ndri_read_8(ndev, data)
#define nand_write(ndev, len, data) \
    ndev->ndev_driver->ndri_write(ndev, len, data)
#define nand_read_rnb(ndev) ndev->ndev_driver->ndri_read_rnb(ndev)
#define nand_wait_rnb(ndev)				\
do {							\
	if (ndev->ndev_driver->ndri_read_rnb != NULL) {	\
		int rnb;				\
		rnb = nand_read_rnb(ndev);		\
		while (rnb == 0) {			\
			DELAY(100);			\
			rnb = nand_read_rnb(ndev);	\
		}					\
	}						\
} while (0)
#define nand_calc_ecc(ndev, data) ndev->ndev_driver->ndri_calc_ecc(ndev, data)

int nand_probe(nand_device_t);
int nand_attach(nand_device_t);
int nand_detach(nand_device_t);

#endif

