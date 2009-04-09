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

struct nand_driver;
struct nand_device;

typedef struct nand_driver* nand_driver_t;
typedef struct nand_device* nand_device_t;

struct nand_driver {
	int (*ndri_command)(nand_device_t, uint8_t);
	int (*ndri_address)(nand_device_t, uint8_t);
	int (*ndri_read)(nand_device_t, size_t, uint8_t *);
	int (*ndri_write)(nand_device_t, size_t, uint8_t *);
	int (*ndri_calc_ecc)(nand_device_t, uint8_t *);
};

#define NAND_ECC_MAX 16
struct nand_device {
	/* Set by the NAND controller */
	nand_driver_t	ndev_driver;

	unsigned int	ndev_width;	/* Bus width in bits, eg. 8 or 16bits */

	/* Used by the NAND driver */
	uint8_t		read_ecc[NAND_ECC_MAX];
	uint8_t		calc_ecc[NAND_ECC_MAX];

	uint8_t		manf_id;
	uint8_t		dev_id;
	uint16_t	spare_size;
	uint32_t	page_size;
	uint32_t	page_cnt;
	uint32_t	block_size;
	uint32_t	block_cnt;

	/* Should we issue a NAND_CMD_READ_START before reading? */
	char		read_start;

	struct disk	*disk;

	uint8_t		*spare_tmp;	/* Used for the spare data on read */
};

extern uma_zone_t nand_device_zone;
#define nand_alloc_device(ndev, driver) \
do { \
	ndev = uma_zalloc(nand_device_zone, M_ZERO); \
	ndev->ndev_driver = driver; \
} while (0)

#define nand_free_device(ndev) uma_zfree(ata_request_zone, ndev)

#define nand_command(ndev, data) ndev->ndev_driver->ndri_command(ndev, data)
#define nand_address(ndev, data) ndev->ndev_driver->ndri_address(ndev, data)
#define nand_read(ndev, len, data) \
    ndev->ndev_driver->ndri_read(ndev, len, data)
#define nand_write(ndev, len, data) \
    ndev->ndev_driver->ndri_write(ndev, len, data)
#define nand_calc_ecc(ndev, data) ndev->ndev_driver->ndri_calc_ecc(ndev, data)

int nand_probe(nand_device_t);
int nand_attach(nand_device_t);
int nand_detach(nand_device_t);

#endif

