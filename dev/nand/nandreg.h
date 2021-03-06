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

#ifndef DEV_NAND_NANDREG_H
#define DEV_NAND_NANDREG_H

#define NAND_CMD_READ		0x00
#define  NAND_CMD_READ_START	0x30

#define NAND_CMD_ERASE		0x60
#define  NAND_CMD_ERASE_END	0xD0

#define NAND_CMD_READ_STATUS	0x70
#define  NAND_STATUS_FAIL	(1<<0)
#define  NAND_STATUS_FAILC	(1<<1)
#define  NAND_STATUS_ARDY	(1<<5)
#define  NAND_STATUS_RDY	(1<<6)
#define  NAND_STATUS_WP		(1<<7)

#define NAND_CMD_PROGRAM	0x80
#define  NAND_CMD_PROGRAM_END	0x10

#define NAND_CMD_READID		0x90
#define  NAND_READID_MANFID	0x00
#define  NAND_READID_NANDID	0x20

#define NAND_CMD_RESET	0xFF

/* Device identification */
#define NAND_MANF_SAMSUNG	0xEC
#define  NAND_DEV_SAMSUNG_256MB	0xAA /* 256MiB 8bit 1.8v */
#define  NAND_DEV_SAMSUNG_64MB	0x76 /* 64MiB 8bit 3.3v */
#define  NAND_DEV_SAMSUNG_32MB	0x75 /* 32MiB 8bit 3.3v */

#endif

