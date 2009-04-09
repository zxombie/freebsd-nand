# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/nand

KMOD=	nandsim
SRCS=	nandsim.c nandreg.h nandvar.h

CFLAGS+= -DINVARIANTS

.include <bsd.kmod.mk>

