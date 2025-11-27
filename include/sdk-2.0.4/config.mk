# system detection
KERNEL=$(subst _%,,$(shell uname -s | sed s/_.*//))
ifeq ($(KERNEL),MINGW32)
    # MinGW
    SYS = WIN32
else
    SYS = POSIX
endif

# common variables (GNU toolchain)
CC=gcc
AR=ar
CFLAGS=-g -Wall -Werror -DLITTLE_ENDIAN
LDFLAGS=
LIBS=

# system dependant additions
ifeq ($(SYS), WIN32)
    # Windows build requires:
    #     Windows XP SP0 or newer
    #     WinSock 2
    CFLAGS+=-DWINVER=0x0501
    LIBS+=-lws2_32
endif

