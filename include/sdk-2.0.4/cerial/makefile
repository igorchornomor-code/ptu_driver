include ../config.mk

COMMON_FILES=cerial.c cerstdio.c tcpbsd.c
POSIX_FILES=serposix.c
WIN32_FILES=serwin32.c

OBJECT_FILES=$(patsubst %.c, %.o, $(COMMON_FILES) $($(SYS)_FILES))

all: libcerial.a($(OBJECT_FILES))

$(OBJECT_FILES): cerial.h

.PHONY: clean force

clean:
	-rm -f *.o *.d *.a *~

force: ;

