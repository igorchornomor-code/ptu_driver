include config.mk

CFLAGS+=-Icerial
LDFLAGS+=-Lcerial
HERE=$(shell basename `pwd`)
BINS=opgen
ifeq ($(SYS),WIN32)
    BINEXES=$(addsuffix .exe, $(BINS))
else
    BINEXES=$(BINS)
endif

all: cerial libcpi.a $(BINS) examples

release: CFLAGS_BUILD=-O2 -static
release: clean all strip package

# dependencies
cpi.o: cpi.c cpi.h cerial/cerial.h opcodes.h openum.h cfrm.h
cfrm.o: cfrm.c cfrm.h crc_cfrm.h
crc_cfrm.o: crc_cfrm.c crc_cfrm.h
optable.o: optable.c opcodes.h openum.h

# library
libcpi.a: cerial/libcerial.a cpi.o cfrm.o crc_cfrm.o optable.o
	cp cerial/libcerial.a libcpi.a
	ar ru $@ $(filter %.o, $^)
	ranlib $@

# programs made should link to libcpi
$(BINS) : libcpi.a
% : %.o ;
	$(CC) $(CFLAGS) $(LDFLAGS) -L. -o $@ $(filter %.o, $^) $(LIBS) -lcpi

# code generator
opgen: opgen.o

strip:
	strip $(BINEXES)

.PHONY: cerial examples doc clean tags

examples cerial:
	$(MAKE) -C $@

doc: doc/html doc/README.html doxygen
doc/html:
	mkdir doc/html
doc/README.html: README
	markdown README > doc/html/README.html
doxygen:
	doxygen doc/doxygen.conf 1>/dev/null 2>&1

clean:
	-rm -f $(DEPS) $(BINEXES) libcpi.a *.o *.d *~
	$(MAKE) -C cerial clean
	$(MAKE) -C examples clean

tags:
	ctags -R --excmd=pattern .

