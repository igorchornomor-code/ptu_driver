include ../config.mk

LDFLAGS+=-L..
CFLAGS+=-I.. -I../cerial
LIBS:=-lcpi $(LIBS) -lm
BINS=$(basename $(filter-out estrap.c, $(wildcard *.c)))
ifeq ($(SYS),WIN32)
    BINEXES=$(addsuffix .exe, $(BINS))
else
    BINEXES=$(BINS)
endif

all: $(BINS)

$(BINS): estrap.o ../libcpi.a
% : %.c ;
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $@.c estrap.o $(LIBS)


.PHONY: clean

clean:
	-rm -f $(BINEXES) *.o
