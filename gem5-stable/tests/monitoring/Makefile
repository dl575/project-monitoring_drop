ifndef MONITOR
MONITOR=UMC_HWFILTER
endif
ifndef MODEL
MODEL=ATOMIC
endif
TARGET=
CCFLAGS=-O2 -D$(MONITOR) -D$(MODEL)

${TARGET}: ${TARGET}.c
	arm-linux-gnueabi-gcc -DUNIX ${CCFLAGS} ${TARGET}.c -o ${TARGET}.arm --static
	# arm-linux-gnueabi-objdump -dS ${TARGET}.arm > ${TARGET}.obj

clean:
	rm -f *.arm *.obj

