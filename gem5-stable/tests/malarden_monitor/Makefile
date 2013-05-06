ifndef MONITOR
MONITOR=UMC_HWFILTER
endif
ifndef MODEL
MODEL=ATOMIC
endif
CCFLAGS=-D$(MONITOR) -D$(MODEL) -O2
INCLUDES=$(wildcard include/*.c)

malarden_wcet.obj: malarden_wcet.arm
	arm-linux-gnueabi-objdump -dS malarden_wcet.arm > malarden_wcet.obj

malarden_wcet.arm: malarden_wcet.c ${INCLUDES} include/malarden.h
	arm-linux-gnueabi-gcc -DUNIX ${CCFLAGS} ${INCLUDES} malarden_wcet.c -o malarden_wcet.arm --static
	
clean:
	rm -f *.arm *.obj

