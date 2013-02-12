TARGET=
CCFLAGS=-DUMC -O2

${TARGET}: ${TARGET}.c
	arm-linux-gnueabi-gcc -DUNIX ${CCFLAGS} ${TARGET}.c -o ${TARGET}.arm --static
	arm-linux-gnueabi-objdump -dS ${TARGET}.arm > ${TARGET}.obj

malarden_wcet: malarden_wcet.c include/malarden.c
	arm-linux-gnueabi-gcc -DUNIX ${CCFLAGS} malarden_wcet.c include/malarden.c -o malarden_wcet.arm --static
	arm-linux-gnueabi-objdump -dS malarden_wcet.arm > malarden_wcet.obj

clean:
	rm -f *.arm *.obj

