TARGET=
CCFLAGS=

${TARGET}: ${TARGET}.c
	arm-linux-gnueabi-gcc -DUNIX ${CCFLAGS} ${TARGET}.c -o ${TARGET}.arm --static
	arm-linux-gnueabi-objdump -dS ${TARGET}.arm > ${TARGET}.obj

malarden_wcet: malarden_wcet.c include/malarden.c
	arm-linux-gnueabi-gcc -DUNIX malarden_wcet.c include/malarden.c -o malarden_wcet.arm --static

clean:
	rm -f *.arm *.obj

