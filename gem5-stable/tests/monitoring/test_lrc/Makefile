TARGET=
CCFLAGS=-O2 -DLRC -I ../

${TARGET}: ${TARGET}.c
	arm-linux-gnueabi-gcc -DUNIX ${CCFLAGS} ${TARGET}.c -o ${TARGET}.arm --static
	arm-linux-gnueabi-objdump -dS ${TARGET}.arm > ${TARGET}.obj

clean:
	rm -f *.arm *.obj

