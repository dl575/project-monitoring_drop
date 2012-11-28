TARGET=

${TARGET}: ${TARGET}.c
	arm-linux-gnueabi-gcc -DUNIX ${TARGET}.c -o ${TARGET}.arm --static
	arm-linux-gnueabi-objdump -dS ${TARGET}.arm > ${TARGET}.obj

clean:
	rm -f *.arm *.obj

