TARGET=

${TARGET}: ${TARGET}.c
	arm-linux-gnueabi-gcc -DUNIX ${TARGET}.c -o ${TARGET}.arm --static

clean:
	rm -f *.arm

