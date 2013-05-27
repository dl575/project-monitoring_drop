all: dummy.arm

dummy.arm: dummy.c
	arm-linux-gnueabi-gcc -nostdlib -o dummy.arm dummy.c

dummy.objdump: dummy.arm
	arm-linux-gnueabi-objdump -dS dummy.arm > dummy.objdump

.PHONY: clean
clean:
	rm dummy.arm dummy.objdump
