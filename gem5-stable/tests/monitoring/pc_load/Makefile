CC=arm-linux-gnueabi-gcc
OBJ=arm-linux-gnueabi-objdump
CFLAGS=-c -Wall -O2 -I ../
LDFLAGS=--static
SOURCES=increment.c pc_load.c
HEADERS=increment.h ../monitoring.h
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=run.arm
DUMP=$(EXECUTABLE:.arm=.obj)

all: $(HEADERS) $(SOURCES) $(EXECUTABLE) $(DUMP)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
    
$(DUMP): $(EXECUTABLE)
	$(OBJ) -dS $(EXECUTABLE) > $@
	
.c.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean:
	rm -f *.o *.arm *.obj

