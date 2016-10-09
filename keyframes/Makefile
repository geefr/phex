TARGET		= keyframes
OBJS		= keyframes.o phex_pose.o
SRCS        = keyframes.c phex_pose/phex_pose.cpp
INCLUDEDIRS	+= -I./ -I../include -I./phex_pose
LIBDIRS		+= -L../lib
CFLAGS		= $(INCLUDEDIRS) -Wall -DEBUG
LDFLAGS		= $(LIBDIRS) -lm -ldxl

CC			= g++

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)
	
$(OBJS): $(SRC)
	$(CC) -c $(SRCS) $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

