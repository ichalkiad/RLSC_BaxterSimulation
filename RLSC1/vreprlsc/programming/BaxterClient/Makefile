CFLAGS = -I../remoteApi -I../include -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -L../lib/ -lpthread -lorocos-kdl -lcurses -Wl,-rpath='../lib'

OBJS = ../remoteApi/extApi.o ../remoteApi/extApiPlatform.o BaxterTools.o ../remoteApi/extApiCustom.o
OUTPUTEX = BaxterClientExample
#OUTPUTSOL = BaxterClientSolution
OUTPUTSOL = IoannisChalkiadakis_s1635749

OS = $(shell uname -s)
ECHO = 

ifeq ($(OS), Linux)
	CFLAGS += -D__linux
else
	CFLAGS += -D__APPLE__
endif

all: $(OBJS) $(OUTPUTEX).o $(OUTPUTSOL).o
		@echo "Linking $(OBJS) $(OUTPUTEX).o to $(OUTPUTEX)"
		$(ECHO)$(CXX) $(CFLAGS) $(OBJS) $(OUTPUTEX).o -o $(OUTPUTEX) $(LDFLAGS)
		@echo "Linking $(OBJS) $(OUTPUTSOL).o to $(OUTPUTSOL)"
		$(ECHO)$(CXX) $(CFLAGS) $(OBJS) $(OUTPUTSOL).o -o $(OUTPUTSOL) $(LDFLAGS)

%.o: %.cpp
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

%.o: %.c
		@echo "Compiling $< to $@" 
		$(ECHO)$(CC) $(CFLAGS) -c $< -o $@

clean:
		@rm -f $(OBJS) $(OUTPUTEX).o $(OUTPUTSOL).o $(OUTPUTEX) $(OUTPUTSOL)
