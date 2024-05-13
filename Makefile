.PHONY: clean all install

EXE_NAME = $(notdir $(CURDIR)).out
COMPILE_PREFIX ?= 

CFLAGS  = 
CFLAGS += -g

LDFLAGS  = 

OBJS_C = $(addsuffix .o,$(wildcard *.c))
OBJS_CPP = $(addsuffix .o,$(wildcard *.cpp))

all: $(EXE_NAME)

$(EXE_NAME): $(OBJS_C) $(OBJS_CPP)
	$(COMPILE_PREFIX)g++ $^ $(LDFLAGS) -o $(EXE_NAME)

$(OBJS_C): %.c.o: %.c
	$(COMPILE_PREFIX)gcc -c $(CFLAGS) $< -o $@ 

$(OBJS_CPP): %.cpp.o: %.cpp
	$(COMPILE_PREFIX)g++ -c $(CFLAGS) $< -o $@ 

clean:
	rm -f $(EXE_NAME)
	rm -f $(OBJS_C) $(OBJS_CPP)