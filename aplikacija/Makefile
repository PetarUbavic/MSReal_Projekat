sources=$(wildcard *.c)
objs=$(sources:.c=.o)

result=application

all: $(result)

$(result): $(objs)
	@echo -n "Building output binary: "
	@echo $@
	$(CC) -o $@ $(objs) -lm  # Link with the math library

%.o: %.c
	@echo -n "Compiling source into: "
	@echo $@
	$(CC) -c $<

%.d: %.c
	@echo -n "Creating dependency: "
	@echo $@
	@rm -f $@; \
		$(CC) -MM $(CPPFLAGS) $< > $@

-include $(sources:.c=.d)

.PHONY: clean

clean:
	@rm -rf $(result) *.o *.d
	@echo "Clean done.."
