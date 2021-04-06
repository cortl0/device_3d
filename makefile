TARGET = prepare
BBN_DIR = ./src/bnn

.PHONY: prepare all clean

all: $(TARGET)

clean:
	rm -rf $(BBN_DIR)
	mkdir $(BBN_DIR)

$(TARGET):
	git clone https://github.com/cortl0/binary_neurons_network.git $(BBN_DIR)
