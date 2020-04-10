BIN_DIR=bin
BIN=$(BIN_DIR)/cross-track
$(shell mkdir -p $(BIN_DIR))

SRC=src/cross-track.cpp
INC=-Ilib/eigen
CPP_FLAGS=-Wall -Werror -pedantic

all:
	g++ -g -std=c++17 $(CPP_FLAGS) -o $(BIN) $(SRC) $(INC)

clean:
	rm -rf $(BIN)
