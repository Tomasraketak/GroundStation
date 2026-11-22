# Host tests of the ground station firmware and the header copies.
#   make -C test          build + run everything
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers

all: fw sync

gui:
	QT_QPA_PLATFORM=offscreen python3 gui_smoke.py

fw: fw_host_test
	./fw_host_test

fw_host_test: fw_host_test.cpp ../firmware/Ground_Station/Ground_Station.ino ../firmware/Ground_Station/*.h stub/*.h
	$(CXX) $(CXXFLAGS) -fsanitize=address,undefined -Istub -I../firmware/Ground_Station -o $@ fw_host_test.cpp

sync:
	python3 ../tools/sync_common.py

clean:
	rm -f fw_host_test

.PHONY: all fw sync gui clean
