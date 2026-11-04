CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra
TEST_BINARY := /tmp/avrhvpp-protocol-test

.PHONY: test
test:
	$(CXX) $(CXXFLAGS) -Ifakes protocol.cpp ../firmware-atmega32u2/avrhvpp.cpp -o $(TEST_BINARY)
	$(TEST_BINARY)
