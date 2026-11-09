CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra
TEST_BINARY := /tmp/avrhvpp-protocol-test
STK500_BINARY := $(TEST_BINARY)-stk500

.PHONY: test
test:
	$(CXX) $(CXXFLAGS) -Ifakes protocol.cpp ../firmware-atmega32u2/avrhvpp.cpp -o $(TEST_BINARY)
	$(TEST_BINARY)
	$(CXX) $(CXXFLAGS) -DTEST_STK500 -Ifakes stk500.cpp ../firmware-atmega32u2/avrhvpp.cpp -o $(STK500_BINARY)
	$(STK500_BINARY)
