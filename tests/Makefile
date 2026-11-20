CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra
TEST_BINARY := /tmp/avrhvpp-protocol-test
STK500_BINARY := $(TEST_BINARY)-stk500
HOST_SOURCES := ../app/AvrIsp.cpp ../app/AvrUpdi.cpp ../app/Connection.cpp

.PHONY: test
test:
	$(CXX) $(CXXFLAGS) -Ifakes protocol.cpp $(HOST_SOURCES) ../firmware-atmega32u2/avrhvpp.cpp -o $(TEST_BINARY)
	$(TEST_BINARY)
	$(CXX) $(CXXFLAGS) -DTEST_STK500 -Ifakes stk500.cpp ../firmware-atmega32u2/avrhvpp.cpp -o $(STK500_BINARY)
	$(STK500_BINARY)
