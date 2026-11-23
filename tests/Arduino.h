#pragma once
// Minimal host mock; tests include the real sketch rather than copying logic.
#include <algorithm>
#include <cstdint>
#include <string>
#include <cassert>
#include <limits>
using byte = unsigned char;
#define F(value) value
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;
constexpr int INPUT_PULLUP = 2;
inline unsigned long mockNow = 0;
inline int mockPins[32] = {};
inline unsigned int mockRisingEdges[32] = {};
inline unsigned int mockToneCount = 0;
inline unsigned long millis() { return mockNow; }
inline void digitalWrite(byte pin, int value) {
  if (mockPins[pin] == LOW && value == HIGH) ++mockRisingEdges[pin];
  mockPins[pin] = value;
}
inline int digitalRead(byte pin) { return mockPins[pin]; }
inline void pinMode(byte pin, int mode) { if (mode == INPUT_PULLUP) mockPins[pin] = HIGH; }
inline void tone(byte, unsigned int, unsigned long) { ++mockToneCount; }
inline void noTone(byte) {}
// Mock the Arduino Print formatter separately from the finite serial transport.
class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t value) = 0;
  virtual size_t write(const uint8_t *data, size_t length) {
    size_t count = 0;
    for (size_t i=0;i<length;i++) count += write(data[i]);
    return count;
  }
  size_t print(const char *value) {
    return write(reinterpret_cast<const uint8_t *>(value), std::char_traits<char>::length(value));
  }
  size_t print(char value) { return write(static_cast<uint8_t>(value)); }
  template<class T> size_t print(T value) { return print(std::to_string(value).c_str()); }
  template<class T> size_t println(T value) { return print(value) + write('\n'); }
  size_t println() { return write('\n'); }
};
struct SerialMock {
  std::string output, input;
  int writeCapacity = 4096;
  size_t writeLimit = std::numeric_limits<size_t>::max();
  unsigned int writeCalls = 0;
  bool connected = true;
  void begin(int) {}
  int available() const { return static_cast<int>(input.size()); }
  char read() {
    if (input.empty()) return 0;
    char value=input[0]; input.erase(0,1); return value;
  }
  int availableForWrite() const { return writeCapacity; }
  size_t write(const uint8_t *data, size_t length) {
    ++writeCalls;
    assert(length <= static_cast<size_t>(writeCapacity)); // oversizing could block on real R4
    size_t sent=connected ? std::min(length,writeLimit) : 0;
    output.append(reinterpret_cast<const char *>(data),sent);
    return sent;
  }
};
inline SerialMock Serial;
