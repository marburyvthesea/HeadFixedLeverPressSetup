#pragma once
// Minimal host mock; tests include the real sketch rather than copying logic.
#include <algorithm>
#include <cstdint>
#include <string>
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
struct SerialMock {
  std::string output;
  void begin(int) {}
  int available() const { return 0; }
  char read() const { return 0; }
  void print(const char *value) { output += value; }
  void print(char value) { output += value; }
  template<class T> void print(T value) { output += std::to_string(value); }
  template<class T> void println(T value) { print(value); output += '\n'; }
  void println() { output += '\n'; }
};
inline SerialMock Serial;
