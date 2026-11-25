#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <string>
using byte = unsigned char;
#define F(value) value
constexpr int LOW=0, HIGH=1, OUTPUT=1, INPUT_PULLUP=2;
inline unsigned long mockNow=1000;
inline int mockPins[32]={};
inline unsigned int mockRisingEdges[32]={};
inline unsigned int mockToneCount=0;
inline bool (*mockAllowWrite)()=nullptr;
inline unsigned long millis(){return mockNow;}
inline void digitalWrite(byte pin,int value){
 if(mockPins[pin]==LOW && value==HIGH)mockRisingEdges[pin]++;
 mockPins[pin]=value;
}
inline int digitalRead(byte pin){return mockPins[pin];}
inline void pinMode(byte pin,int mode){if(mode==INPUT_PULLUP)mockPins[pin]=HIGH;}
inline void tone(byte,unsigned int,unsigned long){mockToneCount++;}
inline void noTone(byte){}
struct SerialMock {
 std::string output,input;
 unsigned int writeCalls=0;
 bool readingAllowed=true;
 void begin(int){}
 int available()const{return static_cast<int>(input.size());}
 int read(){
  if(input.empty() || !readingAllowed)return -1;
  unsigned char c=input.front();input.erase(0,1);return c;
 }
 void append(const std::string &text){
  assert(mockAllowWrite==nullptr || mockAllowWrite());
  writeCalls++;output+=text;
 }
 void print(const char *v){append(v);}
 void print(char v){append(std::string(1,v));}
 template<class T>void print(T v){append(std::to_string(v));}
 template<class T>void println(T v){print(v);append("\r\n");}
 void println(){append("\r\n");}
};
inline SerialMock Serial;
