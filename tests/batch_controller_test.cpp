#include <cassert>
#include <climits>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "../leverTaskControllerWindow/leverTaskControllerWindow.ino"

void resetController(){
 taskMode=IMMEDIATE;trialState=NO_TRIAL;activeLever=0;
 sessionActive=stopPending=valveActive=valveGapActive=false;
 valveStartedAt=valveClosedAt=0;trialWindowMs=1000;trialPauseMs=100;buzzerMs=100;buzzerHz=4000;
 trialNumber=trialStartedAt=trialGapStartedAt=0;
 trialLever1=trialLever2=trialActivePresses=0;continueRequested=false;
 windowStartMinMs=windowStartMaxMs=200;windowLengthMinMs=windowLengthMaxMs=300;
 windowSeed=windowRngState=1;windowStartOffsetMs=windowEndOffsetMs=0;
 earlyL1=earlyL2=windowL1=windowL2=lateL1=lateL2=0;windowConfigured=false;
 omissionIntervalMs=1000;omissionResetDelayMs=omissionRewardPauseMs=100;
 omissionBuzzerMs=100;omissionBuzzerHz=4000;omissionConfigured=false;
 batchLogging=batchReadPending=false;batchReadTarget=0;batchPressCount=0;batchDropped=0;
 batchSnapshot={};batchDeferredError=nullptr;
 queueHead=queueTail=queueCount=0;pressNumber=0;commandLength=0;commandOverflow=false;
 lever1={2,3,1,INVALID,0,false};lever2={9,10,2,INVALID,0,false};
 mockNow=1000;mockToneCount=0;
 std::fill(std::begin(mockPins),std::end(mockPins),LOW);
 std::fill(std::begin(mockRisingEdges),std::end(mockRisingEdges),0);
 mockPins[lever1.noPin]=mockPins[lever2.noPin]=HIGH;
 Serial.output.clear();Serial.input.clear();Serial.writeCalls=0;Serial.readingAllowed=true;
 mockAllowWrite=[](){return !batchCritical();};
}
std::string command(const std::string &value){
 assert(value.size()<sizeof(commandBuffer));
 std::copy(value.begin(),value.end(),commandBuffer);commandLength=static_cast<byte>(value.size());
 auto before=Serial.output.size();handleCommand();return Serial.output.substr(before);
}
void tick(unsigned long now){mockNow=now;loop();}
void advance(unsigned long elapsed){tick(mockNow+elapsed);}
void press(byte id,unsigned long elapsed){mockNow=trialStartedAt+elapsed;handlePress(id,mockNow);}
void configure(const char *mode="TRIAL",bool batch=true,const char *selection="LEVER2"){
 assert(command(std::string("SET_TASK,")+mode)==std::string("ACK_TASK,")+mode+"\r\n");
 assert(command(std::string("SET_ACTIVE,")+selection)==std::string("ACK_ACTIVE,")+selection+"\r\n");
 assert(command("SET_TRIAL,1000,100,100,4000")=="ACK_TRIAL,1000,100,100,4000\r\n");
 if(taskMode==WINDOW)assert(command("SET_WINDOW,200,200,300,300,1")=="ACK_WINDOW,200,200,300,300,1\r\n");
 if(batch)assert(command("SET_PRESS_LOG,BATCH")=="ACK_PRESS_LOG,BATCH,1024,325\r\n");
}
std::vector<std::string> fields(const std::string &line){
 std::istringstream in(line);std::string field;std::vector<std::string> result;
 while(std::getline(in,field,',')){if(!field.empty() && field.back()=='\n')field.pop_back();if(!field.empty() && field.back()=='\r')field.pop_back();result.push_back(field);}
 return result;
}
void checkSmall(const std::string &reply){assert(!reply.empty() && reply.size()<64);}
void finishReward(){tick(trialStartedAt+trialWindowMs);if(valveActive)advance(solenoidMs);}
void acknowledge(){assert(command("ACK_BATCH,"+std::to_string(batchSnapshot.trial))=="ACK_BATCH,"+std::to_string(batchSnapshot.trial)+"\r\n");}
void testHandshakeAndQuietTrial(){
 resetController();setup();assert(Serial.output=="READY\r\n");
 assert(command("STOP")=="ACK_STOP\r\n");assert(command("GET_BATCH")=="BATCH_EMPTY\r\n");
 configure();assert(command("START")=="ACK_START\r\n");
 auto output=Serial.output;unsigned int writes=Serial.writeCalls;
 assert(command("GET_BATCH").empty());assert(batchReadPending);
 press(1,100);press(2,250);tick(1999);
 assert(Serial.output==output && Serial.writeCalls==writes && !valveActive);
 tick(2000);assert(valveActive && batchSnapshot.valid && batchSnapshot.completed);
 assert(batchSnapshot.water && batchSnapshot.lever1==1 && batchSnapshot.lever2==1 && batchSnapshot.active==1);
 assert(Serial.output==output && Serial.writeCalls==writes);
 tick(2324);assert(valveActive && Serial.output==output);
 tick(2325);assert(!valveActive && mockPins[solenoidPin]==LOW && mockRisingEdges[solenoidPin]==1);
 assert(Serial.output.substr(output.size())=="BATCH_READY,1,2,0,1,1,0\r\n");
 assert(command("GET_META,1")=="META,1,1000,2000\r\n");
 assert(command("GET_COUNTS,1")=="COUNTS,1,1,1,1\r\n");
 assert(command("GET_WINDOW,1")=="WINDOW,1,0,1000\r\n");
 assert(command("GET_CONTEXT,1")=="CONTEXT,1,TRIAL,LEVER2,325\r\n");
 assert(command("GET_PRESS,1,1")=="PRESS_DATA,1,1,200,501\r\n");
 assert(command("CONTINUE")=="ERROR,DATA_PENDING\r\n");
 acknowledge();assert(command("ACK_BATCH,1")=="ACK_BATCH,1\r\n");
 assert(command("GET_PRESS,1,1")=="PRESS_DATA,1,1,200,501\r\n");
 assert(command("CONTINUE")=="ACK_CONTINUE,1\r\n");
 // Host pull in the configured gap targets the NEXT trial, not old ACKed data.
 assert(command("GET_BATCH").empty());assert(batchReadTarget==2);
 tick(2424);assert(trialNumber==1);
 output=Serial.output;tick(2425);assert(trialNumber==2 && batchPressCount==0 && batchReadPending);
 assert(Serial.output==output);tick(3425);assert(!valveActive && batchSnapshot.trial==2);
 assert(Serial.output.substr(output.size())=="BATCH_READY,2,0,0,1,0,0\r\n");
}
void testWindowPagesAndCounts(){
 resetController();configure("WINDOW",true,"BOTH");command("START");
 auto output=Serial.output;
 press(1,199);press(1,200);press(2,200);press(2,499);press(2,500);press(1,999);
 assert(Serial.output==output);finishReward();
 assert(batchSnapshot.active==3 && batchSnapshot.lever1==3 && batchSnapshot.lever2==3);
 assert(batchSnapshot.windowStart==200 && batchSnapshot.windowEnd==500);
 assert(command("GET_BATCH")=="BATCH_READY,1,6,0,1,1,0\r\n");
 assert(command("GET_PRESS,1,1")=="PRESS_DATA,1,1,398,400,401,999\r\n");
 assert(command("GET_PRESS,1,5")=="PRESS_DATA,1,5,1001,1998\r\n");
 assert(command("GET_PRESS,1,6")=="PRESS_DATA,1,6,1998\r\n");
 assert(command("GET_PRESS,1,0")=="ERROR,PRESS_INDEX_RANGE\r\n");
 assert(command("GET_PRESS,1,7")=="ERROR,PRESS_INDEX_RANGE\r\n");
 assert(command("GET_PRESS,2,1")=="ERROR,BATCH_NOT_FOUND\r\n");
 // Largest normal packed words, trial ID and page position remain below64B.
 batchSnapshot.trial=UINT32_MAX;batchPressCount=1024;
 for(unsigned int i=1020;i<1024;i++)batchPressWords[i]=7200001;
 checkSmall(command("GET_PRESS,4294967295,1021"));
 checkSmall(command("GET_META,4294967295"));checkSmall(command("GET_COUNTS,4294967295"));
 checkSmall(command("GET_WINDOW,4294967295"));checkSmall(command("GET_CONTEXT,4294967295"));
 checkSmall(command("GET_BATCH"));checkSmall(command("ACK_BATCH,4294967295"));
}
void testStopAndRecovery(){
 resetController();configure();command("START");press(2,100);
 assert(command("GET_BATCH").empty());
 mockNow=1500;assert(command("STOP")=="ACK_STOP\r\n");
 assert(!valveActive && !sessionActive && !batchReadPending && batchSnapshot.valid);
 assert(!batchSnapshot.completed && !batchSnapshot.water && batchSnapshot.active==1);
 assert(command("GET_BATCH")=="BATCH_READY,1,1,0,0,0,0\r\n");
 assert(command("SET_TASK,WINDOW")=="ERROR,DATA_PENDING\r\n");
 assert(command("START")=="ERROR,DATA_PENDING\r\n");
 auto data=command("GET_PRESS,1,1");assert(data==command("GET_PRESS,1,1"));
 acknowledge();assert(command("SET_TASK,TRIAL")=="ACK_TASK,TRIAL\r\n");assert(!batchLogging);
 command("SET_ACTIVE,LEVER1");command("SET_TRIAL,2000,200,0,1000");
 assert(command("GET_CONTEXT,1")=="CONTEXT,1,TRIAL,LEVER2,325\r\n");
 assert(command("GET_WINDOW,1")=="WINDOW,1,0,1000\r\n");
 // STOP during an existing pulse waits to reply and preserves reward metadata.
 resetController();configure();command("START");press(2,100);tick(2000);
 auto output=Serial.output;assert(command("GET_BATCH").empty());assert(command("STOP").empty());
 assert(!sessionActive && valveActive && stopPending && !batchReadPending);
 tick(2324);assert(Serial.output==output && valveActive);
 tick(2325);assert(Serial.output.substr(output.size())=="ACK_STOP\r\n");
 assert(command("GET_BATCH")=="BATCH_READY,1,1,0,1,1,0\r\n");
}
void testOverflowAndDeferredErrors(){
 resetController();configure();command("START");auto output=Serial.output;
 for(unsigned int i=0;i<1024;i++)press(2,static_cast<unsigned long>(i%999));
 assert(batchPressCount==1024 && batchDropped==0 && Serial.output==output);
 press(2,999);assert(!sessionActive && !valveActive && !batchSnapshot.completed && !batchSnapshot.water);
 assert(batchDropped==1 && batchSnapshot.active==1025 && batchSnapshot.lever2==1025 && batchPressCount==1024);
 assert(Serial.output==output);assert(command("GET_BATCH")=="BATCH_READY,1,1024,1,0,0,0\r\n");
 assert(command("START")=="ERROR,DATA_PENDING\r\n");
 resetController();configure();command("START");output=Serial.output;
 assert(command("GET_COUNTS,1").empty());assert(command("SET_ACTIVE,BOTH").empty());
 assert(activeLever==2 && Serial.output==output);tick(2000);
 assert(Serial.output.substr(output.size())=="ERROR,BATCH_BUSY\r\n");
 // A negative USB read must not consume/corrupt the command buffer.
 commandLength=0;commandOverflow=false;Serial.input="STOP\n";Serial.readingAllowed=false;readCommands();
 assert(commandLength==0 && !commandOverflow && Serial.input=="STOP\n");
 Serial.readingAllowed=true;readCommands();assert(Serial.input.empty());
 // Overlong input is consumed in bounded chunks without any timed output.
 resetController();configure();command("START");output=Serial.output;
 commandLength=0;commandOverflow=false;Serial.input=std::string(100,'x')+"\n";
 for(int i=0;i<4;i++)advance(1);
 assert(Serial.input.empty() && Serial.output==output && commandLength==0);
 tick(2000);assert(Serial.output.substr(output.size())=="ERROR,COMMAND_TOO_LONG\r\n");
}
void testDetectorAndClockWrap(){
 resetController();configure();command("START");tick(1000);tick(1010);
 mockPins[lever2.noPin]=LOW;mockPins[lever2.ncPin]=HIGH;tick(1100);tick(1109);assert(batchPressCount==0);
 tick(1110);assert(batchPressCount==1 && batchPressWords[0]==221);
 tick(1500);assert(batchPressCount==1); // Held levers do not repeat.
 resetController();configure();mockNow=ULONG_MAX-100;command("START");
 press(2,200);assert(batchPressWords[0]==401);tick(trialStartedAt+1000);advance(solenoidMs);
 assert(batchSnapshot.startMs==static_cast<uint32_t>(ULONG_MAX-100));
 assert(static_cast<uint32_t>(batchSnapshot.startMs+(batchPressWords[0]>>1))==99);
}
void testLegacyModes(){
 resetController();configure("TRIAL",false);assert(!batchLogging);command("START");
 assert(Serial.output.find("TRIAL_START,1,1000\r\n")!=std::string::npos);
 press(2,100);finishReward();assert(Serial.output.find("TRIAL_END,1,1,1,1,0,1,1,1000,2000\r\n")!=std::string::npos);
 resetController();configure("WINDOW",false);command("START");press(2,199);press(1,200);press(2,500);finishReward();
 assert(!valveActive && trialActivePresses==0 && Serial.output.find("WINDOW_END,")!=std::string::npos);
 resetController();command("SET_TASK,IMMEDIATE");command("SET_ACTIVE,LEVER2");command("START");
 mockNow=1100;handlePress(2,mockNow);servicePressQueue(mockNow);assert(valveActive && Serial.output.find("PRESS,1,2,1,1100\r\n")!=std::string::npos);
 resetController();command("SET_TASK,OMISSION");command("SET_ACTIVE,LEVER2");
 command("SET_OMISSION,1000,100,100,100,4000");command("START");press(1,100);press(2,200);
 assert(trialState==OMISSION_RESET_GAP && !valveActive && Serial.output.find("INTERVAL_END,1,RESET,2\r\n")!=std::string::npos);
 tick(1300);tick(2300);assert(valveActive && Serial.output.find("INTERVAL_END,2,REWARD,0\r\n")!=std::string::npos);
}
int main(){
 testHandshakeAndQuietTrial();testWindowPagesAndCounts();testStopAndRecovery();
 testOverflowAndDeferredErrors();testDetectorAndClockWrap();testLegacyModes();
 std::cout<<"All buffered-trial timing, pull/retry/recovery, overflow and legacy-task checks passed.\n";
}
