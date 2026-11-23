#include <cassert>
#include <climits>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "../leverTaskControllerWindow/leverTaskControllerWindow.ino"

void resetController() {
  taskMode=IMMEDIATE;trialState=NO_TRIAL;activeLever=0;
  sessionActive=stopPending=valveActive=valveGapActive=pressLogEnabled=false;
  valveStartedAt=valveClosedAt=0;
  trialWindowMs=15000;trialPauseMs=1000;buzzerMs=500;buzzerHz=4000;
  trialNumber=trialStartedAt=trialGapStartedAt=0;
  trialLever1=trialLever2=trialActivePresses=0;continueRequested=false;
  windowStartMinMs=1000;windowStartMaxMs=8000;windowLengthMinMs=2000;windowLengthMaxMs=5000;
  windowSeed=windowRngState=1;windowStartOffsetMs=windowEndOffsetMs=0;
  earlyL1=earlyL2=windowL1=windowL2=lateL1=lateL2=0;windowConfigured=false;
  omissionIntervalMs=1000;omissionResetDelayMs=100;omissionRewardPauseMs=100;
  omissionBuzzerMs=100;omissionBuzzerHz=4000;omissionConfigured=false;
  queueHead=queueTail=queueCount=0;pressNumber=0;commandLength=0;commandOverflow=false;
  lever1={2,3,1,INVALID,0,false};lever2={9,10,2,INVALID,0,false};
  mockNow=1000;mockToneCount=0;
  std::fill(std::begin(mockPins),std::end(mockPins),LOW);
  std::fill(std::begin(mockRisingEdges),std::end(mockRisingEdges),0);
  mockPins[lever1.noPin]=mockPins[lever2.noPin]=HIGH; // released: NO open, NC closed
  Serial.output.clear();Serial.input.clear();Serial.writeCapacity=4096;
  Serial.writeLimit=std::numeric_limits<size_t>::max();Serial.writeCalls=0;Serial.connected=true;
  Protocol.reset();txFaultLatched=txOverflowReportPending=false;
}
void drainTx() {
  for (unsigned int i=0;i<4096 && Protocol.queuedBytes();i++) {
    auto before=Protocol.queuedBytes(); serviceTx();
    if(Protocol.queuedBytes()==before)break;
  }
}
std::string command(const std::string &value) {
  assert(value.size()<sizeof(commandBuffer));
  std::copy(value.begin(),value.end(),commandBuffer);commandLength=static_cast<byte>(value.size());
  drainTx();auto before=Serial.output.size();handleCommand();commandLength=0;commandOverflow=false;drainTx();return Serial.output.substr(before);
}
void tick(unsigned long now) {mockNow=now;loop();}
void pressAt(byte id,unsigned long elapsed) {mockNow=trialStartedAt+elapsed;handlePress(id,mockNow);}
void configure(const std::string &mode,bool logging=true,const std::string &selection="LEVER2") {
  assert(command("SET_TASK,"+mode)=="ACK_TASK,"+mode+"\n");
  assert(command("SET_ACTIVE,"+selection)=="ACK_ACTIVE,"+selection+"\n");
  assert(command("SET_TRIAL,1000,100,100,4000")=="ACK_TRIAL,1000,100,100,4000\n");
  if(mode=="WINDOW")assert(command("SET_WINDOW,200,200,300,300,42")=="ACK_WINDOW,200,200,300,300,42\n");
  if(mode=="OMISSION")assert(command("SET_OMISSION,1000,100,100,100,4000")=="ACK_OMISSION,1000,100,100,100,4000\n");
  if(logging)assert(command("SET_PRESS_LOG,ON")=="ACK_PRESS_LOG,ON\n");
}
std::vector<std::vector<std::string>> records(const std::string &kind) {
  drainTx();
  std::vector<std::vector<std::string>> result;std::istringstream lines(Serial.output);std::string line;
  while(std::getline(lines,line)) {
    if(line.rfind(kind+",",0)!=0)continue;
    std::vector<std::string> fields;std::istringstream cells(line);std::string cell;
    while(std::getline(cells,cell,','))fields.push_back(cell);
    result.push_back(fields);
  }
  return result;
}
void event(std::size_t index,unsigned long trial,byte lever,unsigned long clock,long elapsed,
           bool inTrial,bool selected,bool qualifying,const std::string &phase) {
  auto all=records("PRESS_EVENT");assert(index<all.size());
  std::vector<std::string> want={"PRESS_EVENT",std::to_string(trial),std::to_string(lever),
    std::to_string(clock),std::to_string(elapsed),std::to_string(inTrial),std::to_string(selected),
    std::to_string(qualifying),phase};
  assert(all[index]==want);
}
void testOptInAndCompatibility() {
  resetController();configure("TRIAL",false);command("START");pressAt(2,100);
  assert(records("PRESS_EVENT").empty());
  assert(command("SET_PRESS_LOG,ON")=="ERROR,STOP_BEFORE_CONFIGURING\n");
  tick(2000);assert(records("TRIAL_END").back()[4]=="1");
  tick(2350);command("STOP");
  assert(command("SET_PRESS_LOG,INVALID")=="ERROR,INVALID_PRESS_LOG\n");
  assert(!pressLogEnabled);
  assert(command("SET_PRESS_LOG,ON")=="ACK_PRESS_LOG,ON\n");assert(pressLogEnabled);
  assert(command("SET_PRESS_LOG,OFF")=="ACK_PRESS_LOG,OFF\n");assert(!pressLogEnabled);
  command("SET_PRESS_LOG,ON");command("START");pressAt(2,50);
  assert(records("PRESS_EVENT").size()==1);
  command("STOP");assert(!pressLogEnabled);
  const auto before=records("PRESS_EVENT").size();handlePress(2,mockNow);
  assert(records("PRESS_EVENT").size()==before);
  command("START");pressAt(2,50);assert(records("PRESS_EVENT").size()==before);
}
void testTrialEventsAndSummary() {
  resetController();configure("TRIAL");command("START");
  pressAt(1,100);pressAt(2,250);
  event(0,1,1,1100,100,true,false,false,"TRIAL");
  event(1,1,2,1250,250,true,true,true,"TRIAL");
  tick(2000);auto summary=records("TRIAL_END").back();
  assert(summary.size()==10 && summary[2]=="2" && summary[4]=="1");
  assert(summary[5]=="1" && summary[6]=="1" && summary[7]=="1");
  assert(Serial.output.find("PRESS_EVENT,1,2")<Serial.output.find("TRIAL_END,"));
  // Exactly at deadline, the loop closes the trial first. These presses are ITI.
  handlePress(2,mockNow);handlePress(1,mockNow);
  event(2,0,2,2000,-1,false,true,false,"INTERTRIAL");
  event(3,0,1,2000,-1,false,false,false,"INTERTRIAL");
  assert(trialLever1==1 && trialLever2==1 && trialActivePresses==1);
  assert(valveActive && mockRisingEdges[solenoidPin]==1 && solenoidMs==350);
  tick(2350);command("CONTINUE");tick(2450);
  assert(trialNumber==2 && trialActivePresses==0);
  pressAt(2,10);event(4,2,2,2460,10,true,true,true,"TRIAL");
}
void testWindowClassification() {
  resetController();configure("WINDOW");command("START");
  pressAt(2,199);pressAt(1,200);pressAt(2,200);pressAt(2,499);pressAt(2,500);pressAt(1,999);
  event(0,1,2,1199,199,true,true,false,"EARLY");
  event(1,1,1,1200,200,true,false,false,"WINDOW");
  event(2,1,2,1200,200,true,true,true,"WINDOW");
  event(3,1,2,1499,499,true,true,true,"WINDOW");
  event(4,1,2,1500,500,true,true,false,"LATE");
  event(5,1,1,1999,999,true,false,false,"LATE");
  tick(2000);auto result=records("WINDOW_END").back();
  assert(result.size()==18 && result[2]=="6" && result[7]=="2" && result[4]=="1");
  assert(result[12]=="0" && result[13]=="1" && result[14]=="1" && result[15]=="2");
  assert(result[16]=="1" && result[17]=="1");
  handlePress(2,mockNow);event(6,0,2,2000,-1,false,true,false,"INTERTRIAL");
  // Both selected levers qualify in WINDOW; still exactly one reward at the end.
  resetController();configure("WINDOW",true,"BOTH");command("START");pressAt(1,200);pressAt(2,200);
  event(0,1,1,1200,200,true,true,true,"WINDOW");event(1,1,2,1200,200,true,true,true,"WINDOW");
  tick(2000);assert(mockRisingEdges[solenoidPin]==1);
}
void testAbortAndUnrewardedGaps() {
  resetController();configure("WINDOW");command("START");pressAt(1,100);pressAt(2,250);
  mockNow=1300;command("STOP");
  auto aborted=records("WINDOW_ABORT").back();assert(aborted.size()==18 && aborted[4]=="0");
  assert(records("PRESS_EVENT").size()==2 && !pressLogEnabled && !valveActive);
  assert(Serial.output.find("PRESS_EVENT,1,2")<Serial.output.find("WINDOW_ABORT,"));
  resetController();configure("TRIAL");command("START");tick(2000);
  assert(!valveActive && trialState==TRIAL_GAP);
  handlePress(2,mockNow);event(0,0,2,2000,-1,false,true,false,"INTERTRIAL");
  assert(trialActivePresses==0 && records("TRIAL_END").back()[2]=="0");
}
void testDebouncedDetectionAndHeldLever() {
  resetController();configure("TRIAL");command("START");tick(1000);tick(1010); // arm released levers
  mockPins[lever2.noPin]=LOW;mockPins[lever2.ncPin]=HIGH;tick(1100);
  tick(1109);assert(records("PRESS_EVENT").empty());
  tick(1110);event(0,1,2,1110,110,true,true,true,"TRIAL");
  tick(1200);assert(records("PRESS_EVENT").size()==1); // held does not repeat
  mockPins[lever2.noPin]=HIGH;mockPins[lever2.ncPin]=LOW;tick(1210);tick(1220);
  mockPins[lever2.noPin]=LOW;mockPins[lever2.ncPin]=HIGH;tick(1250);tick(1260);
  event(1,1,2,1260,260,true,true,true,"TRIAL");
  assert(trialActivePresses==2);
  resetController();mockPins[lever2.noPin]=LOW;mockPins[lever2.ncPin]=HIGH;
  configure("TRIAL");command("START");tick(1000);tick(1010);tick(1050);
  assert(records("PRESS_EVENT").empty()); // held before START is not a new event
}
void testImmediateAndOmissionUnaffected() {
  resetController();configure("IMMEDIATE");command("START");pressAt(2,100);servicePressQueue(mockNow);
  assert(records("PRESS_EVENT").empty() && records("PRESS").size()==1 && valveActive);
  resetController();configure("OMISSION");command("START");pressAt(1,100);
  assert(records("PRESS_EVENT").empty() && trialLever1==1 && trialState==TRIAL_RUNNING);
  pressAt(2,200);assert(trialState==OMISSION_RESET_GAP && !valveActive);
  assert(records("LEVER_PRESS").size()==1 && records("INTERVAL_END").back()[2]=="RESET");
  tick(1300);assert(trialNumber==2 && trialState==TRIAL_RUNNING);
  tick(2300);assert(valveActive && records("INTERVAL_END").back()[2]=="REWARD");
  assert(records("PRESS_EVENT").empty());
}
void testUnsignedClockWrap() {
  resetController();configure("TRIAL");mockNow=ULONG_MAX-100;command("START");
  pressAt(2,200);event(0,1,2,99,200,true,true,true,"TRIAL");
  tick(trialStartedAt+1000);assert(valveActive && records("TRIAL_END").back()[7]=="1");
}
void testBlockedTxKeepsWaterAndStopResponsive() {
  resetController();configure("TRIAL");command("START");
  const auto before=Serial.output;
  Serial.writeCapacity=0;Serial.writeCalls=0;
  pressAt(2,250);tick(2000);
  assert(valveActive && mockPins[solenoidPin]==HIGH && Protocol.queuedBytes()>0);
  // Receive STOP while no output byte can move. Control state still stops now.
  Serial.input="STOP\n";tick(2100);
  assert(!sessionActive && stopPending && valveActive && Serial.input.empty());
  tick(2349);assert(valveActive && mockPins[solenoidPin]==HIGH);
  tick(2350);assert(!valveActive && mockPins[solenoidPin]==LOW && !stopPending);
  assert(Serial.output==before && Serial.writeCalls==0); // never tried a blocking write
  assert(!Protocol.faultPending && !txFaultLatched);
  // Recovery accepts short writes, preserving the unsent tail and record order.
  Serial.writeCapacity=3;Serial.writeLimit=2;drainTx();
  event(0,1,2,1250,250,true,true,true,"TRIAL");
  assert(records("TRIAL_END").back()[4]=="1");
  const auto eventPos=Serial.output.find("PRESS_EVENT,");
  const auto endPos=Serial.output.find("TRIAL_END,");
  const auto ackPos=Serial.output.find("ACK_STOP\n");
  assert(eventPos<endPos && endPos<ackPos && Protocol.queuedBytes()==0);
}
void testDisconnectedAndPartialWriterRetainsBytes() {
  resetController();configure("TRIAL");command("START");
  const auto before=Serial.output;
  Serial.connected=false; // native core may report FIFO capacity but write zero when disconnected
  pressAt(2,100);auto pending=Protocol.queuedBytes();
  serviceTx();assert(Protocol.queuedBytes()==pending && Serial.output==before);
  tick(2000);assert(valveActive);tick(2350);assert(!valveActive);
  assert(!Protocol.faultPending && !txFaultLatched);
  Serial.connected=true;Serial.writeLimit=1;drainTx();
  event(0,1,2,1100,100,true,true,true,"TRIAL");
  assert(records("TRIAL_END").size()==1 && Protocol.queuedBytes()==0);
  // Stage complete lines atomically, then drain a byte at a time across FIFO wraps.
  resetController();Serial.writeCapacity=5;Serial.writeLimit=1;
  Protocol.print("ACK_");Protocol.print("TEST");
  assert(Protocol.queuedBytes()==0 && Serial.output.empty());
  Protocol.println();drainTx();assert(Serial.output=="ACK_TEST\n");
  Serial.output.clear();std::string expected;
  for(unsigned int i=0;i<600;i++) {
    Protocol.print("TEST,");Protocol.println(i);
    expected+="TEST,"+std::to_string(i)+"\n";
    drainTx();
  }
  assert(Serial.output==expected && Protocol.queuedBytes()==0);
}
void testOverflowIsExplicitAndStopsSession() {
  resetController();configure("TRIAL");command("START");
  Serial.output.clear();Serial.writeCapacity=0;Serial.writeCalls=0;
  for(unsigned int i=0;i<200 && sessionActive;i++) {
    mockNow++;handlePress(2,mockNow);loop();
  }
  assert(!sessionActive && !pressLogEnabled && txFaultLatched);
  assert(!valveActive && mockPins[solenoidPin]==LOW);
  assert(Protocol.droppedLines==1 && !txOverflowReportPending);
  assert(Serial.output.empty() && Serial.writeCalls==0);
  assert(command("START").empty() && !sessionActive); // cannot restart before fault records drain
  const auto expectedActive=trialActivePresses;
  Serial.writeCapacity=7;Serial.writeLimit=3;drainTx();
  auto events=records("PRESS_EVENT");auto errors=records("ERROR");
  assert(events.size()+1==expectedActive);
  for(const auto &line:events) assert(line.size()==9);
  assert(errors.size()==1 && errors[0].size()==4);
  assert(errors[0][1]=="TX_OVERFLOW" && errors[0][2]=="1" && errors[0][3]=="SESSION_STOPPED");
  auto abort=records("TRIAL_ABORT");assert(abort.size()==1 && abort[0][4]=="0");
  assert(std::stoul(abort[0][7])==expectedActive);
  assert(Serial.output.find("ERROR,TX_OVERFLOW,")<Serial.output.find("ACK_STOP\n"));
  assert(Serial.output.rfind("ACK_STOP\n")==Serial.output.size()-9);
  loop();assert(!txFaultLatched); // recovered transport permits normal connection/configuration again
  assert(command("SET_TASK,TRIAL")=="ACK_TASK,TRIAL\n");
}
void testOverflowDoesNotTruncateActiveReward() {
  resetController();configure("TRIAL");command("START");
  Serial.writeCapacity=0;pressAt(2,100);tick(2000);assert(valveActive);
  for(unsigned int i=0;i<200 && sessionActive;i++) {
    mockNow++;handlePress(2,mockNow);loop(); // presses during reward are logged as ITI
  }
  assert(txFaultLatched && !sessionActive && valveActive);
  tick(2349);assert(valveActive && mockPins[solenoidPin]==HIGH);
  tick(2350);assert(!valveActive && mockPins[solenoidPin]==LOW && !stopPending);
  Serial.writeCapacity=64;drainTx();
  assert(records("TRIAL_END").size()==1 && records("TRIAL_END")[0][4]=="1");
  assert(records("ERROR").size()==1 && Serial.output.find("ACK_STOP\n")!=std::string::npos);
}
void fillNormalTxWithoutOverflow() {
  const char *padding="TEST,012345678901234567890123456789012345678901234567890123456789";
  while(Protocol.roomFor(80)) Protocol.println(padding);
  assert(!Protocol.roomFor(80) && !Protocol.faultPending);
}
void testImmediateBackpressureRetainsCanceledPress() {
  resetController();configure("IMMEDIATE");command("START");
  Serial.output.clear();Serial.writeCapacity=0;
  fillNormalTxWithoutOverflow();mockNow=1100;handlePress(2,mockNow);
  assert(queueCount==1);servicePressQueue(mockNow);serviceTxFault();
  assert(txFaultLatched && !sessionActive && stopPending && queueCount==1);
  assert(!valveActive && Protocol.droppedLines==0);
  Serial.writeCapacity=17;Serial.writeLimit=5;
  for(unsigned int i=0;i<4096 && (Protocol.queuedBytes() || stopPending || txFaultLatched);i++)loop();
  assert(queueCount==0 && !stopPending && !txFaultLatched && !valveActive);
  auto presses=records("PRESS");assert(presses.size()==1);
  assert(presses[0][3]=="0" && presses[0][4]=="1100");
  auto errors=records("ERROR");assert(errors.size()==1 && errors[0][2]=="0");
  assert(Serial.output.find("PRESS,1,2,0,1100\n")<Serial.output.find("ACK_STOP\n"));
}
void testStopUsesReservedCapacity() {
  resetController();configure("TRIAL");command("START");
  Serial.output.clear();Serial.writeCapacity=0;fillNormalTxWithoutOverflow();
  mockNow=1400;command("STOP");
  assert(!sessionActive && !stopPending && !Protocol.faultPending && !txFaultLatched);
  Serial.writeCapacity=1;drainTx();
  assert(records("TRIAL_ABORT").size()==1 && records("ERROR").empty());
  assert(Serial.output.find("TRIAL_ABORT,")<Serial.output.find("ACK_STOP\n"));
}
int main() {
  testOptInAndCompatibility();testTrialEventsAndSummary();testWindowClassification();
  testAbortAndUnrewardedGaps();testDebouncedDetectionAndHeldLever();
  testImmediateAndOmissionUnaffected();testUnsignedClockWrap();
  testBlockedTxKeepsWaterAndStopResponsive();testDisconnectedAndPartialWriterRetainsBytes();
  testOverflowIsExplicitAndStopsSession();testOverflowDoesNotTruncateActiveReward();
  testImmediateBackpressureRetainsCanceledPress();testStopUsesReservedCapacity();
  std::cout<<"All individual-press timing, stalled/partial TX and unified-task regression checks passed.\n";
}
