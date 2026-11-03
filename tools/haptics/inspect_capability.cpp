#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"
static std::atomic<bool> done{false};
static void on_landscape(const Landscape* p) {
  if (!p || done.exchange(true)) return;
  printf("glove_count=%u\n",p->gloveDevices.gloveCount);
  for (uint32_t i=0; i<p->gloveDevices.gloveCount && i<MAX_NUMBER_OF_GLOVES; i++) {
    const auto &g=p->gloveDevices.gloves[i];
    printf("glove_id=%08X side=%d isHaptics=%s family=%d paired=%d\n",g.id,(int)g.side,g.isHaptics?"true":"false",(int)g.familyType,(int)g.pairedState);
  }
  fflush(stdout);
}
int main() {
  auto r=CoreSdk_InitializeCore(); if(r!=SDKReturnCode_Success) {printf("init=%d\n",int(r)); return 1;}
  r=CoreSdk_RegisterCallbackForLandscapeStream(on_landscape); if(r!=SDKReturnCode_Success) {printf("callback=%d\n",int(r)); return 2;}
  CoordinateSystemVUH c; CoordinateSystemVUH_Init(&c); c.view=AxisView_XFromViewer;c.up=AxisPolarity_PositiveZ;c.handedness=Side_Right;c.unitScale=1.0f;
  r=CoreSdk_InitializeCoordinateSystemWithVUH(c,true); if(r!=SDKReturnCode_Success) {printf("coordinates=%d\n",int(r));return 3;}
  r=CoreSdk_LookForHosts(5,false); if(r!=SDKReturnCode_Success) {printf("search=%d\n",int(r));return 4;}
  uint32_t n=0; CoreSdk_GetNumberOfAvailableHostsFound(&n); if(!n){printf("no hosts discovered\n");return 5;}
  std::unique_ptr<ManusHost[]> hosts(new ManusHost[n]); r=CoreSdk_GetAvailableHostsFound(hosts.get(),n); if(r!=SDKReturnCode_Success){printf("hosts=%d\n",int(r));return 6;}
  const char* wanted=std::getenv("MANUS_CORE_IP"); if(!wanted || !*wanted){printf("Set MANUS_CORE_IP to the Windows Core IPv4 address\n"); CoreSdk_ShutDown(); return 7;}
  int idx=-1; for(uint32_t i=0;i<n;i++) { std::string ip=hosts[i].ipAddress; if(ip==wanted || ip.rfind(std::string(wanted)+":",0)==0) {idx=(int)i;break;} }
  if(idx<0){printf("requested host not found\n");return 7;}
  r=CoreSdk_ConnectToHost(hosts[idx]);printf("connect=%d host=%s\n",int(r),hosts[idx].ipAddress);fflush(stdout);if(r!=SDKReturnCode_Success)return 8;
  for(int i=0;i<40 && !done;i++) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if(!done)printf("no landscape received\n");
  CoreSdk_ShutDown();return done?0:9;
}
