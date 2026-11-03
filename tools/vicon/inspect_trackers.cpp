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
static std::atomic<bool> tracked{false};
static void on_tracking(const TrackerStreamInfo* info) {
  if(!info || !info->trackerCount || tracked.exchange(true)) return;
  printf("stream count=%u\n",info->trackerCount);
  for(uint32_t i=0;i<info->trackerCount && i<MAX_NUMBER_OF_TRACKERS;i++) {
    TrackerData t{}; auto r=CoreSdk_GetTrackerData(i,&t);
    printf("stream id=%s result=%d quality=%d pos=%.4f %.4f %.4f quat_xyzw=%.4f %.4f %.4f %.4f\n",t.trackerId.id,(int)r,(int)t.quality,t.position.x,t.position.y,t.position.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
  }
  fflush(stdout);
}
static void on_landscape(const Landscape* p) {
  if (!p || done.exchange(true)) return;
  printf("tracker_count=%u\n",p->trackers.trackerCount);
  for (uint32_t i=0; i<p->trackers.trackerCount && i<MAX_NUMBER_OF_TRACKERS; i++) {
    const auto &t=p->trackers.trackers[i];
    printf("id=%s system=%d role=%d user=%u manufacturer=%s product=%s\n",t.id,(int)t.systemType,(int)t.type,t.user,t.manufacturer,t.productName);
  }
  fflush(stdout);
}
int main() {
  auto r=CoreSdk_InitializeCore(); if(r!=SDKReturnCode_Success) {printf("init=%d\n",int(r)); return 1;}
  r=CoreSdk_RegisterCallbackForTrackerStream(on_tracking); if(r!=SDKReturnCode_Success) {printf("tracker callback=%d\n",int(r)); return 2;}
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
  for(int i=0;i<50 && (!done || !tracked);i++) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if(!done)printf("no landscape received\n");
  uint32_t count=0; r=CoreSdk_GetNumberOfAvailableTrackers(&count); printf("get_count=%d n=%u\n",(int)r,count);
  if(r==SDKReturnCode_Success && count && count<=MAX_NUMBER_OF_TRACKERS) {
    std::unique_ptr<TrackerId[]> ids(new TrackerId[count]); r=CoreSdk_GetIdsOfAvailableTrackers(ids.get(),count); printf("get_ids=%d\n",(int)r);
    if(r==SDKReturnCode_Success) for(uint32_t i=0;i<count;i++){TrackerData t{};r=CoreSdk_GetDataForTracker_UsingTrackerId(ids[i],&t);printf("data id=%s result=%d quality=%d pos=%.4f %.4f %.4f quat_xyzw=%.4f %.4f %.4f %.4f\n",ids[i].id,(int)r,(int)t.quality,t.position.x,t.position.y,t.position.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);}
  }
  CoreSdk_ShutDown();return done?0:9;
}
