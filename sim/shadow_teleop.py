#!/usr/bin/env python3
"""MANUS to Shadow Hand E3M5, simulation only; no robot or haptics output."""
import argparse
import math
import os
import time
from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from manus_ros2_msgs.msg import ManusGlove

SCENE = Path(os.environ.get('MANUS_MENAGERIE', str(Path(__file__).resolve().parents[1] / 'third_party/mujoco_menagerie'))) / 'shadow_hand/scene_right.xml'
FINGERS = ('Thumb','Index','Middle','Ring','Pinky')
FIELDS = ('ThumbCMCStretch','IndexMCPStretch','MiddleMCPStretch','RingMCPStretch','PinkyMCPStretch')
# Positional targets, radians. FFJ0 etc. are tendons coupling distal joints.
MAP = {
    'Thumb': [('rh_A_THJ4', 0.90), ('rh_A_THJ2', 0.35), ('rh_A_THJ1', 0.80)],
    'Index': [('rh_A_FFJ3', 1.0), ('rh_A_FFJ0', 1.7)],
    'Middle':[('rh_A_MFJ3', 1.0), ('rh_A_MFJ0', 1.7)],
    'Ring':  [('rh_A_RFJ3', 1.0), ('rh_A_RFJ0', 1.7)],
    'Pinky': [('rh_A_LFJ3', 1.0), ('rh_A_LFJ0', 1.7)],
}

class Glove(Node):
    def __init__(self, topic):
        super().__init__('manus_shadow_sim_input')
        self.data=None; self.received=None; self.samples=0
        self.create_subscription(ManusGlove,topic,self.callback,10)
        self.get_logger().info('SIMULATION ONLY; listening: '+topic)
    def callback(self,msg):
        if msg.side.lower()!='right': return
        d={e.type:e.value for e in msg.ergonomics}
        if not all(k in d and math.isfinite(d[k]) for k in FIELDS): return
        self.data=np.asarray([d[k] for k in FIELDS],dtype=float)
        self.received=time.monotonic(); self.samples+=1

class Retarget:
    def __init__(self, model, data):
        self.model=model; self.data=data; self.open=None; self.closed=None; self.enabled=False
        self.targets=np.clip(np.zeros(model.nu),model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1])
        self.data.ctrl[:]=self.targets
        self.act={name:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,name) for specs in MAP.values() for name,_ in specs}
        if any(v<0 for v in self.act.values()): raise ValueError('Shadow Hand actuator missing')
    def capture(self,name,v):
        self.enabled=False
        if v is None or not np.all(np.isfinite(v)):
            print('没有手套数据，无法记录标定。',flush=True); return
        setattr(self,name,np.array(v,dtype=float).copy())
        print(('张开' if name=='open' else '握拳')+'已记录：'+str(np.round(v,1).tolist()),flush=True)
        if self.open is not None and self.closed is not None:
            span=np.abs(self.closed-self.open)
            print('五指变化：'+str(np.round(span,1).tolist()),flush=True)
            if np.any(span<5): print('某些手指变化不足 5，请重新张开/握拳标定。',flush=True)
            else: print('标定完成，按 E 启用仿真。',flush=True)
    def valid(self):
        return self.open is not None and self.closed is not None and np.all(np.abs(self.closed-self.open)>=5.0)
    def apply(self,v,dt):
        if not self.enabled or not self.valid(): return
        fractions=np.clip((v-self.open)/(self.closed-self.open),0,1)
        desired=self.targets.copy()
        for i,f in enumerate(FINGERS):
            for name,closed in MAP[f]: desired[self.act[name]]=fractions[i]*closed
        desired=np.clip(desired,self.model.actuator_ctrlrange[:,0],self.model.actuator_ctrlrange[:,1])
        self.targets+=np.clip(desired-self.targets,-1.8*dt,1.8*dt)
        self.data.ctrl[:]=self.targets


def self_test():
    m=mujoco.MjModel.from_xml_path(str(SCENE))
    joints=('rh_THJ4','rh_FFJ3','rh_MFJ3','rh_RFJ3','rh_LFJ3')
    for i,(name,joint) in enumerate(zip(FINGERS,joints)):
        d=mujoco.MjData(m); c=Retarget(m,d)
        c.open=np.zeros(5); c.closed=np.full(5,40.); assert c.valid(); c.enabled=True
        jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,joint)
        assert jid>=0
        pos=m.jnt_qposadr[jid]; before=d.qpos[pos]
        v=np.zeros(5); v[i]=40
        for _ in range(600): c.apply(v,m.opt.timestep); mujoco.mj_step(m,d)
        target_name=MAP[name][0][0]; target=c.targets[c.act[target_name]]
        delta=abs(d.qpos[pos]-before)
        assert target>0.3 and delta>0.1
        assert all(c.targets[c.act[MAP[other][0][0]]]<0.01 for other in FINGERS if other!=name)
        assert np.isfinite(d.qpos).all()
        print(name,joint,'delta_rad',round(delta,3),'PASS')
    print('PASS: synthetic inputs move all five fingers independently in MuJoCo. No real glove claimed.')


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--topic',default=os.environ.get('MANUS_RIGHT_TOPIC', '/manus/right/glove_ID')); ap.add_argument('--self-test',action='store_true'); args=ap.parse_args()
    if args.self_test: self_test(); return
    rclpy.init(); glove=Glove(args.topic)
    model=mujoco.MjModel.from_xml_path(str(SCENE)); data=mujoco.MjData(model); ctrl=Retarget(model,data)
    print('\n仿真专用：手腕固定，五指简化映射；不连接真实机器人。\n在 MuJoCo 窗口按 O 记录张手，按 F 记录握拳，按 E 开始，空格暂停，C 清除标定。\n',flush=True)
    def fresh(): return glove.received is not None and time.monotonic()-glove.received<0.5
    def key(k):
        if k==32: ctrl.enabled=False; print('已暂停。',flush=True)
        elif k in (ord('c'),ord('C')): ctrl.open=ctrl.closed=None; ctrl.enabled=False; print('标定已清除。',flush=True)
        elif k in (ord('o'),ord('O'),ord('f'),ord('F')):
            if fresh(): ctrl.capture('open' if k in (ord('o'),ord('O')) else 'closed',glove.data)
            else: print('ROS 手套消息未到达，不能标定。',flush=True)
        elif k in (ord('e'),ord('E')):
            if fresh() and ctrl.valid(): ctrl.enabled=True; print('五指仿真已启用。',flush=True)
            else: print('先完成五指张手/握拳标定并确认消息可用。',flush=True)
    try:
        with mujoco.viewer.launch_passive(model,data,key_callback=key) as viewer:
            last_sync=time.monotonic()
            while viewer.is_running() and rclpy.ok():
                start=time.monotonic(); rclpy.spin_once(glove,timeout_sec=0.0)
                if ctrl.enabled and not fresh(): ctrl.enabled=False; print('ROS 消息超时，已暂停；重新启用需按 E。',flush=True)
                if ctrl.enabled: ctrl.apply(glove.data,model.opt.timestep)
                mujoco.mj_step(model,data)
                if start-last_sync>=1/30: viewer.sync(); last_sync=start
                time.sleep(max(0.,model.opt.timestep-(time.monotonic()-start)))
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        try: glove.destroy_node(); rclpy.try_shutdown()
        except KeyboardInterrupt: pass

if __name__=='__main__': main()
