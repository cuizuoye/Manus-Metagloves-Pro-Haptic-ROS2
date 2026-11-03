#!/usr/bin/env python3
"""IMU orientation + finger flex teleoperation; fixed wrist origin, MuJoCo only."""
import argparse
import math
import os
import time
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from shadow_teleop import SCENE, Glove, Retarget


def normalize(q):
    q=np.asarray(q,dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all(): return None
    n=np.linalg.norm(q)
    if not 0.5 < n < 1.5: return None
    return q/n


def mul(a,b):
    out=np.empty(4); mujoco.mju_mulQuat(out,a,b); return out


def inv(q): return q * np.array([1.,-1.,-1.,-1.])


class ImuGlove(Glove):
    def __init__(self,topic):
        super().__init__(topic); self.imu=None; self.imu_received=None
    def callback(self,msg):
        super().callback(msg)
        if msg.side.lower() != 'right' or msg.raw_sensor_count < 1: return
        q=msg.raw_sensor_orientation
        v=normalize([q.w,q.x,q.y,q.z])
        if v is not None:
            self.imu=v; self.imu_received=time.monotonic()


class WristAnchor:
    def __init__(self,model,data):
        self.model=model; self.data=data; self.reference=None
        self.root=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rh_forearm')
        self.wrist=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rh_wrist')
        if min(self.root,self.wrist)<0: raise ValueError('Shadow wrist bodies missing')
        self.local=model.body_pos[self.wrist].copy()
        mujoco.mj_forward(model,data)
        self.pivot=data.xpos[self.wrist].copy()
        palm=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rh_palm')
        if palm < 0: raise ValueError('Shadow palm body missing')
        root_R=data.xmat[self.root].reshape(3,3).copy()
        palm_R=data.xmat[palm].reshape(3,3).copy()
        root_to_palm=root_R.T @ palm_R
        # Palm surface faces down (-Z), fingers point away from operator (-Y).
        desired_palm=np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
        root_target=desired_palm @ root_to_palm.T
        q=np.empty(4); mujoco.mju_mat2Quat(q,root_target.reshape(9))
        self.base_q=normalize(q)
        # Measured on 2026-11-03 from separate held neutral, roll and yaw poses.
        # Use orthonormal bases: sensor roll -> world -Y; sensor right turn -> world -Z (user verified direction).
        a=np.array([0.89841592,-0.18371496,0.39887045]); a/=np.linalg.norm(a)
        b=np.array([0.31392334,0.01335658,-0.94935438]); b-=a*np.dot(a,b); b/=np.linalg.norm(b)
        src=np.column_stack((a,b,np.cross(a,b)))
        dst=np.column_stack(([0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]))
        self.sensor_to_sim=dst @ src.T
        model.body_quat[self.root]=self.base_q
        model.body_pos[self.root]=self.pivot-root_target @ self.local
        mujoco.mj_forward(model,data)
    def recenter(self,q):
        self.reference=normalize(q)
        if self.reference is None: raise ValueError('invalid IMU')
    def apply(self,q):
        q=normalize(q)
        if q is None or self.reference is None: return
        relative=mul(inv(self.reference),q)
        relative_mat=np.empty(9); mujoco.mju_quat2Mat(relative_mat,relative)
        r=self.sensor_to_sim @ relative_mat.reshape(3,3) @ self.sensor_to_sim.T
        delta=np.empty(4); mujoco.mju_mat2Quat(delta,r.reshape(9))
        target=normalize(mul(delta,self.base_q))
        self.model.body_quat[self.root]=target
        r=np.empty(9); mujoco.mju_quat2Mat(r,target)
        self.model.body_pos[self.root]=self.pivot-r.reshape(3,3)@self.local


def self_test():
    m=mujoco.MjModel.from_xml_path(str(SCENE)); d=mujoco.MjData(m); a=WristAnchor(m,d)
    a.recenter([1,0,0,0]); a.apply([1,0,0,0]); mujoco.mj_forward(m,d)
    assert np.linalg.norm(d.xpos[a.wrist]-a.pivot)<1e-7
    assert np.allclose(a.sensor_to_sim.T @ a.sensor_to_sim, np.eye(3),atol=1e-7)
    rot=[math.cos(math.pi/8),0,0,math.sin(math.pi/8)]; a.apply(rot); mujoco.mj_forward(m,d)
    assert np.linalg.norm(d.xpos[a.wrist]-a.pivot)<1e-7
    assert not np.allclose(m.body_quat[a.root],a.base_q)
    assert normalize([0,0,0,0]) is None
    print('PASS: 45-degree synthetic IMU rotation keeps wrist origin fixed; invalid quaternion rejected.')


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--topic',default=os.environ.get('MANUS_RIGHT_TOPIC', '/manus/right/glove_ID')); ap.add_argument('--self-test',action='store_true'); args=ap.parse_args()
    if args.self_test: self_test(); return
    rclpy.init(); glove=ImuGlove(args.topic)
    model=mujoco.MjModel.from_xml_path(str(SCENE)); data=mujoco.MjData(model)
    ctrl=Retarget(model,data); anchor=WristAnchor(model,data)
    imu_active=True
    def fresh(t): return t is not None and time.monotonic()-t<0.5
    def key(k):
        nonlocal imu_active
        if k==32: ctrl.enabled=False; imu_active=False; print('全部暂停，按 R 恢复旋转。',flush=True)
        elif k in (ord('r'),ord('R')):
            if fresh(glove.imu_received): anchor.recenter(glove.imu); imu_active=True; print('手腕方向参考已记录，旋转已启用。',flush=True)
            else: print('没有有效 IMU 数据，无法设置参考。',flush=True)
        elif k in (ord('o'),ord('O'),ord('f'),ord('F')):
            if fresh(glove.received): ctrl.capture('open' if k in (ord('o'),ord('O')) else 'closed',glove.data)
            else: print('未收到有效手指数据。',flush=True)
        elif k in (ord('e'),ord('E')):
            if fresh(glove.received) and ctrl.valid():
                ctrl.enabled=True; print('五指已启用。',flush=True)
            else: print('请检查 IMU 和手指数据，依次按 R、O、F，再按 E。',flush=True)
        elif k in (ord('c'),ord('C')):
            ctrl.open=ctrl.closed=None; ctrl.enabled=False; anchor.reference=None; print('标定清除。',flush=True)
    print('仿真专用：IMU 接收后自动启用，保持参考姿态按 R 可重新归零；五指张手按 O、握拳按 F、E 启用。空格暂停。',flush=True)
    print('固定手腕位置，只显示 IMU 相对朝向；并非手腕关节或空间位移测量。',flush=True)
    try:
        with mujoco.viewer.launch_passive(model,data,key_callback=key) as viewer:
            viewer.cam.lookat[:] = anchor.pivot + np.array([0.0, -0.10, 0.0])
            viewer.cam.azimuth = 90; viewer.cam.elevation = -75; viewer.cam.distance = 0.7
            last_sync=time.monotonic()
            while viewer.is_running() and rclpy.ok():
                start=time.monotonic(); rclpy.spin_once(glove,timeout_sec=0.0)
                if ctrl.enabled and (not fresh(glove.received) or not fresh(glove.imu_received)):
                    ctrl.enabled=False; print('ROS 数据超时，暂停；恢复后按 E。',flush=True)
                with viewer.lock():
                    if imu_active and fresh(glove.imu_received):
                        if anchor.reference is None:
                            anchor.recenter(glove.imu); print('已自动记录手腕参考方向，旋转开始。',flush=True)
                        anchor.apply(glove.imu)
                    if ctrl.enabled:
                        ctrl.apply(glove.data,model.opt.timestep)
                    mujoco.mj_step(model,data)
                if start-last_sync>=1/30: viewer.sync(); last_sync=start
                time.sleep(max(0.,model.opt.timestep-(time.monotonic()-start)))
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        try: glove.destroy_node(); rclpy.try_shutdown()
        except KeyboardInterrupt: pass

if __name__=='__main__': main()
