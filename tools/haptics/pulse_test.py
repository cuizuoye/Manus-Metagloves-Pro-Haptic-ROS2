#!/usr/bin/env python3
"""Single short vibration test. Default is offline self-test, not hardware output."""
import argparse
import math
import time

MAX_AMPLITUDE=0.12
MAX_DURATION=0.25
FINGERS=('thumb','index','middle','ring','pinky')
SIDES=('left','right')


def sequence(send, wait, finger, amplitude, duration):
    if finger not in FINGERS or not (0 < amplitude <= MAX_AMPLITUDE) or not (0 < duration <= MAX_DURATION):
        raise ValueError('Invalid finger, amplitude, or duration')
    vector=[0.0]*5; vector[FINGERS.index(finger)]=float(amplitude)
    send([0.0]*5)
    try:
        send(vector)
        wait(duration)
    finally:
        send([0.0]*5)


def self_test():
    out=[]
    sequence(out.append,lambda _:None,'index',0.12,0.25)
    assert out==[[0.0]*5,[0.0,0.12,0.0,0.0,0.0],[0.0]*5]
    out=[]
    try:
        sequence(out.append,lambda _:(_ for _ in ()).throw(RuntimeError('simulated interruption')),'index',0.12,0.25)
    except RuntimeError: pass
    assert out[-1]==[0.0]*5
    print('PASS: offline output order is zero, one finger, zero; interruption requests zero. No ROS or hardware output.')


def run(side, finger, base):
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from manus_ros2_msgs.msg import ManusGlove, ManusVibrationCommand
    rclpy.init(); node=Node('manus_single_pulse_test')
    last=[None]
    def receive(msg):
        if msg.side.lower()==side and (msg.glove_id & 0xffffffff) == int(base.rsplit('_',1)[-1],16):
            last[0]=time.monotonic()
    node.create_subscription(ManusGlove,base,receive,10)
    pub=node.create_publisher(ManusVibrationCommand,base+'/vibration_cmd',10)
    until=time.monotonic()+5
    while time.monotonic()<until and rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
        if pub.get_subscription_count()==1 and last[0] is not None and time.monotonic()-last[0]<0.5: break
    if pub.get_subscription_count()!=1 or last[0] is None or time.monotonic()-last[0]>=0.5:
        print('未发现唯一的振动订阅者和近期同侧手套消息；没有发送振动。')
        node.destroy_node(); rclpy.try_shutdown(); return 2
    print(f'目标: {side} {base}; 手指: {finger}; 强度 {MAX_AMPLITUDE}; {MAX_DURATION} 秒。',flush=True)
    results=[]
    def send(values):
        msg=ManusVibrationCommand(); msg.intensities=values
        pub.publish(msg)
        ok=pub.wait_for_all_acked(Duration(seconds=0.4))
        results.append((list(values),bool(ok)))
        if not ok: raise RuntimeError('ROS subscriber did not acknowledge the message')
    try:
        sequence(send,time.sleep,finger,MAX_AMPLITUDE,MAX_DURATION)
    except (KeyboardInterrupt,RuntimeError) as e:
        print('测试中断：',e,flush=True)
    finally:
        # Additional zeros improve retry opportunity, but cannot guarantee delivery after failures.
        for _ in range(2):
            try: send([0.0]*5)
            except Exception as e: print('清零发送异常:',e,flush=True)
            time.sleep(0.1)
        print('ROS 层确认情况:',results,flush=True)
        print('ROS 确认只说明订阅者收到了消息，不能证明手套振动或停止；请由佩戴者确认。',flush=True)
        try: node.destroy_node(); rclpy.try_shutdown()
        except KeyboardInterrupt: pass
    return 0 if results and all(ok for _,ok in results) else 1


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--side',choices=SIDES,default='right');ap.add_argument('--topic',help='Exact glove topic ending in glove_HEXID');ap.add_argument('--finger',choices=FINGERS,default='index');args=ap.parse_args()
    if not args.execute: self_test(); return
    if not args.topic or not args.topic.rsplit('/',1)[-1].startswith('glove_'):
        ap.error('--execute requires --topic ending in glove_HEXID')
    try: int(args.topic.rsplit('_',1)[-1],16)
    except ValueError: ap.error('topic must end in a hexadecimal glove ID')
    raise SystemExit(run(args.side,args.finger,args.topic))

if __name__=='__main__': main()
