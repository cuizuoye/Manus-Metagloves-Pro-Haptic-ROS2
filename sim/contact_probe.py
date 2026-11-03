#!/usr/bin/env python3
"""Read MuJoCo finger/object contacts only; never sends device commands."""
from shadow_teleop import SCENE
import mujoco
import numpy as np

FINGERS = ('thumb','index','middle','ring','pinky')
PREFIXES = ('rh_th','rh_ff','rh_mf','rh_rf','rh_lf')

def contacts(model, data, object_name='object'):
    """Return summed nonnegative normal force by finger and contact pair count.

    Only contacts between the named object and a finger body count; self-contact,
    palm, floor and other objects are deliberately excluded.
    """
    object_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,object_name)
    if object_id<0: raise ValueError('Object not present: '+object_name)
    sums=np.zeros(5); count=np.zeros(5,dtype=int); vec=np.zeros(6)
    for i in range(data.ncon):
        c=data.contact[i]
        b1=int(model.geom_bodyid[c.geom1]); b2=int(model.geom_bodyid[c.geom2])
        if b1==object_id: other=b2
        elif b2==object_id: other=b1
        else: continue
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,other) or ''
        for finger,prefix in enumerate(PREFIXES):
            if name.startswith(prefix):
                mujoco.mj_contactForce(model,data,i,vec)
                sums[finger]+=max(0.,float(vec[0])); count[finger]+=1
                break
    return sums, count


def self_test():
    model=mujoco.MjModel.from_xml_path(str(SCENE)); data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    baseline, base_count=contacts(model,data)
    assert not np.any(base_count), f'Unexpected baseline contact: {base_count}'
    # For an intentionally synthetic geometry-overlap test, move the free object
    # to a known index distal collision geom, then step physics. Not a real grasp.
    tip=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'rh_ffdistal')
    geom=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==tip and model.geom_contype[i])
    obj=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'object')
    joint=int(model.body_jntadr[obj]); q=int(model.jnt_qposadr[joint])
    data.qpos[q:q+3]=data.geom_xpos[geom]
    mujoco.mj_forward(model,data)
    strength, count=contacts(model,data)
    print('baseline counts:',base_count.tolist())
    print('synthetic overlapping object contacts:',dict(zip(FINGERS,count.tolist())))
    assert count[1]>0, 'Index-object contact not detected'
    mujoco.mj_step(model,data)
    strength, count=contacts(model,data)
    print('normal forces after step:',dict(zip(FINGERS,np.round(strength,3).tolist())))
    assert np.all(np.isfinite(strength)) and np.all(strength>=0)
    print('PASS: contact classification only; no haptic commands or hardware access.')

if __name__=='__main__': self_test()
