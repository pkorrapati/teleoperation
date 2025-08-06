#!/usr/bin/env python3

import rospy
from dataclasses import dataclass

from evdev import InputDevice, ecodes, ff
from steering.msg import SteerCtl

@dataclass
class DEVICE:
    NAME = "/dev/input/event14"
    LOOP_RATE = 0.1    
    TOLERANCE = 0.01
    MAX_TORQUE = 1.0
    MIN_TORQUE = 0.2
    
# class SteerCtl:
#     position = 0
#     springrate = 0

# Returns a value between lLim and uLim
def limit(value, lLim, uLim):
    return min(max(lLim, value), uLim)

class SteeringFeedback:
    def __init__(self):
        global DEVICE
        
        self.ns = rospy.get_namespace()        
        rospy.init_node("steering_feedback")

        DEVICE.NAME = rospy.get_param("~device_name", DEVICE.NAME)
        DEVICE.LOOP_RATE = rospy.get_param("~rate", DEVICE.LOOP_RATE)        
        DEVICE.TOLERANCE = rospy.get_param("~tolerance", DEVICE.TOLERANCE)

        DEVICE.MIN_TORQUE = rospy.get_param("~min_torque", DEVICE.MIN_TORQUE)
        DEVICE.MAX_TORQUE = rospy.get_param("~max_torque", DEVICE.MAX_TORQUE)

        self.p_des = 0 # Desired Positon 
        self.p_err = 0               

        self.k_des = 1 # Spring rate
        self.t_corr = 0 # Correcting Torque
        
        self.device = InputDevice(DEVICE.NAME)        
        self.axis_code = ecodes.ABS_X

        axis_info = self.device.absinfo(self.axis_code)
        
        self.axis_min = axis_info.min
        self.axis_max = axis_info.max

        self.device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)

        # Create and upload force feedback effect
        self.effect = ff.Effect(
            ecodes.FF_CONSTANT,
            -1,
            ff.Trigger(0, 0),
            ff.Replay(0xffff, 0),
            ff.Constant(level=0, envelope=ff.Envelope(0, 0, 0, 0)),
            direction=0xC000
        )

        self.effect_id = self.device.upload_effect(self.effect)
        self.device.write(ecodes.EV_FF, self.effect_id, 1)

        rospy.Subscriber(self.ns + "steer_feeback", SteerCtl, self.steer_feedback)
        rospy.Timer(rospy.Duration(DEVICE.LOOP_RATE), self.loop)
    
    def steer_feedback(self, msg):        
        if not self.p_des == msg.position or self.k_des == msg.springrate:                    
            self.p_des = msg.position
            self.k_des = abs(msg.springrate)                        
        
    def loop(self, _):
        p_raw = self.device.absinfo(self.axis_code)
        p_act = (2 * (p_raw.value - (self.axis_max + self.axis_min) * 0.5) / (self.axis_max - self.axis_min))

        self.p_err, t_corr, attack_length = self.getTorque(p_act, self.p_des, self.k_des, self.p_err)

        self.setTorque(t_corr, attack_length)

    def getTorque(self, p_des, p_act, k, p_err_old):
        p_err = p_des - p_act
        dp_err = p_err - p_err_old
        
        mul = 1 if p_err >= 0 else -1       

        t_corr = 0
        attack_length = 0

        if abs(p_err) > DEVICE.TOLERANCE:
            t_corr = mul * limit(k * abs(p_err), DEVICE.MIN_TORQUE, DEVICE.MAX_TORQUE)
            attack_length = DEVICE.LOOP_RATE
        
        return p_err, t_corr, attack_length

    def setTorque(self, t_corr, attack_length):
        torque_level = int(0x7FFF * t_corr)
        self.effect.u.constant.level = torque_level
        self.effect.u.constant.envelope.attack_length = int(attack_length * 1000)
        self.effect.u.constant.envelope.fade_length = int(attack_length * 1000)

        try:
            self.device.erase_effect(self.effect_id)
            self.effect_id = self.device.upload_effect(self.effect)
            self.device.write(ecodes.EV_FF, self.effect_id, 1)
        except Exception as e:
            rospy.logerr(f"Failed to upload force effect: {e}")

if __name__ == "__main__":
    try:
        sNode = SteeringFeedback()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
