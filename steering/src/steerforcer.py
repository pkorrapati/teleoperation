#!/usr/bin/env python3

import rospy
from dataclasses import dataclass

from evdev import InputDevice, ecodes, ff
from steering_data.msg import SteerCtl

@dataclass
class DEVICE:
    NAME = "/dev/input/event20"
    TIME_STEP = 0.1    
    TOLERANCE = 0.01
    MAX_TORQUE = 1.0 #2.5N-m
    MIN_TORQUE = 0.2

# Returns a value between lLim and uLim
def limit(value, lLim, uLim):
    return min(max(lLim, value), uLim)

class SteeringFeedback:
    def __init__(self):
        global DEVICE
        
        self.ns = rospy.get_namespace()        
        rospy.init_node("steering_feedback")

        DEVICE.NAME = rospy.get_param("~device_name", DEVICE.NAME)
        DEVICE.TIME_STEP = limit(rospy.get_param("~time_step", DEVICE.TIME_STEP), 0.001, 32.767) # uInt limits
        DEVICE.TOLERANCE = rospy.get_param("~tolerance", DEVICE.TOLERANCE)

        DEVICE.MIN_TORQUE = rospy.get_param("~min_torque", DEVICE.MIN_TORQUE)
        DEVICE.MAX_TORQUE = rospy.get_param("~max_torque", DEVICE.MAX_TORQUE)

        self.p_des = 0 # Desired Positon                    
        self.k_des = 1 # Spring rate
        
        self.t_corr = 0
        self.atk_len = 0
        
        self.device = InputDevice(DEVICE.NAME)        
        self.axis_code = ecodes.ABS_X

        axis_info = self.device.absinfo(self.axis_code)
        
        self.axis_min = axis_info.min
        self.axis_max = axis_info.max

        self.device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)

        # Create and upload force feedback effect
        self.effect = ff.Effect(
            type = ecodes.FF_CONSTANT,
            id = -1,
            direction = 0xC000,
            ff_replay = ff.Replay(0xffff, 0),
            u = ff.EffectType(ff_constant_effect=ff.Constant(level=0, envelope=ff.Envelope(0, 0, 0, 0)))
        )

        self.effect_id = self.device.upload_effect(self.effect)
        self.device.write(ecodes.EV_FF, self.effect_id, 1)

        rospy.Subscriber(self.ns + "steer_feeback", SteerCtl, self.steer_feedback)
        rospy.Timer(rospy.Duration(DEVICE.TIME_STEP), self.loop)
    
    def steer_feedback(self, msg):        
        if not self.p_des == msg.position or self.k_des == abs(msg.springrate):
            self.p_des = limit(msg.position, -1.0, 1.0)
            self.k_des = abs(msg.springrate)                        
        
    def loop(self, _):
        p_raw = self.device.absinfo(self.axis_code)
        p_act = (2 * (p_raw.value - (self.axis_max + self.axis_min) * 0.5) / (self.axis_max - self.axis_min))

        self.getTorque(p_act)
        self.setTorque(self.t_corr, self.atk_len)

    def getTorque(self, p_act):        
        p_err = self.p_des - p_act        
        
        self.t_corr = 0
        self.atk_len = 0

        if abs(p_err) > DEVICE.TOLERANCE:
            mul = 1 if p_err >= 0 else -1

            t_des = self.k_des * p_err

            self.t_corr = -1 * mul * limit(abs(t_des), DEVICE.MIN_TORQUE, DEVICE.MAX_TORQUE)
            self.atk_len = DEVICE.TIME_STEP / 10

    # effect id must always be -1 to be considered a new effect       
    # Range of valid values for level is -0x7FFF to +0x7FFF
    # Range of valid values for attack_level is 0x0000 to 0x7FFF 
    # Units for attack_length is milliseconds and values must lie between 0x0000 to 0x7FFF ms
    # Valid values for direction are 0x0000, 0x4000, 0x8000, 0xC000 => down, left, up, right
    # t_norm is in the range -1.0 to +1.0
    # atl_len is in the range 0 to 32.767
    def setTorque(self, t_norm, atk_len):
        # self.effect.direction = 0xC000
        self.effect.u.ff_constant_effect.level = int(0x7FFF * t_norm)
        # self.effect.u.ff_constant_effect.ff_envelope.attack_level = 0
        self.effect.u.ff_constant_effect.ff_envelope.attack_length = int(atk_len * 1000) #milliseconds
        # self.effect.u.ff_constant_effect.ff_envelope.fade_level = 0
        self.effect.u.ff_constant_effect.ff_envelope.fade_length = int(atk_len * 1000)   #milliseconds
        
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

# References
# https://www.kernel.org/doc/html/latest/input/ff.html
# https://github.com/torvalds/linux/blob/master/include/uapi/linux/input.h
# https://stackoverflow.com/questions/33201711/how-to-send-a-rumble-effect-to-a-device-using-python-evdev