#include <ros/ros.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "steering_data/SteerCtl.h"

class SteeringFeedback {

private:
    ros::Subscriber sub_target;
    ros::Timer timer;

    // rosparam
    std::string DEVICE_NAME = "/dev/input/event20";
    double DEVICE_TIME_STEP = 0.1;
    double DEVICE_TOLERANCE = 0.01;
    double DEVICE_MAX_TORQUE = 0.5;
    double DEVICE_MIN_TORQUE = 0.3;
    
    // device info
    int device_handle;
    int axis_code = ABS_X;
    int axis_min;
    int axis_max;

    double pAct;

    double pDes; // Desired Position
    double kDes; // Desired Spring Rate
    
    double pErr; 
    double tCorr; // Corrective Torque
    double atk_len;
    
    struct ff_effect ffEffect;

public:
    SteeringFeedback();
    ~SteeringFeedback();

private:
    void initDevice();
    void steerFeedback(const steering_data::SteerCtl &msg);
    void loop(const ros::TimerEvent&);
       
    void getTorque(const double &current_position);    
    void setTorque(const double &torque, const double &attack_length);
    
    // Helpers
    int testBit(int bit, unsigned char *array);
    double limit(const double &a,const double &a_min,const double &a_max);
};


SteeringFeedback::SteeringFeedback() {
    ros::NodeHandle ros_node;

    std::string ns = ros_node.getNamespace();
    
    ros_node.getParam("deviceName", DEVICE_NAME);
    ros_node.getParam("timeStep", DEVICE_TIME_STEP);
    ros_node.getParam("tolerance", DEVICE_TOLERANCE);

    ros_node.getParam("maxTorque", DEVICE_MAX_TORQUE);
    ros_node.getParam("minTorque", DEVICE_MIN_TORQUE);    
    
    pAct = 0;
    pDes = 0;
    kDes = 1;
    
    pErr = 0;
    tCorr = 0;
    atk_len = 0;

    initDevice();

    ros::Duration(1).sleep();

    timer = ros_node.createTimer(ros::Duration(DEVICE_TIME_STEP), &SteeringFeedback::loop, this);
    
    sub_target = ros_node.subscribe(ns + "steer_feeback", 1, &SteeringFeedback::steerFeedback, this);
}

SteeringFeedback::~SteeringFeedback() {
    ffEffect.type = FF_CONSTANT;
    ffEffect.id = -1;
    ffEffect.u.constant.level = 0;
    ffEffect.direction = 0;

    // upload ffEffect
    if (ioctl(device_handle, EVIOCSFF, &ffEffect) < 0) {
        std::cout << "Failed to upload ffEffect" << std::endl;
    }
}

// initialize force feedback device
void SteeringFeedback::initDevice() {
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];
    struct input_event event;
    struct input_absinfo abs_info;

    device_handle = open(DEVICE_NAME.c_str(), O_RDWR|O_NONBLOCK);
    if (device_handle < 0) {
        std::cout << "ERROR: cannot open device : "<< DEVICE_NAME << std::endl;
        exit(1);
    }
    
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }
    
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        exit(1);
    }
    
    if (ioctl(device_handle, EVIOCGABS(axis_code), &abs_info) < 0) {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        exit(1);
    }

    axis_max = abs_info.maximum;
    axis_min = abs_info.minimum;
    if (axis_min >= axis_max) {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }

    if(!testBit(FF_CONSTANT, ff_bits)) {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);

    }
    
    std::cout << "Device Ready" << std::endl;

    // Turn off the default auto centering.
    memset(&event, 0, sizeof(event));

    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;

    if (write(device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "Failed to disable autocentering" << std::endl;
        exit(1);
    }

    // init effect and get effect id
    memset(&ffEffect, 0, sizeof(ffEffect));

    ffEffect.type = FF_CONSTANT;
    ffEffect.id = -1; // initial value
    ffEffect.trigger.button = 0;
    ffEffect.trigger.interval = 0;
    ffEffect.replay.length = 0xffff;  // longest value
    ffEffect.replay.delay = 0; // delay from write(...)
    ffEffect.u.constant.level = 0;
    ffEffect.direction = 0xC000;
    ffEffect.u.constant.envelope.attack_length = 0;
    ffEffect.u.constant.envelope.attack_level = 0;
    ffEffect.u.constant.envelope.fade_length = 0;
    ffEffect.u.constant.envelope.fade_level = 0;

    if (ioctl(device_handle, EVIOCSFF, &ffEffect) < 0) {
        std::cout << "Failed to upload ffEffect" << std::endl;
        exit(1);
    }

    // start ffEffect
    memset(&event, 0, sizeof(event));

    event.type = EV_FF;
    event.code = ffEffect.id;
    event.value = 1;
    
    if (write(device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "Failed to start event" << std::endl;
        exit(1);
    }
}

// get target information of wheel control from ros message
void SteeringFeedback::steerFeedback(const steering_data::SteerCtl &msg) {
    if (pDes != msg.position || kDes != fabs(msg.springrate)){
        pDes = limit(msg.position, -1.0, 1.0);
        kDes = fabs(msg.springrate);
    }
}

// update input event with timer callback
void SteeringFeedback::loop(const ros::TimerEvent&) {
    struct input_event event;
    
    // get current state
    while (read(device_handle, &event, sizeof(event)) == sizeof(event)) {
        if (event.type == EV_ABS && event.code == axis_code) {
            pAct = (event.value - (axis_max + axis_min) * 0.5) * 2 / (axis_max - axis_min);
        }
    }

    getTorque(pAct);    
    setTorque(tCorr, atk_len);    
}

void SteeringFeedback::getTorque(const double &pAct) {    
    double pErr = pDes - pAct;

    if (fabs(pErr) < DEVICE_TOLERANCE) {
        tCorr = 0.0;
        atk_len = 0.0;
    } else {
        double mul = (pErr >= 0.0) ? 1.0 : -1.0;

        tCorr =  mul * limit(fabs(kDes * pErr), DEVICE_MIN_TORQUE, DEVICE_MAX_TORQUE);
        atk_len = DEVICE_TIME_STEP;
    }

    std::cout<< pErr << ", " << tCorr << std::endl;
}

// update input event with writing information to the event file
void SteeringFeedback::setTorque(const double &torque, const double &attack_length) {
    // set effect
    ffEffect.u.constant.level = 0x7fff * torque;
    ffEffect.direction = 0xC000;    
    ffEffect.u.constant.envelope.attack_length = attack_length;    
    ffEffect.u.constant.envelope.fade_length = attack_length;
    
    // ffEffect.u.constant.envelope.fade_level = 0;
    // ffEffect.u.constant.envelope.attack_level = 0; /* 0x7fff * force / 2 */

    // upload effect
    if (ioctl(device_handle, EVIOCSFF, &ffEffect) < 0) {
        std::cout << "Failed to upload effect" << std::endl;
    }
}

// util for initDevice()
int SteeringFeedback::testBit(int bit, unsigned char *array) {
    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}

double SteeringFeedback::limit(const double &a, const double &a_min, const double &a_max){
    return(std::max(std::min(a, a_max), a_min));
}

int main(int argc, char **argv ){
    ros::init(argc, argv, "steering_feedback");
    SteeringFeedback sNode;
    ros::spin();
    return(0);
}