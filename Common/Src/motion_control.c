#include "motion_control.h"

Vec3 MotionControlPIDUpdateVel(PIDContext pidContext, Vec3 curVel, Vec3 targetVel)
{
    
}

void MotionControlUpdatePWM(
    AxisMap axisMap, Vec3 velMax, Vec3 velAngMax, Vec3 mmVelSteps, Vec3 mmAngVelSteps, void *devs, uint32_t nDevs
)
{
    int32_t forward = mmVelSteps.x * axisMap.forward.x + mmVelSteps.y * axisMap.forward.y + mmVelSteps.z * axisMap.forward.z;
    int32_t up = mmVelSteps.x * axisMap.up.x + mmVelSteps.y * axisMap.up.y + mmVelSteps.z * axisMap.up.z;
    int32_t right = mmVelSteps.x * axisMap.right.x + mmVelSteps.y * axisMap.right.y + mmVelSteps.z * axisMap.right.z;

    // Left wing tip is higher than right wing tip
    int32_t rollRight = mmAngVelSteps.x * axisMap.forward.x + mmAngVelSteps.y * axisMap.forward.y + mmAngVelSteps.z * axisMap.forward.z;
    // Left wing tip moves forward and right wing tip moves backwards
    int32_t yawRight = mmAngVelSteps.x * axisMap.up.x + mmAngVelSteps.y * axisMap.up.y + mmAngVelSteps.z * axisMap.up.z;
    // Nose moves up and tail moves down
    int32_t pitchUp = mmAngVelSteps.x * axisMap.right.x + mmAngVelSteps.y * axisMap.right.y + mmAngVelSteps.z * axisMap.right.z;
}