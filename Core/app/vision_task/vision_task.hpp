#pragma once

#include "vision_data.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void VisionTaskStart();

#ifdef __cplusplus
}
#endif

void VisionSetMode(vision::AimMode mode, vision::EnemyColor color,
                   float bullet_speed);
void VisionSetAimingLock(bool lock);
bool VisionIsOnline();
