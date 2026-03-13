#pragma once

#include "robot_def.hpp"
#include <cstdint>

#ifdef VOFA_ENABLED

void VofaTaskStart();
void VofaSetChannel(uint8_t ch, float val);

#else

inline void VofaTaskStart() {}
inline void VofaSetChannel(uint8_t, float) {}

#endif
