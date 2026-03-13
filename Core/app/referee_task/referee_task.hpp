#pragma once

#include "referee_def.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void RefereeTaskStart();
void VideoLinkTaskStart();

#ifdef __cplusplus
}
#endif

bool RefereeIsOnline();
const referee::RefereeData& GetRefereeData();
