#pragma once

namespace referee { class RefereeParser; }

#ifdef __cplusplus
extern "C" {
#endif

void RefereeInit();
void VideoLinkInit();

#ifdef __cplusplus
}
#endif

bool RefereeIsOnline();

/// 获取常规链路 RefereeParser 指针 (用于 UI 发送等)
/// @note 云台板/底盘板未编译常规链路时返回 nullptr
referee::RefereeParser* GetRefereeParser();
