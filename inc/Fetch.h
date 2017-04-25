/*! @brief
 *  抓取的各步骤功能函数
 * */

#ifndef FETCH_H
#define FETCH_H

#include <DJI_Type.h>
#include <DJICommonType.h>
#include <DJI_API.h>
#include <DJI_Flight.h>
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "LinuxFlight.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

/*! 执行“飞至目标物体上方”任务。
 *
 *  @param timeoutInMs 超时时间（毫秒）
 *  @param posThresholdInCm 可容许的坐标偏差（厘米）
 *
 *  @return 错误码：0 代表成功，1 代表超时。
 * */
int flyToTarget(CoreAPI* api, Flight* flight, int timeoutInMs, float posThresholdInCm);

#endif // FETCH_H
