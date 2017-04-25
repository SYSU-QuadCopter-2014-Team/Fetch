/*! @brief
 *  外部数据的获取接口
 * */

#ifndef LINK_H
#define LINK_H

#include <DJI_Type.h>
#include <DJICommonType.h>
#include <DJI_API.h>
#include <DJI_Flight.h>
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "LinuxFlight.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

DJI::Vector3dData getTargetOffset();

#endif // LINK_H
