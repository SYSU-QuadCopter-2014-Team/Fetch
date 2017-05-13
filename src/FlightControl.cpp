#include "FlightControl.h"

// todo 设置PID参数
FlightControl::FlightControl(Flight *flight, data_t maxV, data_t maxYaw, data_t ts)
	: flight(flight),
	  xController(1.0f, 0.1f, 0.01f, maxV, ts),
	  yController(1.0f, 0.1f, 0.01f, maxV, ts),
	  zController(0.7f, 0.1f, 0, maxV, ts),
	  yawController(1, 0, 0, maxYaw, ts),
	  logger("u.txt") {}

void FlightControl::control(data_t ex, data_t ey, data_t ez, data_t eyaw) {
	data_t ux, uy, uz, uyaw;
	ux = xController(ex);
	uy = yController(ey);
	uz = zController(ez);
	uyaw = yawController(eyaw);
	logger("%6.2f %6.2f %6.2f %6.2f", ux, uy, uz, uyaw);
	send_control(flag, ux, uy, uz, uyaw);
}

void FlightControl::stop() {
	send_control(flag, 0, 0, 0, 0);
}

void FlightControl::send_control(uint8_t flag, float x, float y, float z, float yaw)
{
	FlightData fd;
	fd.flag = flag;
	fd.x = x;
	fd.y = y;
	fd.z = z;
	fd.yaw = yaw;
	flight->setFlight(&fd);
}
