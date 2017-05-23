#include "FlightControl.h"

FlightControl::FlightControl(Flight *flight, data_t maxV, data_t maxYaw, data_t ts)
	: flight(flight),
	  xController("../config/kx", maxV, ts),
	  yController("../config/ky", maxV, ts),
	  zController("../config/kz", maxV, ts),
	  yawController("../config/kyaw", maxYaw, ts),
	  logger("../log/u") {}

void FlightControl::control(data_t ex, data_t ey, data_t ez, data_t eyaw) {
	data_t ux, uy, uz, uyaw;
	ux = xController(ex);
	uy = yController(ey);
	uz = zController(ez);
	uyaw = yawController(eyaw);
	logger("%.2f,%.2f,%.2f,%.2f", ux, uy, uz, uyaw);
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
