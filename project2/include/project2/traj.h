
struct traj{
	double x;
	double y;
	double th;
	double d;
	double alpha;

	traj() {
		x = 0.0;
		y = 0.0;
		th = 0.0;
		d = 0.0;
		alpha = 0.0;
	}

	traj(const double& _x, const double& _y, const double& _th, const double& _d, const double& _alpha) {
		x = _x;
		y = _y;
		th = _th;
		d = _d;
		alpha = _alpha;
	}
};
