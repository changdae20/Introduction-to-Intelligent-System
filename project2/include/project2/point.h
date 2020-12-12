
struct point{
	double x;
	double y;
	double th;

	point() {
		x = 0.0;
		y = 0.0;
		th = 0.0;
	}

	point(const double& _x, const double& _y, const double& _th) {
		x = _x;
		y = _y;
		th = _th;
	}

	bool operator==(point a) {
		return x == a.x && y == a.y && th == a.th;
	}

	bool operator!=(point a) {
		return !(*this == a);
	}
};
