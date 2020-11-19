
struct point{
	double x;
	double y;
	double th;

	bool operator==(point a) {
		return x == a.x && y == a.y && th == a.th;
	}

	bool operator!=(point a) {
		return !(*this == a);
	}
};
