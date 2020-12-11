
struct traj{
	double x;
	double y;
	double th;
	double d;
	double alpha;
	traj(){
		x=0;
		y=0;
		th=0;
		d=0;
		alpha=0;
	}
	traj(double _x, double _y, double _th, double _d, double _alpha){
		x=_x;
		y=_y;
		th=_th;
		d=_d;
		alpha=_alpha;
	}
};
