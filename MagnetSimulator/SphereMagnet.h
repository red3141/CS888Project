#include "gluvi.h"
#include "gl/glu.h"

class SphereMagnet {
	Vec3d position;
	Vec3d linearMomentum;
	double radius;
	double mass;
	GLUquadric* particle_sphere;

public:
	SphereMagnet(Vec3d position, Vec3d linearMomentum, double mass);
	~SphereMagnet();
	Vec3d getPosition() {return position;}
	Vec3d getVelocity() {return linearMomentum / mass;}
	Vec3d getLinearMomentum() {return linearMomentum;}
	double getRadius() {return radius;}
	double getMass() {return mass;}
	void setPosition(Vec3d newPosition) {position = newPosition;}
	void setLinearMomentum(Vec3d newLinearMomentum) {linearMomentum = newLinearMomentum;}
	void draw();
};