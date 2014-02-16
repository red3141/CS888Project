#include "gluvi.h"
#include "gl/glu.h"

class SphereMagnet {
	Vec3d position;
	Vec3d velocity;
	double radius;
	GLUquadric* particle_sphere;

public:
	SphereMagnet(Vec3d position, Vec3d velocity);
	~SphereMagnet();
	Vec3d getPosition() {return position;}
	Vec3d getVelocity() {return velocity;}
	void setPosition(Vec3d newPosition) {position = newPosition;}
	void setVelocity(Vec3d newVelocity) {velocity = newVelocity;}
	void draw();
};