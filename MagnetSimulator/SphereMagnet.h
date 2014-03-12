#include "gluvi.h"
#include "gl/glu.h"

class SphereMagnet {
	Vec3d position;
	Vec3d linearMomentum;
	Matrix33d rotation;
	Vec3d angularMomentum;
	double radius;
	double mass;
	Matrix33d iBodyInverse;
	GLUquadric* particle_sphere;

public:
	SphereMagnet(Vec3d position, Vec3d linearMomentum, double mass);
	~SphereMagnet();
	Vec3d getPosition() {return position;}
	Vec3d getVelocity() {return linearMomentum / mass;}
	Vec3d getLinearMomentum() {return linearMomentum;}
	Matrix33d getRotation() {return rotation;}
	Vec3d getAngularMomentum() {return angularMomentum;}
	double getRadius() {return radius;}
	double getMass() {return mass;}
	Vec3d getMagneticMomentDirection();
	Matrix33d getIBodyInverse() {return iBodyInverse;}
	void setPosition(Vec3d newPosition) {position = newPosition;}
	void setLinearMomentum(Vec3d newLinearMomentum) {linearMomentum = newLinearMomentum;}
	void setRotation(Matrix33d newRotation) {rotation = newRotation;}
	void setAngularMomentum(Vec3d newAngularMomentum) {angularMomentum = newAngularMomentum;}
	void draw();
};