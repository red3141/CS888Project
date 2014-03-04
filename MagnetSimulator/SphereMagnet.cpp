#include <iostream>

#include "SphereMagnet.h"

using namespace std;

SphereMagnet::SphereMagnet(Vec3d position, Vec3d linearMomentum, double mass) {
	this->position = position;
	this->linearMomentum = linearMomentum;
	this->radius = 0.05;
	this->mass = mass;
	this->particle_sphere = gluNewQuadric();
}

SphereMagnet::~SphereMagnet() {
}

void SphereMagnet::draw() {
	gluQuadricDrawStyle(particle_sphere, GLU_FILL);
	gluSphere(particle_sphere, radius, 20, 20);
}