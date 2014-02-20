#include <iostream>

#include "SphereMagnet.h"

using namespace std;

SphereMagnet::SphereMagnet(Vec3d position, Vec3d velocity) {
	this->position = position;
	this->velocity = velocity;
	this->radius = 0.05;
	this->mass = 1;
	this->particle_sphere = gluNewQuadric();
}

SphereMagnet::~SphereMagnet() {
}

void SphereMagnet::draw() {
	gluQuadricDrawStyle(particle_sphere, GLU_FILL);
	gluSphere(particle_sphere, radius, 20, 20);
}