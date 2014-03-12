#include <iostream>

#include "SphereMagnet.h"

using namespace std;

SphereMagnet::SphereMagnet(Vec3d position, Vec3d linearMomentum, double mass) {
	this->position = position;
	this->linearMomentum = linearMomentum;
	makeIdentityMatrix(this->rotation);
	this->angularMomentum[0] = this->angularMomentum[1] =
		this->angularMomentum[2] = 0.0;
	this->radius = 0.05;
	this->mass = mass;
	this->particle_sphere = gluNewQuadric();

	// Compute I_{body}^{-1}
	double diagonalEntry = 2.5 / (mass * radius * radius);
	for(int i = 0; i < 3; ++i) {
		for(int j = 0; j < 3; ++j) {
			this->iBodyInverse[i][j] = ((i == j) ? diagonalEntry : 0);
		}
	}
}

SphereMagnet::~SphereMagnet() {
}

Vec3d SphereMagnet::getMagneticMomentDirection() {
	Vec3d north(0);
	north[1] = 1;
	north = matrixVectorMult(rotation, north);
	normalize(north);
	return north;
}

void SphereMagnet::draw() {
	// Draw the south magnetic pole.
	GLfloat obj_colorSouth[4] = {.3f, .3f, .6f};
	glMaterialfv (GL_FRONT, GL_AMBIENT, obj_colorSouth);
	glMaterialfv (GL_FRONT, GL_DIFFUSE, obj_colorSouth);

	GLfloat specularSouth[4] = {.2f, .2f, .2f};
	glMaterialf (GL_FRONT, GL_SHININESS, 32);
	glMaterialfv (GL_FRONT, GL_SPECULAR, specularSouth);

    glTranslated(position[0], position[1], position[2]);
	gluQuadricDrawStyle(particle_sphere, GLU_FILL);
	gluSphere(particle_sphere, radius, 20, 20);

	// Draw the north magnetic pole.
	GLfloat obj_colorNorth[4] = {.6f, .3f, .3f};
	glMaterialfv (GL_FRONT, GL_AMBIENT, obj_colorNorth);
	glMaterialfv (GL_FRONT, GL_DIFFUSE, obj_colorNorth);

	GLfloat specularNorth[4] = {.2f, .2f, .2f};
	glMaterialf (GL_FRONT, GL_SHININESS, 32);
	glMaterialfv (GL_FRONT, GL_SPECULAR, specularNorth);

	Vec3d north = getMagneticMomentDirection();
	north *= 0.002;
	glTranslated(north[0], north[1], north[2]);
	gluSphere(particle_sphere, radius, 20, 20);
}