#include <iostream>

#include "SphereMagnet.h"

using namespace std;

SphereMagnet::SphereMagnet(Vec3d position, Vec3d linearMomentum, double mass, double magnetStrength, Vec3d north) {
	this->position = position;
	this->linearMomentum = linearMomentum;
	makeIdentityMatrix(this->rotation);
	this->angularMomentum[0] = this->angularMomentum[1] =
		this->angularMomentum[2] = 0.0;
	this->radius = 0.05;
	this->mass = mass;
	this->magnetStrength = magnetStrength;
	this->north = normalized(north);
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

Vec3d SphereMagnet::getMagneticMoment() {
	return magnetStrength * normalized(matrixVectorMult(rotation, this->north));
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

	Vec3d north = normalized(getMagneticMoment());
	north *= 0.002;
	glTranslated(north[0], north[1], north[2]);
	gluSphere(particle_sphere, radius, 20, 20);
}

std::vector<Vec3d> SphereMagnet::getMagneticInductionStartPoints() {
		// Find 2 vectors perpendicular to the direction north is pointing.
		//Vec3d north = matrixVectorMult(this->rotation, this->north);

		Vec3d perp(0);
		if(north[0] == 0) {
			perp[1] = -north[2];
			perp[2] = north[1];
		} else if(north[1] == 0) {
			perp[0] = north[2];
			perp[2] = -north[0];
		} else {
			perp[0] = -north[1];
			perp[1] = north[0];
		}
		normalize(perp);
		Vec3d otherPerp = normalized(cross(north, perp));

		std::vector<Vec3d> startPoints;
		startPoints.push_back(position + radius * normalized(north));
		startPoints.push_back(position + radius * normalized(north + perp));
		startPoints.push_back(position + radius * normalized(north - perp));
		startPoints.push_back(position + radius * normalized(north + otherPerp));
		startPoints.push_back(position + radius * normalized(north - otherPerp));
		startPoints.push_back(position + radius * normalized(north + perp + otherPerp));
		startPoints.push_back(position + radius * normalized(north - perp + otherPerp));
		startPoints.push_back(position + radius * normalized(north + perp - otherPerp));
		startPoints.push_back(position + radius * normalized(north + perp - otherPerp));
		startPoints.push_back(position + radius * normalized(north - perp - otherPerp));
		startPoints.push_back(position + radius * normalized(-north));
		startPoints.push_back(position + radius * normalized(-north + perp));
		startPoints.push_back(position + radius * normalized(-north - perp));
		startPoints.push_back(position + radius * normalized(-north + otherPerp));
		startPoints.push_back(position + radius * normalized(-north - otherPerp));
		startPoints.push_back(position + radius * normalized(-north + perp + otherPerp));
		startPoints.push_back(position + radius * normalized(-north - perp + otherPerp));
		startPoints.push_back(position + radius * normalized(-north + perp - otherPerp));
		startPoints.push_back(position + radius * normalized(-north - perp - otherPerp));

		return startPoints;
}