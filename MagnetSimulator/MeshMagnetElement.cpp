#include <iostream>

#include "MeshMagnetElement.h"

using namespace std;

MeshMagnetElement::MeshMagnetElement(double magnetStrength, Vec3d north, Vec3f colour) {
	this->magnetStrength = magnetStrength;
	this->north = normalized(north);
	this->colour = colour;
}

MeshMagnetElement::~MeshMagnetElement() {
}

void MeshMagnetElement::draw() {
	GLfloat elementColour[4] = {colour[0], colour[1], colour[2]};
	glMaterialfv (GL_FRONT, GL_AMBIENT, elementColour);
	glMaterialfv (GL_FRONT, GL_DIFFUSE, elementColour);
	glMaterialf (GL_FRONT, GL_SHININESS, 32);
	glMaterialfv (GL_FRONT, GL_SPECULAR, elementColour);

	for(unsigned int i = 0; i < this->faces.size(); ++i) {
		vector<Vec3d> face = this->faces[i];
		if(face.size() == 3) {
			glBegin(GL_TRIANGLES);
		} else if(face.size() == 4) {
			glBegin(GL_QUADS);
		}

		for(unsigned int j = 0; j < face.size(); ++j) {
			glVertex3d(face[j][0], face[j][1], face[j][2]);
		}

		glEnd();
	}
	glFlush();
}