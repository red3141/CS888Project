#include <iostream>

#include "MeshMagnetElement.h"

using namespace std;

MeshMagnetElement::MeshMagnetElement(double magnetStrength, Vec3d north, Vec3d position, Vec3f colour) {
	this->magnetStrength = magnetStrength;
	this->north = normalized(north);
	this->position = position;
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

		Vec3d normal = normalized(cross(face[1] - face[0], face[2] - face[1]));
		glNormal3d(normal[0], normal[1], normal[2]);
		for(unsigned int j = 0; j < face.size(); ++j) {
			glVertex3d(face[j][0], face[j][1], face[j][2]);
		}

		glEnd();
	}
	glFlush();
}

Vec3d MeshMagnetElement::getMagneticMoment() {
	return magnetStrength * normalized(this->north);
}

std::vector<Vec3d> MeshMagnetElement::getMagneticInductionStartPoints() {
	std::vector<Vec3d> startPoints;

	for(unsigned int i = 0; i < faces.size(); ++i) {
		Vec3d centreOfFace(0.0, 0.0, 0.0);

		for(unsigned int j = 0; j < faces[i].size(); ++j) {
			centreOfFace += faces[i][j];
		}

		centreOfFace *= (1.0 / faces[i].size());
		startPoints.push_back(centreOfFace);

		// Draw magnetic induction lines starting from every point halfway between the centre of a face
		// and a vertex of the face.
		/*for(unsigned int j = 0; j < faces[i].size(); ++j) {
			startPoints.push_back(0.5 * (centreOfFace + faces[i][j]));
		}*/
	}

	return startPoints;
}