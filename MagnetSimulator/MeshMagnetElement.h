#include "gluvi.h"
#include "gl/glu.h"

class MeshMagnetElement {
	std::vector<std::vector<Vec3d>> faces;
	double magnetStrength;
	Vec3d north;
	Vec3f colour;

public:
	MeshMagnetElement(double magnetStrength, Vec3d north, Vec3f colour);
	~MeshMagnetElement();
	Vec3d getMagneticMoment();
	Vec3d getNorth() {return north;}
	void draw();
	void addFace(std::vector<Vec3d> newFace) {faces.push_back(newFace);}
};