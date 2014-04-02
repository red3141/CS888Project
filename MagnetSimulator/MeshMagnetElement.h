#include "gluvi.h"
#include "gl/glu.h"

class MeshMagnetElement {
	// Faces are defined by a vector of vertices.
	std::vector<std::vector<Vec3d>> faces;
	double magnetStrength;
	Vec3d north;
	Vec3d position;
	Vec3f colour;

public:
	MeshMagnetElement(double magnetStrength, Vec3d north, Vec3d position, Vec3f colour);
	~MeshMagnetElement();
	Vec3d getMagneticMoment();
	Vec3d getNorth() {return north;}
	Vec3d getPosition() {return position;}
	void draw();
	void addFace(std::vector<Vec3d> newFace) {faces.push_back(newFace);}
	std::vector<Vec3d> getMagneticInductionStartPoints();
};