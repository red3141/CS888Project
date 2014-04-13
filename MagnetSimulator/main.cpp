#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "gluvi.h"
#include "gl/glu.h"
#include "SphereMagnet.h"
#include "MeshMagnetElement.h"
#include "BMPWriter.h"

using namespace std;

string frame_number="Frame 0";
unsigned int frame= 0;

//Sim parameters
double timestep = 0.001;
double coeff_restitution = 0.85;
Vec3d constant_acceleration(0, -9.81, 0); //gravity

bool filming = false;
bool running = false;
bool drawingInduction = false;

char * bmpfileformat;

std::vector<SphereMagnet*> particles;
std::vector<std::vector<MeshMagnetElement> > immobileMagnets;

// Handle collisions using penalty/repulsion forces
void handle_collisions() {
	const double SPRING_CONSTANT = 10.0;
	const double DAMPING_CONSTANT = 1000.0;

	// Testing if two spheres are colliding is pretty simple; just
	// check if the distance between the centres of the two spheres
	// is less than the sum of their radii.
	for(unsigned int i = 0; i < particles.size(); ++i) {
		for(unsigned int j = i + 1; j < particles.size(); ++j) {
			SphereMagnet* magnet1 = particles[i];
			SphereMagnet* magnet2 = particles[j];
			Vec3d displacement = magnet1->getPosition() - magnet2->getPosition();
			double distance = mag(displacement);
			double radiusSum = magnet1->getRadius() + magnet2->getRadius();
			double overlapDistance = radiusSum - distance;

			if(overlapDistance > 0) {
				// Apply repulsion force as a spring force, as described on
				// slide SH10 of the Collision and Contact section of the
				// Pixar course notes.

				Vec3d normal = normalized(magnet1->getPosition() - magnet2->getPosition());
				Vec3d fa = DAMPING_CONSTANT * overlapDistance * normal;
				Vec3d fb = -fa;

				magnet1->setLinearMomentum(magnet1->getLinearMomentum() + fa);
				magnet2->setLinearMomentum(magnet2->getLinearMomentum() + fb);
			}
		}
	}
}


// Compute the magnetic induction, as described in section 3.1 of the Thomaszewski et al. paper.
Vec3d computeMagneticInduction(Vec3d position) {
	Vec3d result(0.0, 0.0, 0.0);

	for(unsigned int p = 0; p < particles.size(); ++p) {
		Vec3d m = particles[p]->getMagneticMoment();
		Vec3d distance = position - particles[p]->getPosition();
		Vec3d n = normalized(distance);
		normalize(n);

		// mu_0 = 4 * pi * 10 ^ -7 (V*s)/(A*m), so mu_0/(4*pi) = 10 ^ -7 (V*s)/(A*m)
		Vec3d b = 0.0000001 * (3.0 * n * dot(n, m) - m) / mag(distance);
		result += b;
	}

	for(unsigned int p = 0; p < immobileMagnets.size(); ++p) {
		for(unsigned int e = 0; e < immobileMagnets[p].size(); ++e) {
			Vec3d m = immobileMagnets[p][e].getMagneticMoment();
			Vec3d distance = position -immobileMagnets[p][e].getPosition();
			Vec3d n = normalized(distance);
			normalize(n);

			// mu_0 = 4 * pi * 10 ^ -7 (V*s)/(A*m), so mu_0/(4*pi) = 10 ^ -7 (V*s)/(A*m)
			Vec3d b = 0.0000001 * (3.0 * n * dot(n, m) - m) / mag(distance);
			result += b;
		}
	}

	return result;
}


void drawMagneticInductionLines() {
	// The idea here is to draw the magnetic field lines starting at 18 points on the surface of each magnet.
	// The 18 points are the north and south magnetic poles and 9 points around the poles, halfway between the poles
	// and the equators. These points are found by finding two vectors perpendicular to the magnetization direction
	// of the magnets, and then combining these vectors with the magnetization direction.
	// For the points on the north side of a magnet, the magnetic field is followed in a forward direction, while for
	// the points on the south side of a magnet, the magentic field is followed in a backward direction. This is because
	// the magnetic field goes from the north sides of the magnets to the south sides.

	for(unsigned int i = 0; i < particles.size(); ++i) {
		vector<Vec3d> startingPoints = particles[i]->getMagneticInductionStartPoints();
		
		for(unsigned int j = 0; j < startingPoints.size(); ++j) {
			Vec3d position = startingPoints[j];
			for(int k = 0; k < 1000; ++k) {
				Vec3d induction = computeMagneticInduction(position);
				Vec3d newPosition = position + induction;
				glVertex3d(position[0], position[1], position[2]);
				glVertex3d(newPosition[0], newPosition[1], newPosition[2]);
				position = newPosition;
			}

			position = startingPoints[j];
			for(int k = 0; k < 1000; ++k) {
				Vec3d induction = computeMagneticInduction(position);
				Vec3d newPosition = position - induction;
				glVertex3d(position[0], position[1], position[2]);
				glVertex3d(newPosition[0], newPosition[1], newPosition[2]);
				position = newPosition;
			}
		}
	}

	// TODO: this code is almost identical to the above code; clean it up.
	for(unsigned int i = 0; i < immobileMagnets.size(); ++i) {
		for(unsigned int j = 0; j < immobileMagnets[i].size(); ++j) {
			vector<Vec3d> startingPoints = immobileMagnets[i][j].getMagneticInductionStartPoints();

			for(unsigned int j = 0; j < startingPoints.size(); ++j) {
				Vec3d position = startingPoints[j];
				for(int k = 0; k < 1000; ++k) {
					Vec3d induction = computeMagneticInduction(position);
					Vec3d newPosition = position + induction;
					glVertex3d(position[0], position[1], position[2]);
					glVertex3d(newPosition[0], newPosition[1], newPosition[2]);
					position = newPosition;
				}

				position = startingPoints[j];
				for(int k = 0; k < 1000; ++k) {
					Vec3d induction = computeMagneticInduction(position);
					Vec3d newPosition = position - induction;
					glVertex3d(position[0], position[1], position[2]);
					glVertex3d(newPosition[0], newPosition[1], newPosition[2]);
					position = newPosition;
				}
			}
		}
	}
}


void advance_sim() {
   
   //Update the frame label
   frame++;
   std::ostringstream strout;
   strout.str("");
   strout << "Frame " << frame;
   frame_number = strout.str();

   //time-integrate the particles
   for(unsigned int i = 0; i < particles.size(); ++i) {
      Vec3d position = particles[i]->getPosition();
	  Vec3d linearMomentum = particles[i]->getLinearMomentum();
	  Matrix33d rotation = particles[i]->getRotation();
	  Vec3d angularMomentum = particles[i]->getAngularMomentum();
	  double mass = particles[i]->getMass();

      //Forward Euler time integration on position 
	  position += timestep * (linearMomentum / mass);

	  // Determine the change in rotation for this magnet, as described on pages G4-G15 of the
	  // Rigid Body Dynamics chapter of the Pixar notes.
	  Matrix33d inertiaTensorInverse = matrixMult(
		  matrixMult(rotation, particles[i]->getIBodyInverse()),
		  transpose(rotation));

	  Vec3d angularVelocity = matrixVectorMult(inertiaTensorInverse, particles[i]->getAngularMomentum());

	  Matrix33d deltaRotation;

	  for(int j = 0; j < 3; ++j) {
		  Vec3d column;
		  for(int k = 0; k < 3; ++k) {
			column[k] = rotation[k][j];
		  }
		  column = cross(angularVelocity, column);
		  for(int k = 0; k < 3; ++k) {
			deltaRotation[k][j] = column[k];
		  }
	  }

	  Matrix33d newRotation = rotation + scalarMatrixMult(timestep, deltaRotation);
	  // Normalize the rows of the rotation matrix to prevent the matrix from blowing up due to
	  // numerical error.
	  for(int j = 0; j < 3; ++j) {
		normalize(newRotation[j]);
	  }

      //Forward Euler time integration on linear momentum
	  // Gravity applies evenly over the entire object, so there is no torque resulting from gravity.
	  //linearMomentum += timestep * constant_acceleration * mass;
   
      //process simple impulsed-based, frictionless collisions
      
      //floor
	  if(position[1] < 0 && linearMomentum[1] < 0) {
		 linearMomentum[1] *= -coeff_restitution;

		 // As an example, apply a force opposing motion when the magnet hits the ground.
		 /*Vec3d force(-linearMomentum[0], 0, -linearMomentum[2]);
		 linearMomentum += timestep * force;
		 Vec3d locationOfForceApplication = position + Vec3d(0, -particles[i]->getRadius() , 0);
		 Vec3d torque = cross((locationOfForceApplication - position), force);
		 angularMomentum += timestep * torque;*/
	  }

      //other walls
	  if(position[0] < 0 && linearMomentum[0] < 0 || position[0] > 1 && linearMomentum[0] > 0) {
		 linearMomentum[0] *= -coeff_restitution;
	  }
      if(position[2] < 0 && linearMomentum[2] < 0 || position[2] > 1 && linearMomentum[2] > 0) {
		 linearMomentum[2] *= -coeff_restitution;
	  }

      //save back the results
	  particles[i]->setPosition(position);
	  particles[i]->setLinearMomentum(linearMomentum);
	  particles[i]->setRotation(newRotation);
	  particles[i]->setAngularMomentum(angularMomentum);
   }

	handle_collisions();

	// Determine the magnetic forces and torques being applied to this magnet by the other magnets, as
	// described in Section 3.2 of the Thomaszewski et al. paper. Currently, each magnet is treated as
	// a single element; for more accurate results, the magnets should be treated as many small pieces
	// of magnets.
	for(unsigned int i = 0; i < particles.size(); ++i) {

		// Determine the force applied on this magnet by the other moveable magnets.
		for(unsigned int j = 0; j < particles.size(); ++j) {
			if(i == j)
				continue;

			Vec3d displacement = particles[i]->getPosition() - particles[j]->getPosition();
			// Since repulsion forces are used to push colliding magnets apart, it is possible that the magnets
			// could interpenetrate, resulting in extremely large magnetic forces. By always calculating the
			// magnetic forces as if the magnets are at least as far apart as they would be if they were in
			// contact, these extremely large, incorrect forces can be avoided.
			double distance = max(mag(displacement), particles[i]->getRadius() + particles[j]->getRadius());
			double distanceSquared = distance * distance;
			Vec3d n = normalized(displacement);
			Vec3d magnetizationI = particles[i]->getMagneticMoment();
			Vec3d magnetizationJ = particles[j]->getMagneticMoment();

			// mu_0 = 4 * pi * 10 ^ -7 (V*s)/(A*m), so mu_0/(4*pi) = 10 ^ -7 (V*s)/(A*m)
			Vec3d force = 0.0000001 * (1.0 / (distanceSquared * distanceSquared)) *
				( (-15.0 * n * dot(magnetizationI, n) * dot(magnetizationJ, n)) +
				3.0 * n * dot(magnetizationI, magnetizationJ) +
				3.0 * (magnetizationI * dot(magnetizationJ, n) + magnetizationJ * dot(magnetizationI, n)));

			Vec3d torque = 0.0000001 * (3.0 / (distanceSquared * distance)) *
				(cross(magnetizationI, n) * dot(magnetizationJ, n) - cross(magnetizationI, magnetizationJ));

			particles[i]->setLinearMomentum(particles[i]->getLinearMomentum() + timestep * force);
			particles[i]->setAngularMomentum(particles[i]->getAngularMomentum() + timestep * torque);
		}

		// Determine the force applied on this magnet by the the immoveable magnets.
		for(unsigned int j = 0; j < immobileMagnets.size(); ++j) {
			for(unsigned int e = 0; e < immobileMagnets[j].size(); ++e) {
				Vec3d displacement = particles[i]->getPosition() - immobileMagnets[j][e].getPosition();
				double distance = mag(displacement);
				double distanceSquared = distance * distance;
				Vec3d n = normalized(displacement);
				Vec3d magnetizationI = particles[i]->getMagneticMoment();
				Vec3d magnetizationJ = immobileMagnets[j][e].getMagneticMoment();

				// mu_0 = 4 * pi * 10 ^ -7 (V*s)/(A*m), so mu_0/(4*pi) = 10 ^ -7 (V*s)/(A*m)
				Vec3d force = 0.0000001 * (1.0 / (distanceSquared * distanceSquared)) *
					( (-15.0 * n * dot(magnetizationI, n) * dot(magnetizationJ, n)) +
					3.0 * n * dot(magnetizationI, magnetizationJ) +
					3.0 * (magnetizationI * dot(magnetizationJ, n) + magnetizationJ * dot(magnetizationI, n)));

				Vec3d torque = 0.0000001 * (3.0 / (distanceSquared * distance)) *
					(cross(magnetizationI, n) * dot(magnetizationJ, n) - cross(magnetizationI, magnetizationJ));

				particles[i]->setLinearMomentum(particles[i]->getLinearMomentum() + timestep * force);
				particles[i]->setAngularMomentum(particles[i]->getAngularMomentum() + timestep * torque);
			}
		}
	}
}

void createRingMagnet() {
	std::vector<MeshMagnetElement> ringMagnet;
	
	double innerRadius = 0.3;
	double outerRadius = 0.4;
	double z1 = 0.45;
	double z2 = 0.55;

	double angle = M_PI / 4.0;
	double deltaAngle = M_PI / 8.0;

	int numberOfElements = 12;

	for(int i = 0; i < numberOfElements; ++i) {
		double xOuter1 = outerRadius * sin(angle) + 0.5;
		double xInner1 = innerRadius * sin(angle) + 0.5;
		double xOuter2 = outerRadius * sin(angle + deltaAngle) + 0.5;
		double xInner2 = innerRadius * sin(angle + deltaAngle) + 0.5;
		double yOuter1 = outerRadius * -cos(angle) + 0.5;
		double yInner1 = innerRadius * -cos(angle) + 0.5;
		double yOuter2 = outerRadius * -cos(angle + deltaAngle) + 0.5;
		double yInner2 = innerRadius * -cos(angle + deltaAngle) + 0.5;

		Vec3d north(xOuter2 - xOuter1, yOuter2 - yOuter1, 0.0);
		Vec3d position((xOuter1 + xOuter2 + xInner1 + xInner2) / 4.0, (yOuter1 + yOuter2 + yInner1 + yInner2) / 4.0, (z1 + z2) / 2.0);
		Vec3f colour(0.7f, 0.7f, 0.7f);

		if(i == 0) {
			colour[0] = colour[1] = 0.3f;
			colour[2] = 0.6f;
		} else if(i == numberOfElements - 1) {
			colour[0] = 0.6f;
			colour[1] = colour[2] = 0.3f;
		}

		MeshMagnetElement e(300.0, north, position, colour);

		std::vector<Vec3d> face;

		if(i == 0) {
			face.push_back(Vec3d(xOuter1, yOuter1, z1));
			face.push_back(Vec3d(xOuter1, yOuter1, z2));
			face.push_back(Vec3d(xInner1, yInner1, z2));
			face.push_back(Vec3d(xInner1, yInner1, z1));
			e.addFace(face);
			face.clear();
		}

		face.push_back(Vec3d(xInner2, yInner2, z1));
		face.push_back(Vec3d(xOuter2, yOuter2, z1));
		face.push_back(Vec3d(xOuter1, yOuter1, z1));
		face.push_back(Vec3d(xInner1, yInner1, z1));
		e.addFace(face);
		face.clear();
		face.push_back(Vec3d(xOuter2, yOuter2, z1));
		face.push_back(Vec3d(xOuter2, yOuter2, z2));
		face.push_back(Vec3d(xOuter1, yOuter1, z2));
		face.push_back(Vec3d(xOuter1, yOuter1, z1));
		e.addFace(face);
		face.clear();
		face.push_back(Vec3d(xOuter2, yOuter2, z2));
		face.push_back(Vec3d(xInner2, yInner2, z2));
		face.push_back(Vec3d(xInner1, yInner1, z2));
		face.push_back(Vec3d(xOuter1, yOuter1, z2));
		e.addFace(face);
		face.clear();
		face.push_back(Vec3d(xInner2, yInner2, z2));
		face.push_back(Vec3d(xInner2, yInner2, z1));
		face.push_back(Vec3d(xInner1, yInner1, z1));
		face.push_back(Vec3d(xInner1, yInner1, z2));
		e.addFace(face);
		face.clear();

		if(i == numberOfElements - 1) {
			face.push_back(Vec3d(xOuter2, yOuter2, z1));
			face.push_back(Vec3d(xOuter2, yOuter2, z2));
			face.push_back(Vec3d(xInner2, yInner2, z2));
			face.push_back(Vec3d(xInner2, yInner2, z1));
			e.addFace(face);
			face.clear();
		}

		ringMagnet.push_back(e);

		angle += deltaAngle;
	}

	immobileMagnets.push_back(ringMagnet);
}


void set_view(Gluvi::Target3D &cam)
{
   //setup initial view point
   cam.target[0] = 0.5;
   cam.target[1] = 0.5;
   cam.target[2] = 0.5;
   cam.heading = 0.5;
   cam.pitch = -0.5;
   cam.dist = 3;
}

void set_lights(int object)
{
   glEnable(GL_LIGHTING);
   GLfloat global_ambient[4] = {0.1f, 0.1f, 0.1f, 1.0f};
   glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
   glShadeModel(GL_SMOOTH);

   //Light #1
   GLfloat color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat position[4] = {0.0f, 1.0f, 0.0f, 0.0f};
   glLightfv(GL_LIGHT0, GL_SPECULAR, color);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, color);
   glLightfv(GL_LIGHT0, GL_POSITION, position);

   //Light #2
   GLfloat color2[4] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat position2[4] = {0.0f, 0.0f, 1.0f, 0.0f};
   glLightfv(GL_LIGHT1, GL_SPECULAR, color2);
   glLightfv(GL_LIGHT1, GL_DIFFUSE, color2);
   glLightfv(GL_LIGHT1, GL_POSITION, position2);

   glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);
}

void timer(int value)
{
   if(filming) {
		char* bmpfile = new char[255];
		sprintf(bmpfile, bmpfileformat, frame);
		BMPWriter writer(bmpfile);

		for(int y = 0; y < 480; ++y) {
			for(int x = 0; x < 720; ++x) {
				unsigned char pixel[4];
				glReadPixels(x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel);
				writer.writePixel(pixel[0], pixel[1], pixel[2]);
			}
		}
		delete[] bmpfile;
   }
    
   if(filming || running) {
      advance_sim();
      glutTimerFunc(10, timer, 0);
      glutPostRedisplay();
   }

}


void display(void)
{
   glClearColor(0.6f, 0.7f, 0.9f, 1);

   glEnable(GL_LIGHTING);
   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

   set_lights(1); 

   //Draw the particles as simple spheres.
   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
   for(unsigned int p = 0; p < particles.size(); ++p) {
      glPushMatrix();
	  SphereMagnet* particle = particles[p];
	  particle->draw();
      glPopMatrix();   
   }

   for(unsigned int p = 0; p < immobileMagnets.size(); ++p) {
	   for(unsigned int e = 0; e < immobileMagnets[p].size(); ++e) {
		   glPushMatrix();
		   immobileMagnets[p][e].draw();
		   glPopMatrix();
	   }
   }
 
   //Draw bounding box wireframe
   glDisable(GL_LIGHTING);
   glColor3f(0,0,0);
   glBegin(GL_LINES);
   glVertex3f(0,0,0);
   glVertex3f(0,0,1);

   glVertex3f(0,0,0);
   glVertex3f(0,1,0);

   glVertex3f(0,0,0);
   glVertex3f(1,0,0);

   glVertex3f(0,1,0);
   glVertex3f(1,1,0);

   glVertex3f(1,1,0);
   glVertex3f(1,0,0);

   glVertex3f(1,0,0);
   glVertex3f(1,0,1);

   glVertex3f(0,1,0);
   glVertex3f(0,1,1);

   glVertex3f(1,1,0);
   glVertex3f(1,1,1);

   glVertex3f(0,1,1);
   glVertex3f(1,1,1);

   glVertex3f(1,0,1);
   glVertex3f(1,1,1);

   glVertex3f(0,0,1);
   glVertex3f(1,0,1);

   glVertex3f(0,0,1);
   glVertex3f(0,1,1);

   if(drawingInduction) {
      drawMagneticInductionLines();
   }

   glEnd();
 
}


void toggle_filming() {
   if(!running) {
      if(!filming) {
         filming = true;
         glutTimerFunc(10, timer, 0);
      }
      else {
         filming = false;
      }
   }
}

void toggle_running() {
   if(!filming) {
      if(!running) {
         running = true;
         glutTimerFunc(10, timer, 0);
      }
      else {
         running = false;
      }
   }
}

struct MovieButton : public Gluvi::Button{
   const char *filename_format;
   MovieButton(const char *label, const char *filename_format_) : Gluvi::Button(label), filename_format(filename_format_) {}
   void action()
   { 
      toggle_filming();
   }
};

struct RunButton : public Gluvi::Button{
   RunButton(const char *label) : Gluvi::Button(label){}
   void action()
   { 
      toggle_running();
   }
};


void keyPress(unsigned char key, int x, int y) {

   if(key == 'r') {
      toggle_running();
   }
   else if(key == 'f') {
      toggle_filming();
   }
   else if(key == 'l') {
      drawingInduction = !drawingInduction;
   }
   glutPostRedisplay();


}



Gluvi::Target3D* cam_local;

int main(int argc, char **argv)
{
   Gluvi::init("Particle Animation", &argc, argv);

   glutKeyboardFunc(keyPress);

   Gluvi::Target3D cam;
   set_view(cam);
   Gluvi::camera=&cam;
   
   Gluvi::userDisplayFunc=display;

   Gluvi::StaticText frametext(frame_number.c_str());
   Gluvi::root.list.push_back(&frametext);

   bmpfileformat = new char[255];
   sprintf(bmpfileformat, "screenshot%%04d.bmp");
   printf("%s\n", bmpfileformat);

   MovieButton movie("(f)ilm", bmpfileformat);
   Gluvi::root.list.push_back(&movie);

   RunButton run("(r)un");
   Gluvi::root.list.push_back(&run);

   Gluvi::StaticText instructiontext("Shift + Drag with Mouse Buttons to adjust view.");
   Gluvi::root.list.push_back(&instructiontext);


   //set up initial particle set
   int seed = 0;
   double mass = 1.0;
   double magnetStrength = 300.0;
   /*for(int i = 0; i < 3; ++i) {
      Vec3d position;
      position[0] = randhashf(++seed,0,1);
      position[1] = randhashf(++seed,0,1);
      position[2] = randhashf(++seed,0,1);
	  //position[0] = 0.0 + (double)i * 0.1;
	  //position[1] = 0.0 + (double)i * 0.1;
	  //position[2] = 0.0 + (double)i * 0.1;
	  Vec3d linearMomentum;
	  //linearMomentum[0] = mass * randhashf(++seed,-5,5);
      //linearMomentum[1] = mass * randhashf(++seed,-5,5);
      //linearMomentum[2] = mass * randhashf(++seed,-5,5);
	  linearMomentum[0] = 0.0;
      linearMomentum[1] = 0.0;
      linearMomentum[2] = 0.0;

	  particles.push_back(new SphereMagnet(position, linearMomentum, mass, magnetStrength));
   }*/

   // Three attracting magnets
   /*particles.push_back(new SphereMagnet(Vec3d(0.1, 0.5, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(1.0, 0.0, 0.0)));
   particles.push_back(new SphereMagnet(Vec3d(0.5, 0.5, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(1.0, 0.0, 0.0)));
   particles.push_back(new SphereMagnet(Vec3d(0.9, 0.5, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(1.0, 0.0, 0.0)));*/
   
   // Three repulsing magnets
   /*particles.push_back(new SphereMagnet(Vec3d(0.3, 0.5, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(1.0, 0.0, 0.0)));
   particles.push_back(new SphereMagnet(Vec3d(0.7, 0.51, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(-1.0, 0.0, 0.0)));
   particles.push_back(new SphereMagnet(Vec3d(0.9, 0.5, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(1.0, 0.0, 0.0)));*/

   // A demonstration of two magnets quickly orienting themselves, and then moving towards each other.
	particles.push_back(new SphereMagnet(Vec3d(0.2, 0.3, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(0.0, 1.0, 0.0)));
	particles.push_back(new SphereMagnet(Vec3d(0.8, 0.7, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(0.0, 1.0, 0.0)));

   // A demonstration of a more complicated magnet.
   // Set up the immobile magnet.
   /*createRingMagnet();
   particles.push_back(new SphereMagnet(Vec3d(0.5, 0.1, 0.5), Vec3d(0.0, 0.0, 0.0), mass, magnetStrength, Vec3d(1.0, 0.0, 0.0)));*/

   Gluvi::run();
   return 0;
}
