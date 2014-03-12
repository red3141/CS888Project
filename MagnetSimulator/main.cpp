#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "gluvi.h"
#include "gl/glu.h"
#include "SphereMagnet.h"

using namespace std;

string frame_number="Frame 0";
unsigned int frame= 0;

//Sim parameters
double timestep = 0.01;
double coeff_restitution = 0.85;
int num_particles = 2;
Vec3d constant_acceleration(0, -9.81, 0); //gravity

bool filming = false;
bool running = false;

char * sgifileformat;

std::vector<SphereMagnet*> particles;
Vec<11, Vec<11, Vec<11, Vec3d>>> magneticInduction;

// Handle collisions using penalty/repulsion forces
void handle_collisions() {
	const double SPRING_CONSTANT = 0.5;
	const double DAMPING_CONSTANT = 0.85;

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

			if(distance < radiusSum) {
				// Apply repulsion force as a spring force, as described on
				// page C7 of the Particle System Dynamics chapter of the
				// Pixar course notes.
				Vec3d velocityDifference = magnet1->getVelocity() - magnet2->getVelocity();
				Vec3d fa = -(SPRING_CONSTANT * (distance - radiusSum) +
					DAMPING_CONSTANT * (dot(velocityDifference, displacement)) / distance) *
					(displacement / distance);
				Vec3d fb = -fa;
				Vec3d acceleration1 = fa / magnet1->getMass();
				Vec3d acceleration2 = fb / magnet2->getMass();

				magnet1->setLinearMomentum(magnet1->getLinearMomentum() + fa);
				magnet2->setLinearMomentum(magnet2->getLinearMomentum() + fb);
			}
		}
	}
}


// Compute the magnetic induction, as described in section 3.1 of the Thomaszewski et al. paper.
void computeMagneticInduction() {
	for(int i = 0; i < 11; ++i) {
		for(int j = 0; j < 11; ++j) {
			for(int k = 0; k < 11; ++k) {
				magneticInduction[i][j][k] = Vec3d(0);
				Vec3d position(0.1 * i, 0.1 * j, 0.1 * k);

				for(unsigned int p = 0; p < particles.size(); ++p) {
					Vec3d m = 30.0 * particles[p]->getMagneticMomentDirection(); // TODO: the 30.0 here can be replaced by a magnet strength per magnet
					Vec3d distance = position - particles[p]->getPosition();
					Vec3d n = normalized(distance);
					normalize(n);

					// mu_0 = 4 * pi * 10 ^ -7 (V*s)/(A*m), so mu_0/(4*pi) = 10 ^ -7 (V*s)/(A*m)
					Vec3d b = 0.0000001 * (3.0 * n * dot(n, m) - m) / mag(distance);
					magneticInduction[i][j][k] += b;
				}
			}
		}
	}
}


void drawMagneticInductionLines() {
	for(int i = 0; i < 11; ++i) {
		for(int j = 0; j < 11; ++j) {
			for(int k = 0; k < 11; ++k) {
				Vec3d position(0.1 * i, 0.1 * j, 0.1 * k);
				Vec3d vectorEnds = position + 3000.0 * magneticInduction[i][j][k]; // TODO: the 3000.0 here is kind of arbitrary
				glVertex3d(position[0], position[1], position[2]);
				glVertex3d(vectorEnds[0], vectorEnds[1], vectorEnds[2]);
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
	  linearMomentum += timestep * constant_acceleration * mass;
   
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

   computeMagneticInduction();
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
   GLfloat position2[4] = {1.0f, 0.0f, 0.0f, 0.0f};
   glLightfv(GL_LIGHT1, GL_SPECULAR, color2);
   glLightfv(GL_LIGHT1, GL_DIFFUSE, color2);
   glLightfv(GL_LIGHT1, GL_POSITION, position2);

   glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);
}

void timer(int value)
{
   if(filming) {
      Gluvi::sgi_screenshot(sgifileformat, frame);
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
   glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
   for(unsigned int p = 0; p < particles.size(); ++p) {
      glPushMatrix();
	  SphereMagnet* particle = particles[p];
	  particle->draw();
      glPopMatrix();   
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

   drawMagneticInductionLines();

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

   sgifileformat = new char[255];
   sprintf(sgifileformat, "screenshot%%04d.sgi");
   printf("%s\n", sgifileformat);

   MovieButton movie("(f)ilm", sgifileformat);
   Gluvi::root.list.push_back(&movie);

   RunButton run("(r)un");
   Gluvi::root.list.push_back(&run);

   Gluvi::StaticText instructiontext("Shift + Drag with Mouse Buttons to adjust view.");
   Gluvi::root.list.push_back(&instructiontext);


   //set up initial particle set
   int seed = 0;
   double mass = 1.0;
   for(int i = 0; i < num_particles; ++i) {
      Vec3d position;
      position[0] = randhashf(++seed,0,1);
      position[1] = randhashf(++seed,0,1);
      position[2] = randhashf(++seed,0,1);
	  Vec3d linearMomentum;
	  linearMomentum[0] = mass * randhashf(++seed,-5,5);
      linearMomentum[1] = mass * randhashf(++seed,-5,5);
      linearMomentum[2] = mass * randhashf(++seed,-5,5);

	  particles.push_back(new SphereMagnet(position, linearMomentum, mass));
   }

   Gluvi::run();
   return 0;
}
