#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vecmath.h>
#include <vector>
#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class PendulumSystem: public ParticleSystem
{
public:
	PendulumSystem(int numParticles);
	void windToggle() {wind_ON = !wind_ON;};
	vector<Vector3f> evalF(vector<Vector3f> state);

	void draw();

private:
	vector<Vector4f> m_vSprings;
	bool wind_ON;
	static const float particle_mass;
	static const float drag_const_k;
	static const float spring_const_k;
	static const float spring_rest_len;
};

#endif
