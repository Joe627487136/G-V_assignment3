#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vecmath.h>
#include <vector>
#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class ClothSystem: public ParticleSystem
{
	///ADD MORE FUNCTION AND FIELDS HERE
public:
	ClothSystem(int numParticles);
	vector<Vector3f> evalF(vector<Vector3f> state);
	void Move_button();
	void draw();
	string Move_String;

private:

	static const float particle_mass;
	static const float gap_distance;
	static const float drag_k;
	static const float shear_spring_k;
	static const float shear_spring_rest_len;
	static const float flex_spring_k;
	static const float flex_spring_rest_len;
	static const float structural_spring_k;
	static const float structural_spring_rest_len;
	static bool Move_bool;
	Vector3f CalculateFxt(vector<Vector3f>& state, int i, int j);
	int indexOf(int i, int j);


};


#endif
