#include "ClothSystem.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <string>

//TODO: Initialize here
const float ClothSystem::particle_mass = 0.2;
const float ClothSystem::gap_distance = 0.3;
const float ClothSystem::shear_spring_k = 40.0;
const float ClothSystem::shear_spring_rest_len = gap_distance*sqrt(2.0);
const float ClothSystem::flex_spring_k = 40.0;
const float ClothSystem::flex_spring_rest_len = gap_distance*2;
const float ClothSystem::structural_spring_k = 40.0;
const float ClothSystem::structural_spring_rest_len = gap_distance;
const float ClothSystem::drag_k = 0.2;
bool ClothSystem::Move_bool;

int ClothSystem::indexOf(int i, int j) {
	return (m_numParticles*i+j)*2;
}

ClothSystem::ClothSystem(int numParticles)
{
	m_numParticles = numParticles;
	vector<Vector3f> my_state;

	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) {
		for	(int j = 0; j < m_numParticles; j++){
			Vector3f even_indice = gap_distance*Vector3f(i, 1.f, j);
			Vector3f odd_indice = Vector3f(0.f,0.f,0.f);
			my_state.push_back(even_indice);
			my_state.push_back(odd_indice);
		}
	}

	Move_bool = false;
	Move_String = "Forward";
	setState(my_state);
}

Vector3f ClothSystem::CalculateFxt(vector<Vector3f>& state, int i, int j){
	Vector3f x = state[indexOf(i,j)];
	Vector3f gravity_force = particle_mass*Vector3f(0.f,-1.f,0.f);
	Vector3f drag_force = -drag_k*state[indexOf(i,j)+1];

	Vector3f distance;
	// STRUCTURAL SPRING FORCE
	Vector3f structural_spring_force = Vector3f(0.0,0.0,0.0);
	// force of left spring (except for first column)
	if (i > 0) { 
		distance = x - state[indexOf(i-1, j)];
		structural_spring_force += -structural_spring_k * (distance.abs() - structural_spring_rest_len) * distance.normalized();
	}
	// force of spring above (except for first row)
	if (j > 0) { 
		distance = x - state[indexOf(i, j-1)];
		structural_spring_force += -structural_spring_k * (distance.abs() - structural_spring_rest_len) * distance.normalized();
	}
	// force of right spring (except for last column)
	if (i < m_numParticles-1) { 
		distance = x - state[indexOf(i+1, j)];
		structural_spring_force += -structural_spring_k * (distance.abs() - structural_spring_rest_len) * distance.normalized();
	}
	// force of spring below (except for last row)
	if (j < m_numParticles-1) {
		distance = x - state[indexOf(i, j+1)];
		structural_spring_force += -structural_spring_k * (distance.abs() - structural_spring_rest_len) * distance.normalized();
	}

	// SHEAR SPRING FORCE
	Vector3f shear_spring_force = Vector3f(0.0,0.0,0.0);
	if (i > 0) {
		// force of northwest spring
		if (j > 0) {
			distance = x - state[indexOf(i-1, j-1)];		
			shear_spring_force += -shear_spring_k * (distance.abs() - shear_spring_rest_len) * distance.normalized();
		}
		// force of southwest spring
		if (j < m_numParticles-1) {
			distance = x - state[indexOf(i-1, j+1)];
			shear_spring_force += -shear_spring_k * (distance.abs() - shear_spring_rest_len) * distance.normalized();
		}
	}
	if (i < m_numParticles-1) {
		// force of northeast spring
		if (j > 0) {
			distance = x - state[indexOf(i+1, j-1)];
			shear_spring_force += -shear_spring_k * (distance.abs() - shear_spring_rest_len) * distance.normalized();
		}
		// force of southeast spring
		if (j < m_numParticles-1) {
			distance = x - state[indexOf(i+1, j+1)];
			shear_spring_force += -shear_spring_k * (distance.abs() - shear_spring_rest_len) * distance.normalized();
		}
	}

	// FLEX SPRING FORCE
	Vector3f flex_spring_force = Vector3f(0.0,0.0,0.0);
	// left
	if (i > 1) {
		distance = x - state[indexOf(i-2, j)];
		flex_spring_force += -flex_spring_k * (distance.abs() - flex_spring_rest_len) * distance.normalized();
	}
	// up
	if (j > 1) {
		distance = x - state[indexOf(i, j-2)];
		flex_spring_force += -flex_spring_k * (distance.abs() - flex_spring_rest_len) * distance.normalized();
	}
	// right
	if (i < m_numParticles-2) {
		distance = x - state[indexOf(i+2, j)];
		flex_spring_force += -flex_spring_k * (distance.abs() - flex_spring_rest_len) * distance.normalized();
	}
	// down
	if (j < m_numParticles-2) {
		distance = x - state[indexOf(i, j+2)];
		flex_spring_force += -flex_spring_k * (distance.abs() - flex_spring_rest_len) * distance.normalized();
	}

	return (gravity_force + drag_force + structural_spring_force + shear_spring_force + flex_spring_force)/particle_mass;

}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> ClothSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	for (int i = 0; i < m_numParticles; i++) {
		for	(int j = 0; j < m_numParticles; j++){
			f.push_back(state[indexOf(i,j) + 1]);
			if ((i==0 && j==0) || (i==m_numParticles-1 && j==0)){
				if (Move_bool){
					if (Move_String.compare("Forward")==0){
						f.push_back(Vector3f(0.f,0.f,2.f));
					}
					if (Move_String.compare("Backward")==0){
						f.push_back(Vector3f(0.f,0.f,-1.f));
					}
				}else{
					f.push_back(Vector3f(0.f,0.f,0.f));
				}
			}else{
				//Compute for f(X,t)
				f.push_back(CalculateFxt(state,i,j));
			}
		}
	}

	return f;
}


void ClothSystem::Move_button(){
	Move_bool = !Move_bool;
	bool change_flag = true;
	if((Move_String.compare("Forward")==0)&&change_flag){
		Move_String = "Backward";
		change_flag = false;
	}
	if((Move_String.compare("Backward")==0)&&change_flag){
		Move_String = "Forward";
		change_flag = false;
	}
}

///TODO: render the system (ie draw the particles)
void ClothSystem::draw()
{
	glLineWidth(2);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	for (int i = 0; i < m_numParticles; i++){
		for (int j = 0; j < m_numParticles; j++){
			Vector3f position = getState()[indexOf(i,j)];
			if (i < m_numParticles - 1){
				Vector3f right_oneoff_position = getState()[indexOf(i+1,j)];
				glVertex3f(position[0], position[1], position[2]);
				glVertex3f(right_oneoff_position[0], right_oneoff_position[1], right_oneoff_position[2]);
			}
			if (j < m_numParticles - 1){
				Vector3f up_oneoff_position = getState()[indexOf(i,j+1)];
				glVertex3f(position[0], position[1], position[2]);
				glVertex3f(up_oneoff_position[0], up_oneoff_position[1], up_oneoff_position[2]);
			}
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);

}

