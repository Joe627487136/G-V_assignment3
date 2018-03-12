#include "ClothSystem.h"

//TODO: Initialize here
const float ClothSystem::particle_mass = 0.2;
const float ClothSystem::gap_distance = 0.2;
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
			Vector3f even_indice = Vector3f(i, 1.f, j);
			Vector3f odd_indice = Vector3f(0.f,0.f,0.f);
			my_state.push_back(even_indice);
			my_state.push_back(odd_indice);
		}
	}

	Move_bool = false;
	setState(my_state);
}

Vector3f ClothSystem::CalculateFxt(vector<Vector3f>& state, int i, int j){
	Vector3f position = state[indexOf(i,j)];
	Vector3f left_oneoff_position = state[indexOf(i-1,j)];
	Vector3f right_oneoff_position = state[indexOf(i+1,j)];
	Vector3f up_oneoff_position = state[indexOf(i,j+1)];
	Vector3f down_oneoff_position = state[indexOf(i,j-1)];
	Vector3f left_twooff_position = state[indexOf(i-2,j)];
	Vector3f right_twooff_position = state[indexOf(i+2,j)];
	Vector3f up_twooff_position = state[indexOf(i,j+2)];
	Vector3f down_twooff_position = state[indexOf(i,j-2)];
	Vector3f diagonal_right_up_oneoff_position = state[indexOf(i+1,j+1)];
	Vector3f diagonal_left_down_oneoff_position = state[indexOf(i-1,j-1)];
	Vector3f diagonal_left_up_oneoff_position = state[indexOf(i-1,j+1)];
	Vector3f diagonal_right_down_oneoff_position = state[indexOf(i+1,j-1)];
	Vector3f velocity = state[indexOf(i,j)+1];
	Vector3f drag_f = -1.f*drag_k*velocity;
	Vector3f gravity = particle_mass*Vector3f(0.f,-1.f,0.f);
	Vector3f displacement;

	// structural spring
	Vector3f structural_spring_f;

	if (i > 0){
		displacement = position - left_oneoff_position;
		structural_spring_f += -1.f*structural_spring_k*(displacement.abs()-structural_spring_rest_len)*displacement.normalized();
	}
	if (j > 0){
		displacement = position - down_oneoff_position;
		structural_spring_f += -1.f*structural_spring_k*(displacement.abs()-structural_spring_rest_len)*displacement.normalized();
	}
	if (i < m_numParticles-1){
		displacement = position - right_oneoff_position;
		structural_spring_f += -1.f*structural_spring_k*(displacement.abs()-structural_spring_rest_len)*displacement.normalized();
	}
	if (j < m_numParticles-1){
		displacement = position - up_oneoff_position;
		structural_spring_f += -1.f*structural_spring_k*(displacement.abs()-structural_spring_rest_len)*displacement.normalized();
	}

	// Deformation
	// Flex	Spring
	Vector3f flex_spring_f = Vector3f(0.f,0.f,0.f);

	if (i > 1){
		displacement = position - left_twooff_position;
		flex_spring_f = -1.f*structural_spring_k*(displacement.abs()-flex_spring_rest_len)*displacement.normalized();
	}
	if (j > 1){
		displacement = position - down_twooff_position;
		flex_spring_f = -1.f*structural_spring_k*(displacement.abs()-flex_spring_rest_len)*displacement.normalized();
	}
	if (i < m_numParticles-2){
		displacement = position - right_twooff_position;
		flex_spring_f = -1.f*structural_spring_k*(displacement.abs()-flex_spring_rest_len)*displacement.normalized();
	}
	if (j < m_numParticles-2){
		displacement = position - up_twooff_position;
		flex_spring_f = -1.f*structural_spring_k*(displacement.abs()-flex_spring_rest_len)*displacement.normalized();
	}

	// Shear Spring
	Vector3f shear_spring_f = Vector3f(0.f,0.f,0.f);

	if (i > 0){
		if (j > 0){
			displacement = position - diagonal_left_down_oneoff_position;
			shear_spring_f += -1.f*shear_spring_k*(displacement.abs()-shear_spring_rest_len)*displacement.normalized();
		}
		if (j < m_numParticles-1){
			displacement = position - diagonal_left_up_oneoff_position;
			shear_spring_f += -1.f*shear_spring_k*(displacement.abs()-shear_spring_rest_len)*displacement.normalized();
		}
	}
	if (i < m_numParticles-1){
		if (j > 0){
			displacement = position - diagonal_right_down_oneoff_position;
			shear_spring_f += -1.f*shear_spring_k*(displacement.abs()-shear_spring_rest_len)*displacement.normalized();
		}
		if (j < m_numParticles-1){
			displacement = position - diagonal_right_up_oneoff_position;
			shear_spring_f += -1.f*shear_spring_k*(displacement.abs()-shear_spring_rest_len)*displacement.normalized();
		}
	}
	Vector3f fxt = (gravity+drag_f+structural_spring_f+flex_spring_f+shear_spring_f)/particle_mass;
	return fxt;
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
					f.push_back(Vector3f(0.f,0.f,1.f));
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

