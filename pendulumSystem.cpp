

#include "pendulumSystem.h"

const float PendulumSystem::particle_mass = 1.0;
const float PendulumSystem::spring_const_k = 5.0;
const float PendulumSystem::spring_rest_len = 0.1;
const float PendulumSystem::drag_const_k = 0.5;

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{
	m_numParticles = numParticles;
	wind_ON = false;

	vector<Vector3f> my_state;
	m_vVecState.push_back(Vector3f(0,0,0));
	m_vVecState.push_back(Vector3f(0,0,0));
	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) {
		// for this system, we care about the position and the velocity
		// even indice -> position, odd indice -> velocity
		m_vVecState.push_back(Vector3f(0.f, i+1.f, 0.f));
		m_vVecState.push_back(Vector3f(0.f,0.f,0.f));
		m_vSprings.push_back(Vector4f(i,i+1,spring_const_k,spring_rest_len));
	}
	m_vSprings.push_back(Vector4f(0,1,5,1));
}


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	// Init f
	f.push_back(Vector3f(0.f,0.f,0.f));
	f.push_back(Vector3f(0.f,0.f,0.f));

	// Looping state
	for (unsigned i = 2; i < state.size(); i+=2){

		// Create ref for entries
		Vector3f velocity = state[i+1];
		Vector3f position = state[i];
		Vector3f gravity = particle_mass*Vector3f(0.f,-1.f,0.f);
		Vector3f drag_f = -1.f*drag_const_k*velocity;
		Vector3f force = Vector3f(0.f,0.f,0.f);

		// Compute f(X,t)
		for (unsigned j=0;j < m_vSprings.size();j++){
			Vector4f spring = m_vSprings[j];
			if (spring.x() == i/2) {
				Vector3f dist = position - state[spring.y()*2];
				force += -spring_const_k*(dist.abs() - spring_rest_len)*(dist.normalized());
			}
			else if (spring.y() == i/2) {
				Vector3f dist = position - state[spring.x()*2];
				force += -spring_const_k*(dist.abs() - spring_rest_len)*(dist.normalized());
			}
		}
		// After calculate f(X,t), push
		f.push_back(velocity);
		f.push_back(force/particle_mass);
	}

	if (wind_ON){
		for (int i = 0; i < m_numParticles; i++){
			float fraction = float(rand()) / RAND_MAX;
			f[2*i+1] += fraction*Vector3f(0.0f, 0.0f, 2.0f);
		}
	}


	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {
		Vector3f pos = getState()[2*i];//  position of particle i. YOUR CODE HERE
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}
}
