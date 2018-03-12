
#include "simpleSystem.h"

using namespace std;

SimpleSystem::SimpleSystem()
{
	Vector3f init = Vector3f(1.f,1.f,0.f);
	vector<Vector3f> my_state;
	my_state.push_back(init);
	setState(my_state);

}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	for (int state_num = 0; state_num < state.size(); state_num++){
		Vector3f fxt = Vector3f(-1.f * state[state_num][1], state[state_num][0], 0.f);
		f.push_back(fxt);
	}

	return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw()
{
		Vector3f pos = getState()[0];//YOUR PARTICLE POSITION
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
}
