#pragma once
#include <glm/glm.hpp>

#include "gurobi_c++.h"

using namespace glm;

typedef struct {
	GRBVar x;
	GRBVar y;
	GRBVar z;
}Var;

typedef struct {
	vec3 vertex[4];
	vec3 face[4][3];
	vec3 edge[6][2];


	
	void init(vec3 v[4]) {
		for (int i = 0; i < 4; ++i) {
			vertex[i] = v[i];
		}
		//each face number represent the vertex that the face does not contain.
		face[0][0] = v[1];	face[0][1] = v[2];	face[0][2] = v[3]; //Face 0 has vertex 123
		face[1][0] = v[0];	face[1][1] = v[3];	face[1][2] = v[2]; //Face 1 has vertex 023
		face[2][0] = v[0];	face[2][1] = v[1];	face[2][2] = v[3];	//Face 2 has vertex 031
		face[3][0] = v[0];	face[3][1] = v[2];	face[3][2] = v[1]; //Face 3 has vertex 012

		//edge (0,5) edge(1,4) edge (2,3) are pairs. if one edge is realted to the contact, then other pair edge will be included in the constraints.
		edge[0][0] = v[0];	edge[0][1] = v[1];
		edge[1][0] = v[0];	edge[1][1] = v[2];
		edge[2][0] = v[0];	edge[2][1] = v[3];

		edge[3][0] = v[1];	edge[3][1] = v[2];
		edge[4][0] = v[1];	edge[4][1] = v[3];

		edge[5][0] = v[2];	edge[5][1] = v[3];
	}
}tet;
typedef struct {
	//plane ax+by+cz+d=0
	vec3 n; //n=(a,b,c)
	float d; 
}plane;
typedef struct {
	tet pTets[44];
	int index[44];
	double optValue[44] = { 1000.0 };
	double optTime[44] = { 0.0 };
	vec3 normal[44];
}optResults;
typedef struct {
	tet pTets1[44];
	tet pTets2[44];
	int index[44];
	double optValue[44] = { 1000.0 };
	float optTime[44] = { 0.0 };
	vec3 normal[44];
	vec3 planePoint[44];
}optResults2;

typedef struct {
	tet pTets1[44];
	tet pTets2[44];
	int index[44];
	float optValue[44] = { 1000.0 };
	
	//time will not contained
	float optTime[44] = { 0.0 };
	plane pl[44];
	vec3 planePoint[44];
}optResults3;

//class Tetrahedron {
//public:
//	vec3 vertex[4];
//	vec3 face[4][3];
//	vec3 edge[6][2];
//	float volume;
//	
//	Tetrahedron() {};
//	Tetrahedron(vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(v0, v1, v2, v3); };
//	~Tetrahedron() {};
//
//	void initTet(tet t) { initTet(t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };
//	void initTet(vec3 v0, vec3 v1, vec3 v2, vec3 v3);
//	tet toTet();
//	float calculateTetVolume();
//};
