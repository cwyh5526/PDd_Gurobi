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
