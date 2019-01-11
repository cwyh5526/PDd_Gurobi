#pragma once
#pragma once
#include <glm/glm.hpp>

#include <iostream>
#include<GL/glut.h>


#include "gurobi_c++.h"

using namespace std;
using namespace glm;


typedef struct {
	vec3 vertex[4];
	vec3 face[4][3];
	vec3 edge[6][2];
}tet;
typedef struct {
	tet pTets[44];
	int index[44];
	double optValue[44] = { 1000.0 };
	double optTime[44] = { 0.0 };
}optResults;
typedef struct {
	tet pTets1[44]; //p
	tet pTets2[44]; //s
	int index[44];
	double optValue[44] = { 1000.0 };
	double optTime[44] = { 0.0 };
}optResults2;

typedef struct {
	vec3 faceNormal; //n=(a,b,c);
	double d;		 //plane eq: ax+by+cz = d; 
}plane;


class DefDefPD
{
public:
	// constructor
	DefDefPD(); //create Environment  for optimization
	~DefDefPD(); //delete Environment 

			  // initialize optimization variables and geometries : it is the first function that should be called
	void initDefault();
	void init(tet tetS, tet tetR);
	/* static-deformable PD */
	void resolveStaticDefPenetration();
	/* def-def PD*/
	void resolveDefDefPenetration();

	// getters
	tet getSTet() { return sTet; };
	tet getRTet() { return rTet; };
	tet getPTet() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return pTet; };
	tet getPTet2() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return pTet; };

	optResults  getPTetAll() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return pTetAll; };
	optResults2 getPTetAll2() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return pTetAll2; };

	int getMinOptIndex() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return minOptIndex; };
	float getPD() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return minOptValue; };
	float getOptTime() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return totalOptTime; };
	void printResult(int pairIndex);
	void printResult2(int pairIndex);



protected:

	/* static-deformable PD */
	// substeps of optimization
	void optStaticFace(int fIndex, int pairIndex);
	void optDeformingFace(int fIndex, int pairIndex);
	void optEdgeEdge(int sIndex, int dIndex, int pairIndex);

	/* def-def PD*/
	void optDefDefFace(int fIndex, int pairIndex);
	void optDefDefEdge(int sIndex, int dIndex, int pairIndex);

	// simple calculation functions : should be called right after rTet and sTet is initialized
	float calculateTetVolume(tet t);
	void sumConstantCalculation(tet t, vec3 &sum, float &constant);
	void separatingPlaneCalculation(vec3 faceVrtx[3], glm::vec3 *normal, double *d);//it can be called before optimization.... let me think how to move this. I need to make the data structures for this
	void calculateMidPoint(); //calculate midpoint of two tets : it will be used as the position of separating plane.

							  // setters for tetrahedra
	void setSTet(tet t) { initTet(sTet, t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };
	void setRTet(tet t) { initTet(rTet, t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };
	void setPTet(tet t) { initTet(pTet, t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };
	void setSTet(vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(sTet, v0, v1, v2, v3); };
	void setRTet(vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(rTet, v0, v1, v2, v3); };
	void setPTet(vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(pTet, v0, v1, v2, v3); };

	// utility functions
	void initTet(tet &T, vec3 v0, vec3 v1, vec3 v2, vec3 v3);
	void printV3(vec3 v);

private:

	// optimization variables
	GRBEnv* env = NULL;

	// input geometries and pre-calculated values
	tet sTet, rTet;
	vec3 rSum;
	float rConstant;
	float rVolume;
	vec3 sSum;

	//output geometries and values
	tet pTet;
	optResults pTetAll;
	int minOptIndex = -1;
	float minOptValue = 1000.0f;
	float totalOptTime = 0.f;

	//optimization flag
	//: should be true when optimization function is called. 
	//  should be false when initialized function is called.
	bool optimized = false;


	//for def def
	float sConstant;
	float sVolume;
	vec3 midPoint; //the mid point of two rest tetrahedron for Def-Def case.
	tet pTet2;
	optResults2 pTetAll2;


};
