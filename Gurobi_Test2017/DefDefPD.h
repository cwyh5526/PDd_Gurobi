#pragma once

#include <iostream>
#include <fstream>

#include<GL/glut.h>
#include"DataStructure.h"


using namespace std;

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
	//void resolveStaticDefPenetration();
	/* def-def PD*/
	void resolveDefDefPenetration();

	// getters
	tet getRTet(int i) { return rTet[i]; };
	tet getPTet(int i) { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return pTet[i]; };

	optResults2 getPTetAll() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return pTetAll2; };

	int getMinOptIndex() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return minOptIndex; };
	float getPD() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return minOptValue; };
	float getOptTime() { if (!optimized) { cout << "ERROR: optimization is not performed yet" << endl; } return totalOptTime; };
	int getNumOpt() { return numOpt; }
	void printResult(int pairIndex);

	void writeCSV(string fileName);
	void writeCSVHead(string fileName);

protected:

	/* static-deformable PD */

	/* def-def PD*/
	void optDefDefFace(int fIndex, int pairIndex);
	void optDefDefEdge(int sIndex, int dIndex, int pairIndex);

	// simple calculation functions : should be called right after rTet and sTet is initialized
	float calculateTetVolume(tet t);
	void sumConstantCalculation(tet t, vec3 &sum, float &constant);
	void separatingPlaneCalculation(vec3 faceVrtx[3], vec3 vrtx, vec3 *normal, double *d);//it can be called before optimization.... let me think how to move this. I need to make the data structures for this
	void calculateMidPoint(); //calculate midpoint of two tets : it will be used as the position of separating plane.
	void calculateProjecton(vec3 v, vec3 n);

   // setters for tetrahedra
	void setRTet(int i,tet t) { initTet(rTet[i], t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };
	void setPTet(int i,tet t) { initTet(pTet[i], t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };

	void setRTet(int i, vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(rTet[i], v0, v1, v2, v3);};
	void setPTet(int i, vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(pTet[i], v0, v1, v2, v3); };

	// utility functions
	void initTet(tet &T, vec3 v0, vec3 v1, vec3 v2, vec3 v3);
	void printV3(vec3 v);
	void fprintV3(ofstream &fp, vec3 v);

private:

	// optimization variables
	GRBEnv* env = NULL;

	// input geometries and pre-calculated values
	tet rTet[2];
	vec3 rSum[2];
	float rConstant[2];
	float rVolume[2];

	//output geometries and values
	tet pTet[2];
	optResults2 pTetAll2;
	int minOptIndex = -1;
	float minOptValue = 1000.0f;
	float totalOptTime = 0.f;
	vec3 optPlanePoint;

	//optimization flag
	//: should be true when optimization function is called. 
	//  should be false when initialized function is called.
	bool optimized = false;

	int numOpt = 0;


	//for def def

	//vec3 midPoint; //the mid point of two rest tetrahedron for Def-Def case.

};
