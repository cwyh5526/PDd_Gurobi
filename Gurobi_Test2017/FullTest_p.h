//#pragma once
//#include <glm/glm.hpp>
////#include <glm/gtc/matrix_transform.hpp> //for matrices
////#include <glm/gtc/type_ptr.hpp>
//
//#include <iostream>
//#include<GL/glut.h>
//
//
//#include "gurobi_c++.h"
//
//using namespace std;
//using namespace glm;
//
//typedef struct {
//	vec3 vertex[4];
//	vec3 face[4][3];
//	vec3 edge[6][2];
//	//vec3 faceNormal[4];
//}tet;
//typedef struct {
//	tet pTets[44];
//	int index[44];
//	double optValue[44] = { 1000.0 };
//	double optTime[44] = { 0.0 };
//}optResults;
//typedef struct {
//	vec3 faceNormal; //n=(a,b,c);
//	double d;		 //plane eq: ax+by+cz = d; 
//}plane;
//
//void initTet(tet* T, vec3 v0, vec3 v1, vec3 v2, vec3 v3);
//void init(tet *tetS, tet *tetR, tet *tetP);
//void rSumrConstantCalculation(tet tetR, vec3 *rSum, float *rConstant);
//void separatingPlaneCalculation(vec3 faceVrtx[3], glm::vec3 *normal, double *d);
//void optStaticFace(int fIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime);
//void optDeformingFace(int fIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime);
//void optEdgeEdge(int sIndex, int dIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime);
//
//void printV3(vec3 v);
//void printResult(tet sTet, tet rTet, tet pTet, double minOptValue, int minOptIndex, double totalOptTime);
//void resolvePenetration(tet sTet, tet rTet, tet *pTet, float *minOptValue, int *minOptIndex, float *totalOptTime, optResults *all);