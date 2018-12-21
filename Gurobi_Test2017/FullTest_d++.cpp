//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp> //for matrices
//#include <glm/gtc/type_ptr.hpp>
//
//#include "gurobi_c++.h"
//
//
//using namespace std;
//using namespace glm;
//
//#define STATIC_FACE 100
//#define DEFORM_FACE 101
//#define EDGE_EDGE	102
//
//typedef struct {
//	vec3 vertex[4];
//	vec3 face[4][3];
//	vec3 edge[6][2];
//	//vec3 faceNormal[4];
//}tet;
//
//void initTet(tet* T, vec3 v0, vec3 v1, vec3 v2, vec3 v3) {
//	T->vertex[0] = v0;
//	T->vertex[1] = v1;
//	T->vertex[2] = v2;
//	T->vertex[3] = v3;
//
//	//each face number represent the vertex that the face does not contain.
//	T->face[0][0] = v1;	T->face[0][1] = v3;	T->face[0][2] = v2; //Face 0 has vertex 123
//	T->face[1][0] = v0;	T->face[1][1] = v2;	T->face[1][2] = v3; //Face 1 has vertex 023
//	T->face[2][0] = v0;	T->face[2][1] = v3;	T->face[2][2] = v1;	//Face 2 has vertex 031
//	T->face[3][0] = v0;	T->face[3][1] = v1;	T->face[3][2] = v2; //Face 3 has vertex 012
//
//	//edge (0,5) edge(1,4) edge (2,3) are pairs. if one edge is realted to the contact, then other pair edge will be included in the constraints.
//	T->edge[0][0] = v0;	T->edge[0][1] = v1;
//	T->edge[1][0] = v0;	T->edge[1][1] = v2;
//	T->edge[2][0] = v0;	T->edge[2][1] = v3;
//
//	T->edge[3][0] = v1;	T->edge[3][1] = v2;
//	T->edge[4][0] = v1;	T->edge[4][1] = v3;
//
//	T->edge[5][0] = v2;	T->edge[5][1] = v3;
//}
//void init(tet *tetS, tet *tetR, tet *tetP, vec3 *rSum, float *rConstant) {
//	//static tetrahedron position
//	initTet(tetS, vec3(0.0, 0.0, 0.0),
//		vec3(1.0, 0.0, 0.0),
//		vec3(0.0, 0.0, 1.0),
//		vec3(0.0, 1.0, 0.0));
//
//	//rest pose tetrahedron position
//	initTet(tetR, vec3(0.2, 0.2, 0.2),
//		vec3(1.2, 0.2, 0.2),
//		vec3(0.7, 0.7, 1.2),
//		vec3(0.2, 1.2, 0.2));
//
//	//deformed pose tetrahedron position same as the rest pose for the initial value
//	initTet(tetP, tetR->vertex[0],
//		tetR->vertex[1],
//		tetR->vertex[2],
//		tetR->vertex[3]);
//}
//
//
//void separatingPlaneCalculation(vec3 faceVrtx[3], glm::vec3 *normal){
//	//compute the normal vector and constant of the plane equation for 3 vertices
//	glm::vec3 v01 = faceVrtx[1] - faceVrtx[0];
//	glm::vec3 v02 = faceVrtx[2] - faceVrtx[0];
//	std::cout << "v01: (" << (v01).x << "," << (v01).y << "," << (v01).z << ")" << std::endl;
//	std::cout << "v02: (" << (v02).x << "," << (v02).y << "," << (v02).z << ")" << std::endl;
//
//	(*normal) = glm::normalize(glm::cross(v01, v02));
//}
//void optStaticFace(int fIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime) {
//	cout << "\n =========optStaticFace Face " << fIndex << endl;
//
//	try {
//		GRBEnv env = GRBEnv();
//
//		GRBModel model = GRBModel(env);
//		// Create variables
//
//		GRBVar x0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x0");
//		GRBVar x1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x1");
//		GRBVar x2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x2");
//		GRBVar x3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x3");
//
//		GRBVar y0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y0");
//		GRBVar y1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y1");
//		GRBVar y2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y2");
//		GRBVar y3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y3");
//
//
//		GRBVar z0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z0");
//		GRBVar z1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z1");
//		GRBVar z2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z2");
//		GRBVar z3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z3");
//
//
//		GRBQuadExpr obj =
//			  x0*x0 + x0*x1 + x1*x1 + x0*x2 + x1*x2 + x2*x2 + x0*x3 + x1*x3 + x2*x3 + x3*x3
//			+ y0*y0 + y0*y1 + y1*y1 + y0*y2 + y1*y2 + y2*y2 + y0*y3 + y1*y3 + y2*y3 + y3*y3
//			+ z0*z0 + z0*z1 + z1*z1 + z0*z2 + z1*z2 + z2*z2 + z0*z3 + z1*z3 + z2*z3 + z3*z3;
//
//		vec3 normal;
//
//		separatingPlaneCalculation(sTet.face[fIndex], &normal);
//
//
//		// Add constraint: 
//		double constraintValue[4] = { 0.0 };
//		constraintValue[0] = glm::dot(normal, sTet.face[fIndex][0] - rTet.vertex[0]); // a(s_0x - r_0x) + b(s_0y - r_0y) + c(s_0z - r_0z)
//		constraintValue[1] = glm::dot(normal, sTet.face[fIndex][0] - rTet.vertex[1]); // a(s_0x - r_1x) + b(s_0y - r_1y) + c(s_0z - r_1z)
//		constraintValue[2] = glm::dot(normal, sTet.face[fIndex][0] - rTet.vertex[2]); // a(s_0x - r_2x) + b(s_0y - r_2y) + c(s_0z - r_2z)
//		constraintValue[3] = glm::dot(normal, sTet.face[fIndex][0] - rTet.vertex[3]); // a(s_0x - r_3x) + b(s_0y - r_3y) + c(s_0z - r_3z)
//
//		
//		model.addConstr(normal.x*x0 + normal.y * y0 + normal.z * z0 >= constraintValue[0], "c0"); // n，(p0-s0)>=0 p0-r0=d0
//		model.addConstr(normal.x*x1 + normal.y * y1 + normal.z * z1 >= constraintValue[1], "c1"); // n，(p1-s0)>=0
//		model.addConstr(normal.x*x2 + normal.y * y2 + normal.z * z2 >= constraintValue[2], "c2"); // n，(p2-s0)>=0
//		model.addConstr(normal.x*x3 + normal.y * y3 + normal.z * z3 >= constraintValue[3], "c3"); // n，(p3-s0)>=0
//
//        //Optimization
//		model.optimize();
//		//get Result;
//		*optValue = model.get(GRB_DoubleAttr_ObjVal);
//		*optTime = model.get(GRB_DoubleAttr_Runtime);
//
//		//p-r=d, p=d+r
//		vec3 p0 = vec3(x0.get(GRB_DoubleAttr_X), y0.get(GRB_DoubleAttr_X), z0.get(GRB_DoubleAttr_X)) + rTet.vertex[0];
//		vec3 p1 = vec3(x1.get(GRB_DoubleAttr_X), y1.get(GRB_DoubleAttr_X), z1.get(GRB_DoubleAttr_X)) + rTet.vertex[1];
//		vec3 p2 = vec3(x2.get(GRB_DoubleAttr_X), y2.get(GRB_DoubleAttr_X), z2.get(GRB_DoubleAttr_X)) + rTet.vertex[2];
//		vec3 p3 = vec3(x3.get(GRB_DoubleAttr_X), y3.get(GRB_DoubleAttr_X), z3.get(GRB_DoubleAttr_X)) + rTet.vertex[3];
//
//
//		initTet(pTet,p0,p1,p2,p3);
//	}
//	catch (GRBException e) {
//		cout << "Error code = " << e.getErrorCode() << endl;
//		cout << e.getMessage() << endl;
//	}
//	catch (...) {
//		cout << "Exception during optimization" << endl;
//	}
//}
//
//
//void resolvePenetration(vec3 rSum, float rConstant, tet sTet, tet rTet, tet *pTet, double *minOptValue, int *minOptIndex, double *totalOptTime) {
//	//test 44 find minimum
//	double optValue[44] = { 1000.0 };
//	double optTime[44] = { 0.0 };
//	int contactVertex[4] = { 0 };
//	// 4 static faces
//
//	(*totalOptTime) = 0;
//	(*minOptIndex) = -1;
//	(*minOptValue) = 1000;
//	
//
//	for (int i = 0; i < 4; i++) {
//		optStaticFace(i, sTet, rTet, rSum, rConstant, pTet, &optValue[i], &optTime[i]);
//		(*totalOptTime) += optTime[i];
//		if ((*minOptValue) > optValue[i]) {
//			*minOptValue = optValue[i];
//			*minOptIndex = i;
//		}
//	}
//
//	//4 deforming faces and 4 vertices for each case
//	//for (int i = 0; i < 4; i++) {
//	//	int index = i + 4;
//	//	optDeformingFace(i, sTet, rTet, rSum, rConstant, pTet, &optValue[index], &optTime[index]);// , &contactVertex[i]);
//	//	(*totalOptTime) += optTime[index];
//	//	if ((*minOptValue) > optValue[index]) {
//	//		*minOptValue = optValue[index];
//	//		*minOptIndex = index;
//	//	}
//	//}
//
//	//36 deforming faces from edge-edge case
//	//for (int s = 0; s < 6; s++) { // static edge
//	//	for (int p = 0; p < 6; p++) { //deforming edge
//	//		optEdgeEdge();
//	//	}
//	//}
//
//	//find minimum metric value and that case.
//}
//
//
//void printV3(vec3 v) {
//	cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
//}
//void printResult(tet sTet, tet rTet, tet pTet, double minOptValue, int minOptIndex, double totalOptTime) {
//	if (minOptIndex != -1) {
//
//		cout << "[Optimization Result]" << endl;
//		cout << "Minimum metric vale: " << minOptValue << endl;
//		cout << "Total Optimization time: " << totalOptTime << endl;
//		cout << "Separating plane from: " << minOptIndex << endl;
//
//		for (int i = 0; i < 4; i++) {
//			cout << "tetStatic: s" << i << "=";
//			printV3(sTet.vertex[i]);
//		}
//		for (int i = 0; i < 4; i++) {
//			cout << "tetRest: r" << i << "=";
//			printV3(rTet.vertex[i]);
//		}
//		for (int i = 0; i < 4; i++) {
//			cout << "tetDeformed: p" << i << "=";
//			printV3(pTet.vertex[i]);
//		}
//
//	}
//	else {
//		cout << "ERROR: Not Appropriate Tet Deform calculation " << endl;
//	}
//
//	if (minOptIndex < 4) {
//		cout << "Separating plane from static tetrahedron face " << minOptIndex << ":";
//		for (int i = 0; i < 3; i++) {
//			printV3(sTet.face[minOptIndex][i]);
//		}
//	}
//	else if (minOptIndex < 8) {
//		int index = minOptIndex - 4;
//
//		cout << "Separating Plane from deforming tetrahedron face " << index << ":";
//		for (int i = 0; i < 3; i++) {
//			printV3(pTet.face[index][i]);
//		}
//	}
//	else {
//		int index = minOptIndex - 8;
//		int indexS = index / 6;
//		int indexP = index % 6;
//		cout << "Separating Plane from static edge :";
//		for (int i = 0; i < 2; i++) {
//			printV3(sTet.edge[indexS][i]);
//		}
//		cout << "Separating Plane from deforming edge :";
//		for (int i = 0; i < 2; i++) {
//			printV3(pTet.edge[indexP][i]);
//		}
//
//	}
//}
//int main(int argc, const char *argv[])
//{
//	tet sTet; //static tetrahedron
//	tet rTet; //rest pose tetrahedron
//	tet pTet; //deformed pose tetrahedron
//	double totalOptTime;
//	double minOptValue;
//	int minOptIndex;
//
//	vec3 rSum;
//	float rConstant;
//
//	init(&sTet, &rTet, &pTet, &rSum, &rConstant);
//
//	resolvePenetration(rSum, rConstant, sTet, rTet, &pTet, &minOptValue, &minOptIndex, &totalOptTime);
//
//	printResult(sTet, rTet, pTet, minOptValue, minOptIndex, totalOptTime);
//
//	return 0;
//}