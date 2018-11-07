/* Copyright 2018, Gurobi Optimization, LLC */

/* This example formulates and solves the following simple QP model:

minimize    x^2 + x*y + y^2 + y*z + z^2 + 2 x
subject to  x + 2 y + 3 z >= 4
x +   y       >= 1

It solves it once as a continuous model, and once as an integer model.
*/

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //for matrices
#include <glm/gtc/type_ptr.hpp>

#include "gurobi_c++.h"


using namespace std;
using namespace glm;

#define STATIC_FACE 100
#define DEFORM_FACE 101
#define EDGE_EDGE	102

typedef struct {
	vec3 vertex[4];
	vec3 face[4][3];
	vec3 edge[6][2];
	//vec3 faceNormal[4];
}tet;

typedef struct {
	vec3 faceNormal; //n=(a,b,c);
	double d;		 //plane eq: ax+by+cz = d; 
}plane;

void initTet(tet* T, vec3 v0, vec3 v1, vec3 v2, vec3 v3) {
	T->vertex[0] = v0;
	T->vertex[1] = v1;
	T->vertex[2] = v2;
	T->vertex[3] = v3;

	T->face[0][0] = v0;	T->face[0][1] = v1;	T->face[0][2] = v2;
	T->face[1][0] = v0;	T->face[1][1] = v2;	T->face[1][2] = v3;
	T->face[2][0] = v0;	T->face[2][1] = v3;	T->face[2][2] = v1;
	T->face[3][0] = v1;	T->face[3][1] = v3;	T->face[3][2] = v2;

	T->edge[0][0] = v0;	T->edge[0][1] = v1;
	T->edge[1][0] = v0;	T->edge[1][1] = v2;
	T->edge[2][0] = v0;	T->edge[2][1] = v3;

	T->edge[3][0] = v1;	T->edge[3][1] = v2;
	T->edge[4][0] = v1;	T->edge[4][1] = v3;

	T->edge[5][0] = v2;	T->edge[5][1] = v3;
}
void init(tet *tetS, tet *tetR, tet *tetP, vec3 *rSum, float *rConstant) {
	//static tetrahedron position
	initTet(tetS, vec3(0.0, 0.0, 0.0),
		vec3(1.0, 0.0, 0.0),
		vec3(0.0, 0.0, 1.0),
		vec3(0.0, 1.0, 0.0));

	//rest pose tetrahedron position
	initTet(tetR, vec3(0.2, 0.2, 0.2),
		vec3(1.2, 0.2, 0.2),
		vec3(0.7, 0.7, 1.2),
		vec3(0.2, 1.2, 0.2));

	//deformed pose tetrahedron position same as the rest pose for the initial value
	initTet(tetP, tetR->vertex[0],
		tetR->vertex[1],
		tetR->vertex[2],
		tetR->vertex[3]);


	//precompute the sum of coordinate values of rest pose tet. 
	(*rSum) = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++) {
		rSum->x += tetR->vertex[i].x;
		rSum->z += tetR->vertex[i].z;
		rSum->y += tetR->vertex[i].y;
	}

	//precompute constant term of the objective function
	vec3 constant = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < (i + 1); j++) {
			constant.x += tetR->vertex[i].x * tetR->vertex[j].x;
			constant.y += tetR->vertex[i].y * tetR->vertex[j].y;
			constant.z += tetR->vertex[i].z * tetR->vertex[j].z;
		}
	}
	(*rConstant) = (float)(constant.x + constant.y + constant.z);
}
void separatingPlaneCalculation(vec3 faceVrtx[3], glm::vec3 *normal, double *d) {
	glm::vec3 v01 = faceVrtx[1] - faceVrtx[0];
	glm::vec3 v02 = faceVrtx[2] - faceVrtx[0];
	std::cout << "v01: (" << (v01).x << "," << (v01).y << "," << (v01).z << ")" << std::endl;
	std::cout << "v02: (" << (v02).x << "," << (v02).y << "," << (v02).z << ")" << std::endl;

	(*normal) = glm::normalize(glm::cross(v01, v02));
	*(d) = glm::dot(*normal, faceVrtx[0]);
}
void deformFaceCosntraint() {

}


void optStaticFace(int fIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue,  double *optTime) {

	try {
		GRBEnv env = GRBEnv();

		GRBModel model = GRBModel(env);

		// Create variables
		
		GRBVar xP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
		GRBVar xP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
		GRBVar xP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
		GRBVar xP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");

		GRBVar yP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
		GRBVar yP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
		GRBVar yP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
		GRBVar yP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");


		GRBVar zP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
		GRBVar zP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
		GRBVar zP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP[3]");

		// Set objective
		GRBQuadExpr obj = 
			  xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
			+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
			+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3

			- xP0*(rSum.x + rTet.vertex[0].x)
			- xP1*(rSum.x + rTet.vertex[1].x)
			- xP2*(rSum.x + rTet.vertex[2].x)
			- xP3*(rSum.x + rTet.vertex[3].x)

			- yP0*(rSum.y + rTet.vertex[0].y)
			- yP1*(rSum.y + rTet.vertex[1].y)
			- yP2*(rSum.y + rTet.vertex[2].y)
			- yP3*(rSum.y + rTet.vertex[3].y)

			- zP0*(rSum.z + rTet.vertex[0].z)
			- zP1*(rSum.z + rTet.vertex[1].z)
			- zP2*(rSum.z + rTet.vertex[2].z)
			- zP3*(rSum.z + rTet.vertex[3].z)

			+ rConstant;
					
		model.setObjective(obj);

		// Add constraint: 
	
		glm::vec3 normal;
		double constraintValue = 0.0 ;
		separatingPlaneCalculation(sTet.face[fIndex], &normal, &constraintValue);

		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 >= constraintValue, "c0");
		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1 >= constraintValue, "c1");
		model.addConstr(normal.x*xP2 + normal.y * yP2 + normal.z * zP2 >= constraintValue, "c2");
		model.addConstr(normal.x*xP3 + normal.y * yP3 + normal.z * zP3 >= constraintValue, "c3");

		model.optimize();	

		//get Result;
		*optValue = model.get(GRB_DoubleAttr_ObjVal);
		*optTime = model.get(GRB_DoubleAttr_Runtime);
		
		initTet(pTet, vec3(xP0.get(GRB_DoubleAttr_X),yP0.get(GRB_DoubleAttr_X),zP0.get(GRB_DoubleAttr_X)), 
			vec3(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X)),
			vec3(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X)),
			vec3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X)));
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}
}

void optDeformFace(int f, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime, int *contactVertex){
	// which vertices will be used for constraint? it depends on the face index
	//f:face index of deforming tetrahedron (0~3) used for optimization
	int v[4][4] = { {0,1,2, 3 }, {0,2,3,  1}, {0,3,1,  2},{1,3,2,  0} };// vIndex[faceIndex]={first vertex index, second vertex index, third vertex index, other vertex} of triangle face
	int s[4][4] = { {0,1,2,3},{1,2,3,0},{2,3,0,1},{3,0,1,2} }; //s[i][0]: contact index i, s[i][1,2,3]: used for constraint
	double metricValue[4];
	double time[4];
	vec3 p[4][4];
	
	for (int i = 0; i < 4; i++) { // i : index of static vertex in contact
		try {
			GRBEnv env = GRBEnv();

			GRBModel model = GRBModel(env);

			// Create variables
			GRBVar xP[4], yP[4], zP[4];
			xP[0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
			xP[1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
			xP[2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
			xP[3] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");

			yP[0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
			yP[1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
			yP[2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
			yP[3] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");

			zP[0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
			zP[1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
			zP[2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
			zP[3] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");


			// Set objective
			GRBQuadExpr obj =
				xP[0] * xP[0] + xP[0] * xP[1] + xP[1] * xP[1] + xP[0] * xP[2] + xP[1] * xP[2] + xP[2] * xP[2] + xP[0] * xP[3] + xP[1] * xP[3] + xP[2] * xP[3] + xP[3] * xP[3]
				+ yP[0] * yP[0] + yP[0] * yP[1] + yP[1] * yP[1] + yP[0] * yP[2] + yP[1] * yP[2] + yP[2] * yP[2] + yP[0] * yP[3] + yP[1] * yP[3] + yP[2] * yP[3] + yP[3] * yP[3]
				+ zP[0] * zP[0] + zP[0] * zP[1] + zP[1] * zP[1] + zP[0] * zP[2] + zP[1] * zP[2] + zP[2] * zP[2] + zP[0] * zP[3] + zP[1] * zP[3] + zP[2] * zP[3] + zP[3] * zP[3]

				- xP[0] * (rSum.x + rTet.vertex[0].x)
				- xP[1] * (rSum.x + rTet.vertex[1].x)
				- xP[2] * (rSum.x + rTet.vertex[2].x)
				- xP[3] * (rSum.x + rTet.vertex[3].x)

				- yP[0] * (rSum.y + rTet.vertex[0].y)
				- yP[1] * (rSum.y + rTet.vertex[1].y)
				- yP[2] * (rSum.y + rTet.vertex[2].y)
				- yP[3] * (rSum.y + rTet.vertex[3].y)

				- zP[0] * (rSum.z + rTet.vertex[0].z)
				- zP[1] * (rSum.z + rTet.vertex[1].z)
				- zP[2] * (rSum.z + rTet.vertex[2].z)
				- zP[3] * (rSum.z + rTet.vertex[3].z)

				+ rConstant;

			model.setObjective(obj);

			//Add constraint:
			// contact vertex: s[i][0], constraint vertex: s[i][1~3]
			// a= 
			// b=
			// c=
			vec3 s0 = sTet.vertex[s[i][0]];
			vec3 s1 = sTet.vertex[s[i][1]];
			vec3 s2 = sTet.vertex[s[i][2]];
			vec3 s3 = sTet.vertex[s[i][3]];
			model.addQConstr(((s1.x - s0.x)*((yP[v[f][1]] - yP[v[f][0]])*(zP[v[f][2]]  - zP[v[f][0]]) - (zP[v[f][1]] - zP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]))
							+(s1.y - s0.y)*((zP[v[f][1]] - zP[v[f][0]])*(xP[v[f][2]]  - xP[v[f][0]]) - (xP[v[f][1]] - xP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]))
							+(s1.z - s0.z)*((xP[v[f][1]] - xP[v[f][0]])*(yP[v[f][2]]  - yP[v[f][0]]) - (yP[v[f][1]] - yP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]))) >= 0,"c0");
			model.addQConstr(((s2.x - s0.x)*((yP[v[f][1]] - yP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]) - (zP[v[f][1]] - zP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]))
							+ (s2.y - s0.y)*((zP[v[f][1]] - zP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]) - (xP[v[f][1]] - xP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]))
							+ (s2.z - s0.z)*((xP[v[f][1]] - xP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]) - (yP[v[f][1]] - yP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]))) >= 0, "c0");
			
			model.addQConstr(((s3.x - s0.x)*((yP[v[f][1]] - yP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]) - (zP[v[f][1]] - zP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]))
							+ (s3.y - s0.y)*((zP[v[f][1]] - zP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]) - (xP[v[f][1]] - xP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]))
							+ (s3.z - s0.z)*((xP[v[f][1]] - xP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]) - (yP[v[f][1]] - yP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]))) >= 0, "c0");
			
			//model.addConstr(() >= 0, "c0");

			/*model.addQConstr((s1.x*((yP[v[f][1]] - yP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]) - (zP[v[f][1]] - zP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]))
							+ s1.y*((zP[v[f][1]] - zP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]) - (xP[v[f][1]] - xP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]))
							+ s1.z*((xP[v[f][1]] - xP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]) - (yP[v[f][1]] - yP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]])))
								>= (s0.x*(yP[v[f][1]] * zP[v[f][2]] - yP[v[f][2]] * zP[v[f][1]])
								  + s0.y*(xP[v[f][2]] * zP[v[f][1]] - xP[v[f][1]] * zP[v[f][2]])
								  + s0.z*(xP[v[f][1]] * yP[v[f][2]] - xP[v[f][2]] * yP[v[f][1]])), "c0");

			model.addQConstr((s2.x*((yP[v[f][1]] - yP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]) - (zP[v[f][1]] - zP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]))
							+ s2.y*((zP[v[f][1]] - zP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]) - (xP[v[f][1]] - xP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]))
							+ s2.z*((xP[v[f][1]] - xP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]) - (yP[v[f][1]] - yP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]])))
								>= (s0.x*(yP[v[f][1]] * zP[v[f][2]] - yP[v[f][2]] * zP[v[f][1]])
								  + s0.y*(xP[v[f][2]] * zP[v[f][1]] - xP[v[f][1]] * zP[v[f][2]])
								  + s0.z*(xP[v[f][1]] * yP[v[f][2]] - xP[v[f][2]] * yP[v[f][1]])), "c1");

			model.addQConstr((s3.x*((yP[v[f][1]] - yP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]) - (zP[v[f][1]] - zP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]))
							+ s3.y*((zP[v[f][1]] - zP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]]) - (xP[v[f][1]] - xP[v[f][0]])*(zP[v[f][2]] - zP[v[f][0]]))
							+ s3.z*((xP[v[f][1]] - xP[v[f][0]])*(yP[v[f][2]] - yP[v[f][0]]) - (yP[v[f][1]] - yP[v[f][0]])*(xP[v[f][2]] - xP[v[f][0]])))
								>= (s0.x*(yP[v[f][1]] * zP[v[f][2]] - yP[v[f][2]] * zP[v[f][1]])
								  + s0.y*(xP[v[f][2]] * zP[v[f][1]] - xP[v[f][1]] * zP[v[f][2]])
							 	  + s0.z*(xP[v[f][1]] * yP[v[f][2]] - xP[v[f][2]] * yP[v[f][1]])), "c2");*/
			model.optimize();
			metricValue[i]= model.get(GRB_DoubleAttr_ObjVal);
			time[i]= model.get(GRB_DoubleAttr_Runtime);
			for (int vtx = 0; vtx< 4; vtx++)
			{
				p[i][vtx] = vec3(xP[vtx].get(GRB_DoubleAttr_X), yP[vtx].get(GRB_DoubleAttr_X), zP[vtx].get(GRB_DoubleAttr_X));
			}
			/*cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
			cout << "Time:" << model.get( << endl;*/

		}
		catch (GRBException e) {
			cout << "Error code = " << e.getErrorCode() << endl;
			cout << e.getMessage() << endl;
		}
		catch (...) {
			cout << "Exception during optimization" << endl;
		}
	}


	*optValue = metricValue[0];
	*contactVertex = 0;
	*optTime = 0;

	for (int i = 1; i < 4; i++) {
		*optTime += time[i];// sum up the time spent
		if (*optValue > metricValue[i]) //find min Metric value
		{
			*optValue = metricValue[i];
			*contactVertex = i;			
		}
	}

	initTet(pTet,p[*contactVertex][0], p[*contactVertex][1], p[*contactVertex][2], p[*contactVertex][3]);

}
void optEdgeEdge(int e1,int e2, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime) {
	//e1:static edge index, e2:deforming edge index
	int e[6][2] = { { 0,1 },{ 0,2 },{ 0,3 },{ 1,2 },{ 1,3 },{ 2,3 } };  // e[edgeIndex] = {first vertex Index, second vertex Index}
	int ec[6][2] = { { 2,3 },{ 1,3 },{ 1,2 },{ 0,3 },{ 0,2 },{ 0,1 } }; // index of edges that does not contact.
	try {
		GRBEnv env = GRBEnv();

		GRBModel model = GRBModel(env);

		// Create variables
		GRBVar xP[4], yP[4], zP[4];
		xP[0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
		xP[1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
		xP[2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
		xP[3] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");

		yP[0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
		yP[1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
		yP[2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
		yP[3] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");

		zP[0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
		zP[1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
		zP[2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
		zP[3] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");


		// Set objective
		GRBQuadExpr obj =
			  xP[0] * xP[0] + xP[0] * xP[1] + xP[1] * xP[1] + xP[0] * xP[2] + xP[1] * xP[2] + xP[2] * xP[2] + xP[0] * xP[3] + xP[1] * xP[3] + xP[2] * xP[3] + xP[3] * xP[3]
			+ yP[0] * yP[0] + yP[0] * yP[1] + yP[1] * yP[1] + yP[0] * yP[2] + yP[1] * yP[2] + yP[2] * yP[2] + yP[0] * yP[3] + yP[1] * yP[3] + yP[2] * yP[3] + yP[3] * yP[3]
			+ zP[0] * zP[0] + zP[0] * zP[1] + zP[1] * zP[1] + zP[0] * zP[2] + zP[1] * zP[2] + zP[2] * zP[2] + zP[0] * zP[3] + zP[1] * zP[3] + zP[2] * zP[3] + zP[3] * zP[3]

			- xP[0] * (rSum.x + rTet.vertex[0].x)
			- xP[1] * (rSum.x + rTet.vertex[1].x)
			- xP[2] * (rSum.x + rTet.vertex[2].x)
			- xP[3] * (rSum.x + rTet.vertex[3].x)

			- yP[0] * (rSum.y + rTet.vertex[0].y)
			- yP[1] * (rSum.y + rTet.vertex[1].y)
			- yP[2] * (rSum.y + rTet.vertex[2].y)
			- yP[3] * (rSum.y + rTet.vertex[3].y)

			- zP[0] * (rSum.z + rTet.vertex[0].z)
			- zP[1] * (rSum.z + rTet.vertex[1].z)
			- zP[2] * (rSum.z + rTet.vertex[2].z)
			- zP[3] * (rSum.z + rTet.vertex[3].z)

			+ rConstant;

		model.setObjective(obj);

		//Add constraint:
		// n=(a,b,c) 
		// a = (yP[e[e2][1]] - yP[e[e2][0]])*(s1.z - s0.z) - (zP[e[e2][1]] - zP[e[e2][0]])*(s1.y - s0.y);
		// b = (zP[e[e2][1]] - zP[e[e2][0]])*(s1.x - s0.x) - (xP[e[e2][1]] - xP[e[e2][0]])*(s1.z - s0.z);
		// c = (xP[e[e2][1]] - xP[e[e2][0]])*(s1.y - s0.y) - (yP[e[e2][1]] - yP[e[e2][0]])*(s1.x - s0.x);
		// d= dot((a,b,c),(sTet.vertex[e[e1][0]]))
		// s0=sTet.vertex[e[e1][0]], s1=sTet.vertex[e[e1][1]]
		
		//check if n=(a,b,c) direction is correct
		// no need to check the direction. 
		// what I need to check is that dot(n,p2-s0) * dot(n,p3-s0) >0, 
		// dot(n,s2-s0) * dot(n,s1-s0) >0
		// dot(n,p2-s0) * dot(n,s2-s0) <0
		vec3 s0 = sTet.vertex[e[e1][0]]; 
		vec3 s1 = sTet.vertex[e[e1][1]];
		(yP[e[e2][1]] - yP[e[e2][0]])*(s1.z - s0.z) - (zP[e[e2][1]] - zP[e[e2][0]])*(s1.y - s0.y);
		(zP[e[e2][1]] - zP[e[e2][0]])*(s1.x - s0.x) - (xP[e[e2][1]] - xP[e[e2][0]])*(s1.z - s0.z);
		(xP[e[e2][1]] - xP[e[e2][0]])*(s1.y - s0.y) - (yP[e[e2][1]] - yP[e[e2][0]])*(s1.x - s0.x);
		//();
		//model.addQConstr(, "c0");
		//model.addQConstr(, "c1");


	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}
}

void resolvePenetration(vec3 rSum,float rConstant, tet sTet, tet rTet, tet *pTet, double *minOptValue, int *minOptIndex, double *totalOptTime) {
	//test 44 find minimum
	double optValue[44] = { 1000.0 };
	double optTime[44] = { 0.0 };
	int contactVertex[4] = { 0 };
	// 4 static faces
	
	(*totalOptTime) = 0;
	(*minOptIndex) = - 1;
	(*minOptValue) = 1000;

	for (int i = 0; i < 4; i++) {
		optStaticFace(i,sTet,rTet,rSum,rConstant,pTet,&optValue[i],&optTime[i]);
		(*totalOptTime) += optTime[i];
		if ((*minOptValue) > optValue[i]) {
			*minOptValue = optValue[i];
			*minOptIndex = i;
		}
	}

	//4 deforming faces and 4 vertices for each case
	/*for (int i = 0; i < 4; i++) {
		int index = i + 4;
		optDeformFace(i,sTet,rTet,rSum,rConstant,pTet,&optValue[index],&optTime[index],&contactVertex[i]);
		(*totalOptTime) += optTime[index];
		if ((*minOptValue) > optValue[index]) {
			*minOptValue = optValue[index];
			*minOptIndex = index;
		}
	}*/

	//36 deforming faces from edge-edge case
	//for (int s = 0; s < 6; s++) { // static edge
	//	for (int p = 0; p < 6; p++) { //deforming edge
	//		optEdgeEdge();
	//	}
	//}

	//find minimum metric value and that case.
}

void printV3(vec3 v) {
	cout << "(" << v.x << "," << v.y << ","<< v.z << ")" << endl;
}
void printResult(tet sTet, tet rTet, tet pTet, double minOptValue, int minOptIndex, double totalOptTime) {
	if (minOptIndex != -1) {

		cout << "[Optimization Result]" << endl;
		cout << "Minimum metric vale: " << minOptValue << endl;
		cout << "Total Optimization time: " << totalOptTime << endl;
		cout << "Separating plane from: " << minOptIndex << endl;

		for (int i = 0; i < 4; i++) {
			cout << "tetStatic: s"<<i<<"="; 
			printV3(sTet.vertex[i]);
		}
		for (int i = 0; i < 4; i++) {
			cout << "tetRest: r" << i << "="; 
			printV3(rTet.vertex[i]);
		}
		for (int i = 0; i < 4; i++) {
			cout << "tetDeformed: p" << i << "="; 
			printV3(pTet.vertex[i]);
		}
		
	}
	else {
		cout << "ERROR: Not Appropriate Tet Deform calculation " << endl;
	}

	if (minOptIndex < 4) {
		cout << "Separating plane from static tetrahedron face " << minOptIndex <<":";
		for (int i = 0; i < 3; i++) {
			printV3(sTet.face[minOptIndex][i]);
		}
	}
	else if (minOptIndex < 8) {
		int index = minOptIndex - 4;

		cout << "Separating Plane from deforming tetrahedron face "<<index <<":";
		for (int i = 0; i < 3; i++) {
			printV3(pTet.face[index][i]);
		}
	}
	else {
		int index = minOptIndex - 8;
		int indexS = index / 6;
		int indexP = index % 6;
		cout << "Separating Plane from static edge :";
		for (int i = 0; i < 2; i++) {
			printV3(sTet.edge[indexS][i]);
		}
		cout << "Separating Plane from deforming edge :";
		for (int i = 0; i < 2; i++) {
			printV3(pTet.edge[indexP][i]);
		}

	}
}
int main(int argc, const char *argv[])
{
	tet sTet; //static tetrahedron
	tet rTet; //rest pose tetrahedron
	tet pTet; //deformed pose tetrahedron
	double totalOptTime;
	double minOptValue;
	int minOptIndex;

	vec3 rSum;
	float rConstant;

	init(&sTet,&rTet,&pTet,&rSum,&rConstant);

	resolvePenetration(rSum,rConstant,sTet, rTet, &pTet, &minOptValue, &minOptIndex, &totalOptTime);

	printResult(sTet, rTet, pTet, minOptValue, minOptIndex, totalOptTime);
	
	return 0;
}