///* Copyright 2018, Gurobi Optimization, LLC */
//
///* This example formulates and solves the following simple QP model:
//
//minimize    x^2 + x*y + y^2 + y*z + z^2 + 2 x
//subject to  x + 2 y + 3 z >= 4
//x +   y       >= 1
//
//It solves it once as a continuous model, and once as an integer model.
//*/
//
//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp> //for matrices
//#include <glm/gtc/type_ptr.hpp>
//
//#include "gurobi_c++.h"
//
//
//
//
//using namespace std;
//
//glm::vec3 sT[4];	//static tetrahedron position
//glm::vec3 pT[4];	//deformed pose tetrahedron position
//glm::vec3 rT[4];	//rest pose tetrahedron position
//
//
//void init() {
//	//static tetrahedron position
//	sT[0] = glm::vec3(0.0, 0.0, 0.0);
//	sT[1] = glm::vec3(1.0, 0.0, 0.0);
//	sT[2] = glm::vec3(0.0, 1.0, 0.0);
//	sT[3] = glm::vec3(0.0, 0.0, 1.0);
//
//	//rest pose tetrahedron position
//	rT[0] = glm::vec3(0.2, 0.2, 0.2);
//	rT[1] = glm::vec3(1.2, 0.2, 0.2);
//	rT[2] = glm::vec3(0.2, 1.2, 0.2);
//	rT[3] = glm::vec3(0.7, 0.7, 1.2);
//
//	//deformed pose tetrahedron position same as the rest pose for the initial value
//	pT[0] = rT[0];
//	pT[1] = rT[1];
//	pT[2] = rT[2];
//	pT[3] = rT[3];
//
//	
//}
//
//
//void faceCalculation(glm::vec3 face[3], glm::vec3 *normal, double *d) {
//	glm::vec3 v01 = face[1] - face[0];
//	glm::vec3 v02 = face[2] - face[0];
//	std::cout << "v01: (" << (v01).x << "," << (v01).y << "," << (v01).z << ")" << std::endl;
//	std::cout << "v02: (" << (v02).x << "," << (v02).y << "," << (v02).z << ")" << std::endl;
//
//	(*normal) = glm::normalize(glm::cross(v01, v02));
//	std::cout << "normal: (" << (*normal).x << "," << (*normal).y << "," << (*normal).z << ")" << std::endl;
//
//	double nDotF = glm::dot(*normal, face[0]);
//	d[0] = nDotF - glm::dot(*normal, rT[0]); // a(s_1x - r_0x) + b(s_1y - r_0y) + c(s_1z - r_0z)
//	d[1] = nDotF - glm::dot(*normal, rT[1]); // a(s_1x - r_1x) + b(s_1y - r_1y) + c(s_1z - r_1z)
//	d[2] = nDotF - glm::dot(*normal, rT[2]); // a(s_1x - r_2x) + b(s_1y - r_2y) + c(s_1z - r_2z)
//	d[3] = nDotF - glm::dot(*normal, rT[3]); // a(s_1x - r_3x) + b(s_1y - r_3y) + c(s_1z - r_3z)
//}
//
//
//
//int main(int argc, char *argv[])
//{
//	init();
//
//	glm::vec3 face[3] = { sT[1], sT[2], sT[3] };
//	glm::vec3 normal;
//	double constraintValue[4] = { 0.0 };
//
//	faceCalculation(face, &normal, constraintValue);
//
//
//	try {
//		GRBEnv env = GRBEnv();
//
//		GRBModel model = GRBModel(env);
//
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
//			x0*x0 + x0*x1 + x1*x1 + x0*x2 + x1*x2 + x2*x2 + x0*x3 + x1*x3 + x2*x3 + x3*x3
//			+ y0*y0 + y0*y1 + y1*y1 + y0*y2 + y1*y2 + y2*y2 + y0*y3 + y1*y3 + y2*y3 + y3*y3
//			+ z0*z0 + z0*z1 + z1*z1 + z0*z2 + z1*z2 + z2*z2 + z0*z3 + z1*z3 + z2*z3 + z3*z3;
//
//		// Add constraint: 
//		//		    a (x_0) + b (y_0) + c (z_0) >= a(s_1x - r_0x) + b(s_1y - r_0y) + c(s_1z - r_0z)
//		//			a(x_1) + b(y_1) + c(z_1) >= a(s_1x - r_1x) + b(s_1y - r_1y) + c(s_1z - r_1z)
//		//			a(x_2) + b(y_2) + c(z_2) >= a(s_1x - r_2x) + b(s_1y - r_2y) + c(s_1z - r_2z)
//		//			a(x_3) + b(y_3) + c(z_3) >= a(s_1x - r_3x) + b(s_1y - r_3y) + c(s_1z - r_3z)
//
//		model.addConstr(normal.x*x0 + normal.y * y0 + normal.z * z0 >= constraintValue[0], "c0");
//		model.addConstr(normal.x*x1 + normal.y * y1 + normal.z * z1 >= constraintValue[1], "c1");
//		model.addConstr(normal.x*x2 + normal.y * y2 + normal.z * z2 >= constraintValue[2], "c2");
//		model.addConstr(normal.x*x3 + normal.y * y3 + normal.z * z3 >= constraintValue[3], "c3");
//
//		
//		double tm =0.0;
//
//		// Optimize model
//		//for(int i=0;i<100;i++){
//			model.optimize();
//			tm += model.get(GRB_DoubleAttr_Runtime);
//		//}
//		cout << endl;
//		cout << "p[0]=("
//			 << x0.get(GRB_StringAttr_VarName) << " " << x0.get(GRB_DoubleAttr_X) + rT[0].x << ","
//			 << y0.get(GRB_StringAttr_VarName) << " " << y0.get(GRB_DoubleAttr_X) + rT[0].y << ","
//			 << z0.get(GRB_StringAttr_VarName) << " " << z0.get(GRB_DoubleAttr_X) + rT[0].z << ")"<< endl;
//
//		cout << "p[1]=("
//			 << x1.get(GRB_StringAttr_VarName) << " " << x1.get(GRB_DoubleAttr_X) + rT[1].x << ","
//			 << y1.get(GRB_StringAttr_VarName) << " " << y1.get(GRB_DoubleAttr_X) + rT[1].y << ","
//			 << z1.get(GRB_StringAttr_VarName) << " " << z1.get(GRB_DoubleAttr_X) + rT[1].z << ")" << endl;
//
//		cout << "p[2]=("
//			 << x2.get(GRB_StringAttr_VarName) << " " << x2.get(GRB_DoubleAttr_X) + rT[2].x << ","
//			 << y2.get(GRB_StringAttr_VarName) << " " << y2.get(GRB_DoubleAttr_X) + rT[2].y << ","
//			 << z2.get(GRB_StringAttr_VarName) << " " << z2.get(GRB_DoubleAttr_X) + rT[2].z << ")" << endl;
//
//		cout << "p[3]=("
//			 << x3.get(GRB_StringAttr_VarName) << " " << x3.get(GRB_DoubleAttr_X) + rT[3].x << ","
//			 << y3.get(GRB_StringAttr_VarName) << " " << y3.get(GRB_DoubleAttr_X) + rT[3].y << ","
//			 << z3.get(GRB_StringAttr_VarName) << " " << z3.get(GRB_DoubleAttr_X) + rT[3].z << ")" << endl;
//		 
//
//		cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//		cout << "Time:" << tm << endl;
//		//// Change variable types to integer
//
//		//x.set(GRB_CharAttr_VType, GRB_INTEGER);
//		//y.set(GRB_CharAttr_VType, GRB_INTEGER);
//		//z.set(GRB_CharAttr_VType, GRB_INTEGER);
//
//		//// Optimize model
//
//		//model.optimize();
//
//		//cout << x.get(GRB_StringAttr_VarName) << " "
//		//	<< x.get(GRB_DoubleAttr_X) << endl;
//		//cout << y.get(GRB_StringAttr_VarName) << " "
//		//	<< y.get(GRB_DoubleAttr_X) << endl;
//		//cout << z.get(GRB_StringAttr_VarName) << " "
//		//	<< z.get(GRB_DoubleAttr_X) << endl;
//
//		//cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//
//	}
//	catch (GRBException e) {
//		cout << "Error code = " << e.getErrorCode() << endl;
//		cout << e.getMessage() << endl;
//	}
//	catch (...) {
//		cout << "Exception during optimization" << endl;
//	}
//
//	return 0;
//}
