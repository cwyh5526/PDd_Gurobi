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
//#include "FullTest_p.h"
//
//#define STATIC_FACE 100
//#define DEFORM_FACE 101
//#define EDGE_EDGE	102
//
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
//
//void init(tet *tetS, tet *tetR, tet *tetP) {
//	
//	cout << "default? Y/N" << endl;
//	char ans = 'Y';
//	cin >> ans;
//	if (ans == 'Y' || ans == 'y') {
//		//static tetrahedron position
//		initTet(tetS, vec3(0.0, 0.0, 0.0),
//			vec3(1.0, 0.0, 0.0),
//			vec3(0.0, 0.0, 1.0),
//			vec3(0.0, 1.0, 0.0));
//
//		//rest pose tetrahedron position
//		initTet(tetR, vec3(0.2, 0.2, 0.2),
//			vec3(-1.2, 0.2, 0.2),
//			vec3(0.7, 0.7, 1.2),
//			vec3(0.2, 1.2, 0.2));
//		//initTet(tetR,   vec3(-1.2, 0.2, 0.2),
//		//				vec3(0.2, 0.2, 0.2),
//		//				vec3(-0.3, 0.7, 1.2),
//		//				vec3(-0.8, 1.2, 0.2));
//		//deformed pose tetrahedron position same as the rest pose for the initial value
//		initTet(tetP, tetR->vertex[0],
//			tetR->vertex[1],
//			tetR->vertex[2],
//			tetR->vertex[3]);
//		return;
//	}
//	/*else {
//		vec3 s0, s1, s2, s3;
//		vec3 r0, r1, r2, r3;
//
//		cout << "enter s0 s1 s2 s3" << endl;
//		cin >> s0.x >> s0.y >> s0.z;
//		cin >> s1.x >> s1.y >> s1.z;
//		cin >> s2.x >> s2.y >> s2.z;
//		cin >> s3.x >> s3.y >> s3.z;
//		cout << "enter r0 r1 r2 r3" << endl;
//		cin >> r0.x >> r0.y >> r0.z;
//		cin >> r1.x >> r1.y >> r1.z;
//		cin >> r2.x >> r2.y >> r2.z;
//		cin >> r3.x >> r3.y >> r3.z;
//
//		initTet(tetS, s0, s1, s2, s3);
//		initTet(tetR, r0, r1, r2, r3);
//		initTet(tetP, r0, r1, r2, r3);
//
//
//	}*/
//
//	
//}
//void calculateTetVolume(tet t, float *volume) {
//	(*volume) = abs(dot((t.vertex[0] - t.vertex[3]), cross(t.vertex[1] - t.vertex[3], t.vertex[2] - t.vertex[3])) / 6.0f);	
//	cout << "restTet's volume: " << (*volume) << endl;
//	if (*volume == 0) {
//		cout << "Rest State Volume is Zero!" << endl;;
//		(*volume = 1);
//	}
//}
//
//void rSumrConstantCalculation(tet tetR, vec3 *rSum, float *rConstant) {
//
//	//precompute the sum of coordinate values of rest pose tet. 
//	(*rSum) = vec3(0.0, 0.0, 0.0);
//	for (int i = 0; i < 4; i++) {
//		rSum->x += tetR.vertex[i].x;
//		rSum->z += tetR.vertex[i].z;
//		rSum->y += tetR.vertex[i].y;
//	}
//
//	//precompute constant term of the objective function
//	vec3 constant = vec3(0.0, 0.0, 0.0);
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < (i + 1); j++) {
//			constant.z += tetR.vertex[i].z * tetR.vertex[j].z;
//			constant.x += tetR.vertex[i].x * tetR.vertex[j].x;
//			constant.y += tetR.vertex[i].y * tetR.vertex[j].y;
//		}
//	}
//	(*rConstant) = (float)(constant.x + constant.y + constant.z);
//}
//
//
//void separatingPlaneCalculation(vec3 faceVrtx[3], glm::vec3 *normal, double *d) {
//	//compute the normal vector and constant of the plane equation for 3 vertices
//	glm::vec3 v01 = faceVrtx[1] - faceVrtx[0];
//	glm::vec3 v02 = faceVrtx[2] - faceVrtx[0];
//	//std::cout << "v01: (" << (v01).x << "," << (v01).y << "," << (v01).z << ")" << std::endl;
//	//std::cout << "v02: (" << (v02).x << "," << (v02).y << "," << (v02).z << ")" << std::endl;
//
//	(*normal) = glm::normalize(glm::cross(v01, v02));
//	*(d) = glm::dot(*normal, faceVrtx[0]);
//}
//
//
//void optStaticFace(int fIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue,  double *optTime) {
//	cout << "\n =========optStaticFace Face "<< fIndex << endl;
//	float volume=1.0f;
//	calculateTetVolume(rTet, &volume);
//
//	try {
//		GRBEnv env = GRBEnv();
//
//		GRBModel model = GRBModel(env);
//
//		// Create variables
//		
//		GRBVar xP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
//		GRBVar xP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
//		GRBVar xP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
//		GRBVar xP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");
//
//		GRBVar yP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
//		GRBVar yP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
//		GRBVar yP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
//		GRBVar yP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");
//
//
//		GRBVar zP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
//		GRBVar zP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
//		GRBVar zP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
//		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");
//
//		// Set objective
//		GRBQuadExpr obj =
//			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
//				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
//				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3
//
//				- xP0*(rSum.x + rTet.vertex[0].x)
//				- xP1*(rSum.x + rTet.vertex[1].x)
//				- xP2*(rSum.x + rTet.vertex[2].x)
//				- xP3*(rSum.x + rTet.vertex[3].x)
//
//				- yP0*(rSum.y + rTet.vertex[0].y)
//				- yP1*(rSum.y + rTet.vertex[1].y)
//				- yP2*(rSum.y + rTet.vertex[2].y)
//				- yP3*(rSum.y + rTet.vertex[3].y)
//
//				- zP0*(rSum.z + rTet.vertex[0].z)
//				- zP1*(rSum.z + rTet.vertex[1].z)
//				- zP2*(rSum.z + rTet.vertex[2].z)
//				- zP3*(rSum.z + rTet.vertex[3].z)
//
//				+ rConstant)/ (60.f*volume);
//					
//		model.setObjective(obj, GRB_MINIMIZE);
//
//		//Calculate the static normal vector
//		glm::vec3 normal;
//		double constraintValue = 0.0 ;
//		separatingPlaneCalculation(sTet.face[fIndex], &normal, &constraintValue);
//
//		// Add constraints: 
//		// n·(p0-s0)>=0
//		// n·(p1-s0)>=0
//		// n·(p2-s0)>=0
//		// n·(p3-s0)>=0
//		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 >= constraintValue, "c0");
//		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1 >= constraintValue, "c1");
//		model.addConstr(normal.x*xP2 + normal.y * yP2 + normal.z * zP2 >= constraintValue, "c2");
//		model.addConstr(normal.x*xP3 + normal.y * yP3 + normal.z * zP3 >= constraintValue, "c3");
//
//		//Optimization
//		model.optimize();	
//
//		//get Result;
//		*optValue = model.get(GRB_DoubleAttr_ObjVal);
//		*optTime = model.get(GRB_DoubleAttr_Runtime);
//		
//		initTet(pTet, vec3(xP0.get(GRB_DoubleAttr_X),yP0.get(GRB_DoubleAttr_X),zP0.get(GRB_DoubleAttr_X)), 
//			vec3(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X)),
//			vec3(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X)),
//			vec3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X)));
//	}
//	catch (GRBException e) {
//		cout << "Error code = " << e.getErrorCode() << endl;
//		cout << e.getMessage() << endl;
//	}
//	catch (...) {
//		cout << "Exception during optimization" << endl;
//	}
//}
//void optDeformingFace(int fIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime) {
//	cout<<"\n =========optDeforming Face "<<fIndex<<endl;
//	float volume=1.0f;
//	calculateTetVolume(rTet, &volume);
//
//	int vIndex[4][4] = { {1,3,2,0},{0,2,3,1},{0,3,1,2},{0,1,2,3} };
//	try {
//		GRBEnv env = GRBEnv();
//
//		GRBModel model = GRBModel(env);
//
//		// Create variables
//
//		GRBVar xP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
//		GRBVar xP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
//		GRBVar xP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
//		GRBVar xP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");
//
//		GRBVar yP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
//		GRBVar yP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
//		GRBVar yP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
//		GRBVar yP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");
//
//
//		GRBVar zP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
//		GRBVar zP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
//		GRBVar zP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
//		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");
//
//		// Set objective
//		GRBQuadExpr obj =
//			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
//				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
//				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3
//
//				- xP0*(rSum.x + rTet.vertex[vIndex[fIndex][0]].x)
//				- xP1*(rSum.x + rTet.vertex[vIndex[fIndex][1]].x)
//				- xP2*(rSum.x + rTet.vertex[vIndex[fIndex][2]].x)
//				- xP3*(rSum.x + rTet.vertex[vIndex[fIndex][3]].x)
//
//				- yP0*(rSum.y + rTet.vertex[vIndex[fIndex][0]].y)
//				- yP1*(rSum.y + rTet.vertex[vIndex[fIndex][1]].y)
//				- yP2*(rSum.y + rTet.vertex[vIndex[fIndex][2]].y)
//				- yP3*(rSum.y + rTet.vertex[vIndex[fIndex][3]].y)
//
//				- zP0*(rSum.z + rTet.vertex[vIndex[fIndex][0]].z)
//				- zP1*(rSum.z + rTet.vertex[vIndex[fIndex][1]].z)
//				- zP2*(rSum.z + rTet.vertex[vIndex[fIndex][2]].z)
//				- zP3*(rSum.z + rTet.vertex[vIndex[fIndex][3]].z)
//
//				+ rConstant) / (60.f*volume);
//
//		model.setObjective(obj, GRB_MINIMIZE);
//
//		//Calculate the static normal vector
//		//n=-r01xr02 : face normal for rest state face
//		//d=n dot p0
//		glm::vec3 normal;
//		double constraintValue = 0.0;
//		separatingPlaneCalculation(rTet.face[fIndex], &normal, &constraintValue);
//		glm::vec3 separatingDirection = -normal;
//		
//		//face index에 따라p도 달라져야 하는데..
//		// Add constraints: Face에포함된 vertex를 순서대로 p0 p1 p2라 하고, face에 포함되지 않은 vertex를 p3라 하자. 즉 faceIndex를 가진 vertex 자리에 p3를 넣어주어야 한다.
//		// n·(s0-p0)>=0   --->
//		// n·(s1-p0)>=0
//		// n·(s2-p0)>=0
//		// n·(s3-p0)>=0
//	
//		//(-n)·(p3-p0)>0 ---> n·(p0-p3)>0
//
//		// n·(p1-p0)=0,
//		// n·(p2-p0)=0,
//		// n·(p2-p1)=0,
//		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal,sTet.vertex[0]), "c0");
//		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[1]), "c1");
//		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[2]), "c2");
//		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[3]), "c3");
//
//		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 -(normal.x*xP3 + normal.y * yP3 + normal.z * zP3) >= 0, "c4");
//
//		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1-(normal.x*xP0 + normal.y * yP0 + normal.z * zP0) ==0, "c5");
//		model.addConstr(normal.x*xP2 + normal.y * yP2 + normal.z * zP2 - (normal.x*xP0 + normal.y * yP0 + normal.z * zP0) == 0, "c6");
//		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1 - (normal.x*xP2 + normal.y * yP2 + normal.z * zP2) == 0, "c7");
//	
//
//
//		//Optimization
//		model.optimize();
//
//		//get Result;
//		*optValue = model.get(GRB_DoubleAttr_ObjVal);
//		*optTime = model.get(GRB_DoubleAttr_Runtime);
//		//faceIndex에 따라서 어디 넣어주느냐가 달라지는거 아니냐
//		//0이면 
//
//
//		vec3 p0(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X));
//		vec3 p1(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X));
//		vec3 p2(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X));
//		vec3 p3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X));
//		if (fIndex == 0)	  { initTet(pTet, p3, p0, p2, p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음.p3=v0, p0=v1 p1=v3 p2=v2
//		else if (fIndex == 1) { initTet(pTet, p0, p3, p1, p2); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0,v2=p1,v3=p2
//		else if (fIndex == 2) { initTet(pTet, p0, p2, p3, p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v3=p1 v1=p2
//		else if (fIndex == 3) { initTet(pTet, p0, p1, p2 ,p3); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v1=p1 v2=p2
//		else { cout << "Wrong Face Index" << endl; }//p3이 not contact, p0p1p2이 face를 이루고 있음.
//		
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
//void optEdgeEdge(int sIndex, int dIndex, tet sTet, tet rTet, vec3 rSum, float rConstant, tet *pTet, double *optValue, double *optTime) {
//	cout << "\n =========optEdgEdge staticEdge " << sIndex << "deforming Edge" << dIndex << endl;
//
//	int e[6][2] = { {0,1},{0,2},{0,3},{1,2},{1,3},{2,3} };
//	
//	int d0 = e[dIndex][0];
//	int d1 = e[dIndex][1];
//	int d2 = e[5-dIndex][0];
//	int d3 = e[5-dIndex][1];
//
//	vec3 s0 = sTet.edge[sIndex][0];
//	vec3 s1 = sTet.edge[sIndex][1];
//	vec3 s2 = sTet.edge[5 - sIndex][0];
//	vec3 s3 = sTet.edge[5 - sIndex][1];
//
//	vec3 r0 = rTet.edge[dIndex][0];
//	vec3 r1 = rTet.edge[dIndex][1];
//	vec3 r01 = r1-r0; //contact edge vector in rest state
//	vec3 s01 = s1-s0; //contact edge vector in static tet.
//	vec3 s02 = s2-s0;//non-contact edge vector in static tet.
//	vec3 s03 = s3-s0;//non-contact edge vector in static tet.
//	vec3 n_cross = cross(r01, s01);
//	vec3 n = normalize(cross(r01, s01));// normal vector 두 contact edge에 동시에 수직인 벡터, direction should be decided.
//
//	bool nDecided = false;
//
//	//Decide the normal direction	
//	float nDotS02 = dot(n, s02);// n·(s2 - s0),
//	float nDotS03 = dot(n, s03);// n·(s3 - s0), 
//
//	//cout << "r01 = (" << r01.x << ", " << r01.y << "," << r01.z << ")" << endl;
//	//cout << "s01 = (" << s01.x << ", " << s01.y << "," << s01.z << ")" << endl;
//
//	//cout << "s02 = (" << s02.x << ", " << s02.y << "," << s02.z << ")" << endl;
//	//cout << "s03 = (" << s03.x << ", " << s03.y << "," << s03.z << ")" << endl;
//	//cout << "n = (" << n.x << ", " << n.y << "," << n.z << ")" << endl;
//	//cout << "n_cross = (" << n_cross.x << ", " << n_cross.y << "," << n_cross.z << ")" << endl;
//
//	//cout << "n dot s02=" << nDotS02 << endl;
//	//cout << "n dot s03=" << nDotS03 << endl;
//	if (dot(n_cross,n_cross)==0) {
//		//	when two edges are parallel
//		// separation direction should be calculated in different way
//		// n is the shortest distance vector from the point r0 to static edge s01.
//		vec3 r0s0 = s0 - r0;
//		float s01_length = sqrt(dot(s01,s01));
//		vec3 direction = normalize(s01);
//		vec3 s0h = sqrt(dot(r0s0,r0s0)-dot(cross(direction,-r0s0), cross(direction, -r0s0)))*direction;
//		//cout << "r0s0 = (" << r0s0.x << ", " << r0s0.y << "," << r0s0.z << ")" << endl;
//		//cout << "s01_length = (" << s01_length << ")" << endl;
//
//		n = r0s0 + s0h;
//		n = normalize(n);
//
//		nDecided = false;// both normal has to be tested in this case
//		/*cout<<"WHAT IS GOING ON"<<endl;
//		cout << "n = (" << n.x << ", " << n.y << "," << n.z << ")" << endl;
//		cout << "n_cross = (" << n_cross.x << ", " << n_cross.y << "," << n_cross.z << ")" << endl;*/
//	}
//	else if ((nDotS02 * nDotS03) < 0) { // n·(s2 - s0) <0 or  n·(s3 - s0)<0
//		// if s2, s3 are in different direction, then normal cannot be decided
//		nDecided = false;
//		cout << "\n\n\n\n[FASLE]: 1. Normal not decided \n\n\n" << endl;
//	}
//	else if ((nDotS02 >= 0) && (nDotS03 >= 0)) { // n·(s2 - s0) >=0 and  n·(s3 - s0) >=0
//		//if both s2, s3 are in the same direction of normal, then the normal direction should be inverted
//		n = -n;
//		nDecided = true;
//		cout << "\n\n\n\n[true]: 2 . normal Inverted\n\n\n" << endl;
//	}
//	else if((nDotS02 <= 0) && (nDotS03 <= 0)){// n·(s2 - s0) <=0 and  n·(s3 - s0) <=0
//		//if both s2, s3 are in the opposite direction of normal, then the normal direction is correct
//		nDecided = true;
//		cout << "\n\n\n\n[true]: 3. normal unchanged\n\n\n" << endl;
//	}
//	else { 
//		nDecided = false ; 		cout << "\n\n\n\n[FASLE]: 4. this cannot be reached something wrong\n\n\n" << endl;
//	}
//
//	
//	float volume=1.0f;
//	calculateTetVolume(rTet, &volume);
//
//	try {
//		GRBEnv env = GRBEnv();
//
//		GRBModel model = GRBModel(env);
//
//		// Create variables
//
//		GRBVar xP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
//		GRBVar xP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
//		GRBVar xP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
//		GRBVar xP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");
//
//		GRBVar yP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
//		GRBVar yP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
//		GRBVar yP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
//		GRBVar yP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");
//
//		GRBVar zP0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
//		GRBVar zP1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
//		GRBVar zP2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
//		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");
//		//cout << "-----------------0----------------------------------------------------------" << endl;
//
//		// Set objective
//		GRBQuadExpr obj =
//			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
//				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
//				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3
//
//				- xP0*(rSum.x + rTet.vertex[d0].x)
//				- xP1*(rSum.x + rTet.vertex[d1].x)
//				- xP2*(rSum.x + rTet.vertex[d2].x)
//				- xP3*(rSum.x + rTet.vertex[d3].x)
//
//				- yP0*(rSum.y + rTet.vertex[d0].y)
//				- yP1*(rSum.y + rTet.vertex[d1].y)
//				- yP2*(rSum.y + rTet.vertex[d2].y)
//				- yP3*(rSum.y + rTet.vertex[d3].y)
//
//				- zP0*(rSum.z + rTet.vertex[d0].z)
//				- zP1*(rSum.z + rTet.vertex[d1].z)
//				- zP2*(rSum.z + rTet.vertex[d2].z)
//				- zP3*(rSum.z + rTet.vertex[d3].z)
//
//				+ rConstant)/ (60.f*volume);
//
//		model.setObjective(obj,GRB_MINIMIZE);
//		//cout << "-----------------1----------------------------------------------------------" << endl;
//
//		//Add Constraints
//
//		//1. p0 p1 should be on the same plane
//		//   n·(p1 - p0) ==0,
//		model.addConstr(n.x*xP1 + n.y*yP1 + n.z*zP1 - (n.x*xP0 + n.y*yP0 + n.z*zP0) == 0, "c1");
//		//cout << "----------------2----------------------------------------------------------" << endl;
//
//		//2. p2 p3 should be on the separating direction
//		//   n·(p2 - p0) >= 0,
//		//   n·(p3 - p0) >= 0,
//		model.addConstr(n.x*xP2 + n.y*yP2 + n.z*zP2 - (n.x*xP0 + n.y*yP0 + n.z*zP0) >= 0, "c2");
//		model.addConstr(n.x*xP3 + n.y*yP3 + n.z*zP3 - (n.x*xP0 + n.y*yP0 + n.z*zP0) >= 0, "c3");
//		//cout << "-----------------3----------------------------------------------------------" << endl;
//
//		//3. s0 s1 s2 s3 should be on the opposite direction of separating direction
//		//   n·(s0 - p0) <= 0,
//		//   n·(s1 - p0) <= 0,
//		//   n·(s2 - p0) <= 0,
//		//   n·(s3 - p0) <= 0,
//		model.addConstr(dot(n, s0) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c4");
//		model.addConstr(dot(n, s1) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c5");
//		model.addConstr(dot(n, s2) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c6");
//		model.addConstr(dot(n, s3) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c7");
//		//cout << "-----------------4----------------------------------------------------------" << endl;
//
//		//Optimization
//		model.optimize();
//		//cout << "-----------------5----------------------------------------------------------" << endl;
//
//		//Results
//		// Time, value, pTet.
//		*optTime = model.get(GRB_DoubleAttr_Runtime);
//		*optValue = model.get(GRB_DoubleAttr_ObjVal);
//		//cout << "-----------------6----------------------------------------------------------" << endl;
//
//		vec3 p0(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X));
//		vec3 p1(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X));
//		vec3 p2(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X));
//		vec3 p3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X));		
//		//cout << "-----------------7----------------------------------------------------------" << endl;
//
//		switch (dIndex) {
//		case 0:initTet(pTet, p0, p1, p2, p3); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
//		case 1:initTet(pTet, p0, p2, p1, p3); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
//		case 2:initTet(pTet, p0, p2, p3, p1); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
//		case 3:initTet(pTet, p2, p0, p1, p3); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
//		case 4:initTet(pTet, p2, p0, p3, p1); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
//		case 5:initTet(pTet, p2, p3, p0, p1); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
//		default: cout<<"[EdgeEdge Result]Wrong Edge Index \n"<<endl; break;
//		}
//		if (nDecided == false) {
//			cout << "-----------------seconde case----------------------------------------------------------" << endl;
//
//			n = -n; //do the same thing on the opposite direction
//
//			GRBModel model_second = GRBModel(env);
//
//			// Create variables
//
//			GRBVar xP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
//			GRBVar xP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
//			GRBVar xP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
//			GRBVar xP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");
//					  
//			GRBVar yP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
//			GRBVar yP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
//			GRBVar yP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
//			GRBVar yP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");
//					  
//			GRBVar zP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
//			GRBVar zP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
//			GRBVar zP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
//			GRBVar zP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");
//
//			// Set objective
//			GRBQuadExpr obj_ =
//				 ( xP0_*xP0_ + xP0_*xP1_ + xP1_*xP1_ + xP0_*xP2_ + xP1_*xP2_ + xP2_*xP2_ + xP0_*xP3_ + xP1_*xP3_ + xP2_*xP3_ + xP3_*xP3_
//				+ yP0_*yP0_ + yP0_*yP1_ + yP1_*yP1_ + yP0_*yP2_ + yP1_*yP2_ + yP2_*yP2_ + yP0_*yP3_ + yP1_*yP3_ + yP2_*yP3_ + yP3_*yP3_
//				+ zP0_*zP0_ + zP0_*zP1_ + zP1_*zP1_ + zP0_*zP2_ + zP1_*zP2_ + zP2_*zP2_ + zP0_*zP3_ + zP1_*zP3_ + zP2_*zP3_ + zP3_*zP3_
//
//				- xP0_*(rSum.x + rTet.vertex[d0].x)
//				- xP1_*(rSum.x + rTet.vertex[d1].x)
//				- xP2_*(rSum.x + rTet.vertex[d2].x)
//				- xP3_*(rSum.x + rTet.vertex[d3].x)
//
//				- yP0_*(rSum.y + rTet.vertex[d0].y)
//				- yP1_*(rSum.y + rTet.vertex[d1].y)
//				- yP2_*(rSum.y + rTet.vertex[d2].y)
//				- yP3_*(rSum.y + rTet.vertex[d3].y)
//					 
//				- zP0_*(rSum.z + rTet.vertex[d0].z)
//				- zP1_*(rSum.z + rTet.vertex[d1].z)
//				- zP2_*(rSum.z + rTet.vertex[d2].z)
//				- zP3_*(rSum.z + rTet.vertex[d3].z)
//
//				+ rConstant)/(60.f*volume);
//
//			model_second.setObjective(obj_, GRB_MINIMIZE);
//
//			//Add Constraints
//
//			//1. p0 p1 should be on the same plane
//			//   n·(p1 - p0) ==0,
//			model_second.addConstr(n.x*xP1_ + n.y*yP1_ + n.z*zP1_ - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) == 0, "c1");
//			//2. p2 p3 should be on the separating direction
//			//   n·(p2 - p0) >= 0,
//			//   n·(p3 - p0) >= 0,
//			model_second.addConstr(n.x*xP2_ + n.y*yP2_ + n.z*zP2_ - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) >= 0, "c2");
//			model_second.addConstr(n.x*xP3_ + n.y*yP3_ + n.z*zP3_ - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) >= 0, "c3");
//			//3. s0 s1 s2 s3 should be on the opposite direction of separating direction
//			//   n·(s0 - p0) <= 0,
//			//   n·(s1 - p0) <= 0,
//			//   n·(s2 - p0) <= 0,
//			//   n·(s3 - p0) <= 0,
//			model_second.addConstr(dot(n, s0) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c4");
//			model_second.addConstr(dot(n, s1) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c5");
//			model_second.addConstr(dot(n, s2) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c6");
//			model_second.addConstr(dot(n, s3) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c7");
//
//			//Optimization
//			model_second.optimize();
//
//			//Results
//			// Time, value, pTet.
//			*optTime += model_second.get(GRB_DoubleAttr_Runtime);
//			if(*optValue > model_second.get(GRB_DoubleAttr_ObjVal))	{
//
//				*optValue = model_second.get(GRB_DoubleAttr_ObjVal);
//
//				vec3 p0_(xP0_.get(GRB_DoubleAttr_X), yP0_.get(GRB_DoubleAttr_X), zP0_.get(GRB_DoubleAttr_X));
//				vec3 p1_(xP1_.get(GRB_DoubleAttr_X), yP1_.get(GRB_DoubleAttr_X), zP1_.get(GRB_DoubleAttr_X));
//				vec3 p2_(xP2_.get(GRB_DoubleAttr_X), yP2_.get(GRB_DoubleAttr_X), zP2_.get(GRB_DoubleAttr_X));
//				vec3 p3_(xP3_.get(GRB_DoubleAttr_X), yP3_.get(GRB_DoubleAttr_X), zP3_.get(GRB_DoubleAttr_X));
//
//				switch (dIndex) {
//				case 0:initTet(pTet, p0_, p1_, p2_, p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
//				case 1:initTet(pTet, p0_, p2_, p1_, p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
//				case 2:initTet(pTet, p0_, p2_, p3_, p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
//				case 3:initTet(pTet, p2_, p0_, p1_, p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
//				case 4:initTet(pTet, p2_, p0_, p3_, p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
//				case 5:initTet(pTet, p2_, p3_, p0_, p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
//				default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
//				}
//			}			
//		}
//	}
//	catch (GRBException e) {
//		cout << "Error code = " << e.getErrorCode() << endl;
//		cout << e.getMessage() << endl;
//	}
//	catch (...) {
//		cout << "Exception during optimization" << endl;
//	}
//
//}
//
//void printV3(vec3 v) {
//	cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
//}
//void printResult(tet sTet, tet rTet, tet pTet, double minOptValue, int minOptIndex, double totalOptTime) {
//	if (minOptIndex != -1) {
//
//		cout << endl;
//		cout << "[Optimization Result]" << endl;
//		cout << "1. Minimum metric value: " << minOptValue << endl;
//		cout << "2. Total Optimization time: " << totalOptTime << endl;
//		cout << "3. Separating plane from: " << minOptIndex << endl;
//
//		//cout << "\n1) tetStatic:" << endl;
//		for (int i = 0; i < 4; i++) {
//			cout << "s" << i << "=";
//			printV3(sTet.vertex[i]);
//		}
//
//		//cout << "\n2) tetRest:" << endl;
//		for (int i = 0; i < 4; i++) {
//			cout << "r" << i << "=";
//			printV3(rTet.vertex[i]);
//		}
//		//cout << "\n3) tetDeformed:" << endl;
//		for (int i = 0; i < 4; i++) {
//			cout << "p" << i << "=";
//			printV3(pTet.vertex[i]);
//		}
//
//	}
//	else {
//		cout << "ERROR: Not Appropriate Tet Deform calculation " << endl;
//	}
//
//	if (minOptIndex < 4) {
//		cout << "4. Separating plane from static tetrahedron face " << minOptIndex << ":" << endl;;
//		for (int i = 0; i < 3; i++) {
//			printV3(sTet.face[minOptIndex][i]);
//		}
//	}
//	else if (minOptIndex < 8) {
//		int index = minOptIndex - 4;
//
//		cout << "4. Separating Plane from deforming tetrahedron face " << index << ":" << endl;;
//		for (int i = 0; i < 3; i++) {
//			printV3(pTet.face[index][i]);
//		}
//	}
//	else {
//		int index = minOptIndex - 8;
//		int indexS = index / 6;
//		int indexP = index % 6;
//		cout << "4. Separating Plane from static edge :" << indexS ;
//		for (int i = 0; i < 2; i++) {
//			printV3(sTet.edge[indexS][i]);
//		}
//		cout << "  Separating Plane from deforming edge :" << indexP; 
//		for (int i = 0; i < 2; i++) {
//			printV3(pTet.edge[indexP][i]);
//		}
//
//	}
//	cout << "5. Volume Changes :" << endl;
//	float rVolume = 0.f;
//	float pVolume = 0.f;
//	calculateTetVolume(rTet, &rVolume);
//	calculateTetVolume(pTet, &pVolume);
//	cout << "Rest Volume :" << rVolume << "--> Deformed Volume : " << pVolume << endl;
//
//}
//void resolvePenetration(tet sTet, tet rTet, tet *pTet,  float *minOptValue, int *minOptIndex, float *totalOptTime, optResults *all ) {
//	//test 44 find minimum
//	//tet tempPTet[44];
//	//double optValue[44] = { 1000.0 };
//	//double optTime[44] = { 0.0 };
//
//	// 4 static faces
//	
//	(*totalOptTime) = 0;
//	(*minOptIndex) = - 1;
//	(*minOptValue) = 1000;
//
//	vec3 rSum;
//	float rConstant;
//	
//	rSumrConstantCalculation(rTet, &rSum, &rConstant);
//
//	//4 Static faces and deforming vertex
//	for (int i = 0; i < 4; i++) {
//		(*all).index[i] = i;
//		optStaticFace(i,sTet,rTet,rSum,rConstant, &(*all).pTets[i],&(*all).optValue[i],&(*all).optTime[i]);
//		(*totalOptTime) += (*all).optTime[i];
//		if ((*minOptValue) >(*all).optValue[i]) {
//			*minOptValue = (*all).optValue[i];
//			*minOptIndex = i;
//		}
//	}
//
//	//4 deforming faces and 4 vertices for each case
//	for (int i = 0; i < 4; i++) {
//		int index = i + 4;
//		(*all).index[index] = index;
//		optDeformingFace(i, sTet, rTet, rSum, rConstant, &(*all).pTets[index], &(*all).optValue[index], &(*all).optTime[index]);// , &contactVertex[i]);
//		(*totalOptTime) += (*all).optTime[index];
//		if ((*minOptValue) >(*all).optValue[index]) {
//			*minOptValue = (*all).optValue[index];
//			*minOptIndex = index;
//		}
//	}
//
//	//36 deforming faces from edge-edge case
//	for (int s = 0; s < 6; s++) { // static edge
//		for (int d = 0; d < 6; d++) { //deforming edge
//			int index = s * 6 + d + 8;
//			(*all).index[index] = index;
//
//			optEdgeEdge(s, d, sTet, rTet, rSum, rConstant, &(*all).pTets[index], &(*all).optValue[index], &(*all).optTime[index]);
//			(*totalOptTime) += (*all).optTime[index];
//			if ((*minOptValue) >(*all).optValue[index]) {
//				*minOptValue = (*all).optValue[index];
//				*minOptIndex = index;
//			}
//		}
//	}
//
//	//find minimum metric value and that case.
//	(*pTet) = (*all).pTets[*minOptIndex];
//
//	for (int i = 0; i < 44; i++) {
//		cout << "***************Index " << i << endl;
//		printResult(sTet, rTet, (*all).pTets[i], (*all).optValue[i], i, (*all).optTime[i]);
//	}
//
//}
