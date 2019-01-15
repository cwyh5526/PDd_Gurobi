#include "DefDefPD.h"

//constructor
DefDefPD::DefDefPD() {
	try {
		//environment and model is allocated in the heap space, need to be deleted after using it.
		env = new GRBEnv();
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
}

//destructor
DefDefPD::~DefDefPD() {
	delete env;
}


void DefDefPD::initTet(tet &T, vec3 v0, vec3 v1, vec3 v2, vec3 v3) {
	T.vertex[0] = v0;
	T.vertex[1] = v1;
	T.vertex[2] = v2;
	T.vertex[3] = v3;

	//each face number represent the vertex that the face does not contain.
	T.face[0][0] = v1;	T.face[0][1] = v2;	T.face[0][2] = v3; //Face 0 has vertex 123
	T.face[1][0] = v0;	T.face[1][1] = v3;	T.face[1][2] = v2; //Face 1 has vertex 023
	T.face[2][0] = v0;	T.face[2][1] = v1;	T.face[2][2] = v3;	//Face 2 has vertex 031
	T.face[3][0] = v0;	T.face[3][1] = v2;	T.face[3][2] = v1; //Face 3 has vertex 012


															   //edge (0,5) edge(1,4) edge (2,3) are pairs. if one edge is realted to the contact, then other pair edge will be included in the constraints.
	T.edge[0][0] = v0;	T.edge[0][1] = v1;
	T.edge[1][0] = v0;	T.edge[1][1] = v2;
	T.edge[2][0] = v0;	T.edge[2][1] = v3;

	T.edge[3][0] = v1;	T.edge[3][1] = v2;
	T.edge[4][0] = v1;	T.edge[4][1] = v3;

	T.edge[5][0] = v2;	T.edge[5][1] = v3;
}

void DefDefPD::initDefault() {

	//static tetrahedron position
	setSTet(vec3(0.0, 0.0, 0.0),
		vec3(1.0, 0.0, 0.0),
		vec3(0.0, 0.0, 1.0),
		vec3(0.0, 1.0, 0.0));

	//rest pose tetrahedron position
	setRTet(vec3(0.2, 0.2, 0.2),
		vec3(-1.2, 0.2, 0.2),
		vec3(0.7, 0.7, 1.2),
		vec3(0.2, 1.2, 0.2));

	//2. Preprocessing
	rVolume = calculateTetVolume(rTet);
	sumConstantCalculation(rTet, rSum, rConstant);
	optimized = false;
	totalOptTime = 0.f;
	return;

}
void DefDefPD::init(tet tetS, tet tetR) {
	//1. Initialize each tetrahedon
	setSTet(tetS);
	setRTet(tetR);

	//2. Preprocessing
	rVolume = calculateTetVolume(rTet);
	sumConstantCalculation(rTet, rSum, rConstant);
	optimized = false;
	totalOptTime = 0.f;

}

// given input sTet and rTet, find penetration depth and resolved position pTet
void DefDefPD::resolveStaticDefPenetration() {

	//4 Static faces and deforming vertex
	for (int i = 0; i < 4; i++) {
		pTetAll.index[i] = i;
		optStaticFace(i, i);
		totalOptTime += pTetAll.optTime[i];
		if (minOptValue >pTetAll.optValue[i]) {
			minOptValue = pTetAll.optValue[i];
			minOptIndex = i;
		}
	}

	//4 deforming faces and 4 vertices for each case
	for (int i = 0; i < 4; i++) {
		int index = i + 4;
		pTetAll.index[index] = index;
		optDeformingFace(i, index);
		totalOptTime += pTetAll.optTime[index];
		if (minOptValue > pTetAll.optValue[index]) {
			minOptValue = pTetAll.optValue[index];
			minOptIndex = index;
		}
	}

	//36 deforming faces from edge-edge case
	for (int s = 0; s < 6; s++) { // static edge
		for (int d = 0; d < 6; d++) { //deforming edge
			int index = s * 6 + d + 8;
			pTetAll.index[index] = index;
			optEdgeEdge(s, d, index);
			totalOptTime += pTetAll.optTime[index];
			if (minOptValue > pTetAll.optValue[index]) {
				minOptValue = pTetAll.optValue[index];
				minOptIndex = index;
			}
		}
	}

	//find minimum metric value and that case.
	pTet = pTetAll.pTets[minOptIndex];
	optimized = true;
	for (int i = 0; i < 44; i++) {
		cout << "***************Index " << i << endl;
		printResult(i);
	}

}


void DefDefPD::optStaticFace(int fIndex, int pairIndex) {
	cout << "\n =========optStaticFace Face " << fIndex << endl;

	try {
		GRBModel model = GRBModel(*env);

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
		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");

		// Set objective
		GRBQuadExpr obj =
			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
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

				+ rConstant) / (60.f*rVolume);

		model.setObjective(obj, GRB_MINIMIZE);


		//Calculate the static normal vector
		glm::vec3 normal;
		double constraintValue = 0.0;
		separatingPlaneCalculation(sTet.face[fIndex], &normal, &constraintValue);

		// Add constraints: 
		// n·(p0-s0)>=0
		// n·(p1-s0)>=0
		// n·(p2-s0)>=0
		// n·(p3-s0)>=0
		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 >= constraintValue, "c0");
		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1 >= constraintValue, "c1");
		model.addConstr(normal.x*xP2 + normal.y * yP2 + normal.z * zP2 >= constraintValue, "c2");
		model.addConstr(normal.x*xP3 + normal.y * yP3 + normal.z * zP3 >= constraintValue, "c3");

		//Optimization
		model.optimize();

		//get Result;
		pTetAll.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		pTetAll.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);

		initTet(pTetAll.pTets[pairIndex],
			vec3(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X)),
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
void DefDefPD::optDeformingFace(int fIndex, int pairIndex) {
	cout << "\n =========optDeforming Face " << fIndex << endl;


	int vIndex[4][4] = { { 1,3,2,0 },{ 0,2,3,1 },{ 0,3,1,2 },{ 0,1,2,3 } };
	try {

		GRBModel model = GRBModel(*env);

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
		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");

		// Set objective
		GRBQuadExpr obj =
			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3

				- xP0*(rSum.x + rTet.vertex[vIndex[fIndex][0]].x)
				- xP1*(rSum.x + rTet.vertex[vIndex[fIndex][1]].x)
				- xP2*(rSum.x + rTet.vertex[vIndex[fIndex][2]].x)
				- xP3*(rSum.x + rTet.vertex[vIndex[fIndex][3]].x)

				- yP0*(rSum.y + rTet.vertex[vIndex[fIndex][0]].y)
				- yP1*(rSum.y + rTet.vertex[vIndex[fIndex][1]].y)
				- yP2*(rSum.y + rTet.vertex[vIndex[fIndex][2]].y)
				- yP3*(rSum.y + rTet.vertex[vIndex[fIndex][3]].y)

				- zP0*(rSum.z + rTet.vertex[vIndex[fIndex][0]].z)
				- zP1*(rSum.z + rTet.vertex[vIndex[fIndex][1]].z)
				- zP2*(rSum.z + rTet.vertex[vIndex[fIndex][2]].z)
				- zP3*(rSum.z + rTet.vertex[vIndex[fIndex][3]].z)

				+ rConstant) / (60.f*rVolume);

		model.setObjective(obj, GRB_MINIMIZE);

		//Calculate the static normal vector
		//n=-r01xr02 : face normal for rest state face
		//d=n dot p0
		glm::vec3 normal;
		double constraintValue = 0.0;
		separatingPlaneCalculation(rTet.face[fIndex], &normal, &constraintValue);
		glm::vec3 separatingDirection = -normal;

		//face index에 따라p도 달라져야 하는데..
		// Add constraints: Face에포함된 vertex를 순서대로 p0 p1 p2라 하고, face에 포함되지 않은 vertex를 p3라 하자. 즉 faceIndex를 가진 vertex 자리에 p3를 넣어주어야 한다.
		// n·(s0-p0)>=0   --->
		// n·(s1-p0)>=0
		// n·(s2-p0)>=0
		// n·(s3-p0)>=0

		//(-n)·(p3-p0)>0 ---> n·(p0-p3)>0

		// n·(p1-p0)=0,
		// n·(p2-p0)=0,
		// n·(p2-p1)=0,
		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[0]), "c0");
		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[1]), "c1");
		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[2]), "c2");
		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 <= dot(normal, sTet.vertex[3]), "c3");

		model.addConstr(normal.x*xP0 + normal.y * yP0 + normal.z * zP0 - (normal.x*xP3 + normal.y * yP3 + normal.z * zP3) >= 0, "c4");

		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1 - (normal.x*xP0 + normal.y * yP0 + normal.z * zP0) == 0, "c5");
		model.addConstr(normal.x*xP2 + normal.y * yP2 + normal.z * zP2 - (normal.x*xP0 + normal.y * yP0 + normal.z * zP0) == 0, "c6");
		model.addConstr(normal.x*xP1 + normal.y * yP1 + normal.z * zP1 - (normal.x*xP2 + normal.y * yP2 + normal.z * zP2) == 0, "c7");


		//Optimization
		model.optimize();

		//get Result;
		pTetAll.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		pTetAll.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);

		vec3 p0(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X));
		vec3 p1(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X));
		vec3 p2(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X));
		vec3 p3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X));
		if (fIndex == 0) { initTet(pTetAll.pTets[pairIndex], p3, p0, p2, p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음.p3=v0, p0=v1 p1=v3 p2=v2
		else if (fIndex == 1) { initTet(pTetAll.pTets[pairIndex], p0, p3, p1, p2); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0,v2=p1,v3=p2
		else if (fIndex == 2) { initTet(pTetAll.pTets[pairIndex], p0, p2, p3, p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v3=p1 v1=p2
		else if (fIndex == 3) { initTet(pTetAll.pTets[pairIndex], p0, p1, p2, p3); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v1=p1 v2=p2
		else { cout << "Wrong Face Index" << endl; }//p3이 not contact, p0p1p2이 face를 이루고 있음.

	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}
}
void DefDefPD::optEdgeEdge(int sIndex, int dIndex, int pairIndex) {
	cout << "\n =========optEdgEdge staticEdge " << sIndex << "deforming Edge" << dIndex << endl;

	int e[6][2] = { { 0,1 },{ 0,2 },{ 0,3 },{ 1,2 },{ 1,3 },{ 2,3 } };

	int d0 = e[dIndex][0];
	int d1 = e[dIndex][1];
	int d2 = e[5 - dIndex][0];
	int d3 = e[5 - dIndex][1];

	vec3 s0 = sTet.edge[sIndex][0];
	vec3 s1 = sTet.edge[sIndex][1];
	vec3 s2 = sTet.edge[5 - sIndex][0];
	vec3 s3 = sTet.edge[5 - sIndex][1];

	vec3 r0 = rTet.edge[dIndex][0];
	vec3 r1 = rTet.edge[dIndex][1];
	vec3 r01 = r1 - r0; //contact edge vector in rest state
	vec3 s01 = s1 - s0; //contact edge vector in static tet.
	vec3 s02 = s2 - s0;//non-contact edge vector in static tet.
	vec3 s03 = s3 - s0;//non-contact edge vector in static tet.
	vec3 n_cross = cross(r01, s01);
	vec3 n = normalize(cross(r01, s01));// normal vector 두 contact edge에 동시에 수직인 벡터, direction should be decided.

	bool nDecided = false;

	//Decide the normal direction	
	float nDotS02 = dot(n, s02);// n·(s2 - s0),
	float nDotS03 = dot(n, s03);// n·(s3 - s0), 

								//cout << "r01 = (" << r01.x << ", " << r01.y << "," << r01.z << ")" << endl;
								//cout << "s01 = (" << s01.x << ", " << s01.y << "," << s01.z << ")" << endl;

								//cout << "s02 = (" << s02.x << ", " << s02.y << "," << s02.z << ")" << endl;
								//cout << "s03 = (" << s03.x << ", " << s03.y << "," << s03.z << ")" << endl;
								//cout << "n = (" << n.x << ", " << n.y << "," << n.z << ")" << endl;
								//cout << "n_cross = (" << n_cross.x << ", " << n_cross.y << "," << n_cross.z << ")" << endl;

								//cout << "n dot s02=" << nDotS02 << endl;
								//cout << "n dot s03=" << nDotS03 << endl;
	if (dot(n_cross, n_cross) == 0) {
		//	when two edges are parallel
		// separation direction should be calculated in different way
		// n is the shortest distance vector from the point r0 to static edge s01.
		vec3 r0s0 = s0 - r0;
		float s01_length = sqrt(dot(s01, s01));
		vec3 direction = normalize(s01);
		vec3 s0h = sqrt(dot(r0s0, r0s0) - dot(cross(direction, -r0s0), cross(direction, -r0s0)))*direction;
		//cout << "r0s0 = (" << r0s0.x << ", " << r0s0.y << "," << r0s0.z << ")" << endl;
		//cout << "s01_length = (" << s01_length << ")" << endl;

		n = r0s0 + s0h;
		n = normalize(n);

		nDecided = false;// both normal has to be tested in this case
						 /*cout<<"WHAT IS GOING ON"<<endl;
						 cout << "n = (" << n.x << ", " << n.y << "," << n.z << ")" << endl;
						 cout << "n_cross = (" << n_cross.x << ", " << n_cross.y << "," << n_cross.z << ")" << endl;*/
	}
	else if ((nDotS02 * nDotS03) < 0) { // n·(s2 - s0) <0 or  n·(s3 - s0)<0
										// if s2, s3 are in different direction, then normal cannot be decided
		nDecided = false;
		cout << "\n\n\n\n[FASLE]: 1. Normal not decided \n\n\n" << endl;
	}
	else if ((nDotS02 >= 0) && (nDotS03 >= 0)) { // n·(s2 - s0) >=0 and  n·(s3 - s0) >=0
												 //if both s2, s3 are in the same direction of normal, then the normal direction should be inverted
		n = -n;
		nDecided = true;
		cout << "\n\n\n\n[true]: 2 . normal Inverted\n\n\n" << endl;
	}
	else if ((nDotS02 <= 0) && (nDotS03 <= 0)) {// n·(s2 - s0) <=0 and  n·(s3 - s0) <=0
												//if both s2, s3 are in the opposite direction of normal, then the normal direction is correct
		nDecided = true;
		cout << "\n\n\n\n[true]: 3. normal unchanged\n\n\n" << endl;
	}
	else {
		nDecided = false; 		cout << "\n\n\n\n[FASLE]: 4. this cannot be reached something wrong\n\n\n" << endl;
	}


	try {
		GRBModel model = GRBModel(*env);

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
		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");
		//cout << "-----------------0----------------------------------------------------------" << endl;

		// Set objective
		GRBQuadExpr obj =
			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3

				- xP0*(rSum.x + rTet.vertex[d0].x)
				- xP1*(rSum.x + rTet.vertex[d1].x)
				- xP2*(rSum.x + rTet.vertex[d2].x)
				- xP3*(rSum.x + rTet.vertex[d3].x)

				- yP0*(rSum.y + rTet.vertex[d0].y)
				- yP1*(rSum.y + rTet.vertex[d1].y)
				- yP2*(rSum.y + rTet.vertex[d2].y)
				- yP3*(rSum.y + rTet.vertex[d3].y)

				- zP0*(rSum.z + rTet.vertex[d0].z)
				- zP1*(rSum.z + rTet.vertex[d1].z)
				- zP2*(rSum.z + rTet.vertex[d2].z)
				- zP3*(rSum.z + rTet.vertex[d3].z)

				+ rConstant) / (60.f*rVolume);

		model.setObjective(obj, GRB_MINIMIZE);
		//cout << "-----------------1----------------------------------------------------------" << endl;

		//Add Constraints

		//1. p0 p1 should be on the same plane
		//   n·(p1 - p0) ==0,
		model.addConstr(n.x*xP1 + n.y*yP1 + n.z*zP1 - (n.x*xP0 + n.y*yP0 + n.z*zP0) == 0, "c1");
		//cout << "----------------2----------------------------------------------------------" << endl;

		//2. p2 p3 should be on the separating direction
		//   n·(p2 - p0) >= 0,
		//   n·(p3 - p0) >= 0,
		model.addConstr(n.x*xP2 + n.y*yP2 + n.z*zP2 - (n.x*xP0 + n.y*yP0 + n.z*zP0) >= 0, "c2");
		model.addConstr(n.x*xP3 + n.y*yP3 + n.z*zP3 - (n.x*xP0 + n.y*yP0 + n.z*zP0) >= 0, "c3");
		//cout << "-----------------3----------------------------------------------------------" << endl;

		//3. s0 s1 s2 s3 should be on the opposite direction of separating direction
		//   n·(s0 - p0) <= 0,
		//   n·(s1 - p0) <= 0,
		//   n·(s2 - p0) <= 0,
		//   n·(s3 - p0) <= 0,
		model.addConstr(dot(n, s0) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c4");
		model.addConstr(dot(n, s1) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c5");
		model.addConstr(dot(n, s2) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c6");
		model.addConstr(dot(n, s3) - (n.x*xP0 + n.y*yP0 + n.z*zP0) <= 0, "c7");
		//cout << "-----------------4----------------------------------------------------------" << endl;

		//Optimization
		model.optimize();
		//cout << "-----------------5----------------------------------------------------------" << endl;

		//Results
		// Time, value, pTet.
		pTetAll.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);
		pTetAll.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		//cout << "-----------------6----------------------------------------------------------" << endl;

		vec3 p0(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X));
		vec3 p1(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X));
		vec3 p2(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X));
		vec3 p3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X));
		//cout << "-----------------7----------------------------------------------------------" << endl;

		switch (dIndex) {
		case 0:initTet(pTetAll.pTets[pairIndex], p0, p1, p2, p3); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
		case 1:initTet(pTetAll.pTets[pairIndex], p0, p2, p1, p3); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
		case 2:initTet(pTetAll.pTets[pairIndex], p0, p2, p3, p1); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
		case 3:initTet(pTetAll.pTets[pairIndex], p2, p0, p1, p3); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
		case 4:initTet(pTetAll.pTets[pairIndex], p2, p0, p3, p1); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
		case 5:initTet(pTetAll.pTets[pairIndex], p2, p3, p0, p1); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
		default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
		}
		if (nDecided == false) {
			cout << "-----------------seconde case----------------------------------------------------------" << endl;

			n = -n; //do the same thing on the opposite direction

			GRBModel model_second = GRBModel(*env);

			// Create variables

			GRBVar xP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
			GRBVar xP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
			GRBVar xP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
			GRBVar xP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");

			GRBVar yP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
			GRBVar yP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
			GRBVar yP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
			GRBVar yP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");

			GRBVar zP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
			GRBVar zP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
			GRBVar zP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
			GRBVar zP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");

			// Set objective
			GRBQuadExpr obj_ =
				(xP0_*xP0_ + xP0_*xP1_ + xP1_*xP1_ + xP0_*xP2_ + xP1_*xP2_ + xP2_*xP2_ + xP0_*xP3_ + xP1_*xP3_ + xP2_*xP3_ + xP3_*xP3_
					+ yP0_*yP0_ + yP0_*yP1_ + yP1_*yP1_ + yP0_*yP2_ + yP1_*yP2_ + yP2_*yP2_ + yP0_*yP3_ + yP1_*yP3_ + yP2_*yP3_ + yP3_*yP3_
					+ zP0_*zP0_ + zP0_*zP1_ + zP1_*zP1_ + zP0_*zP2_ + zP1_*zP2_ + zP2_*zP2_ + zP0_*zP3_ + zP1_*zP3_ + zP2_*zP3_ + zP3_*zP3_

					- xP0_*(rSum.x + rTet.vertex[d0].x)
					- xP1_*(rSum.x + rTet.vertex[d1].x)
					- xP2_*(rSum.x + rTet.vertex[d2].x)
					- xP3_*(rSum.x + rTet.vertex[d3].x)

					- yP0_*(rSum.y + rTet.vertex[d0].y)
					- yP1_*(rSum.y + rTet.vertex[d1].y)
					- yP2_*(rSum.y + rTet.vertex[d2].y)
					- yP3_*(rSum.y + rTet.vertex[d3].y)

					- zP0_*(rSum.z + rTet.vertex[d0].z)
					- zP1_*(rSum.z + rTet.vertex[d1].z)
					- zP2_*(rSum.z + rTet.vertex[d2].z)
					- zP3_*(rSum.z + rTet.vertex[d3].z)

					+ rConstant) / (60.f*rVolume);

			model_second.setObjective(obj_, GRB_MINIMIZE);

			//Add Constraints

			//1. p0 p1 should be on the same plane
			//   n·(p1 - p0) ==0,
			model_second.addConstr(n.x*xP1_ + n.y*yP1_ + n.z*zP1_ - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) == 0, "c1");
			//2. p2 p3 should be on the separating direction
			//   n·(p2 - p0) >= 0,
			//   n·(p3 - p0) >= 0,
			model_second.addConstr(n.x*xP2_ + n.y*yP2_ + n.z*zP2_ - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) >= 0, "c2");
			model_second.addConstr(n.x*xP3_ + n.y*yP3_ + n.z*zP3_ - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) >= 0, "c3");
			//3. s0 s1 s2 s3 should be on the opposite direction of separating direction
			//   n·(s0 - p0) <= 0,
			//   n·(s1 - p0) <= 0,
			//   n·(s2 - p0) <= 0,
			//   n·(s3 - p0) <= 0,
			model_second.addConstr(dot(n, s0) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c4");
			model_second.addConstr(dot(n, s1) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c5");
			model_second.addConstr(dot(n, s2) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c6");
			model_second.addConstr(dot(n, s3) - (n.x*xP0_ + n.y*yP0_ + n.z*zP0_) <= 0, "c7");

			//Optimization
			model_second.optimize();

			//Results
			// Time, value, pTet.
			pTetAll.optTime[pairIndex] += model_second.get(GRB_DoubleAttr_Runtime);
			if (pTetAll.optValue[pairIndex] > model_second.get(GRB_DoubleAttr_ObjVal)) {

				pTetAll.optValue[pairIndex] = model_second.get(GRB_DoubleAttr_ObjVal);

				vec3 p0_(xP0_.get(GRB_DoubleAttr_X), yP0_.get(GRB_DoubleAttr_X), zP0_.get(GRB_DoubleAttr_X));
				vec3 p1_(xP1_.get(GRB_DoubleAttr_X), yP1_.get(GRB_DoubleAttr_X), zP1_.get(GRB_DoubleAttr_X));
				vec3 p2_(xP2_.get(GRB_DoubleAttr_X), yP2_.get(GRB_DoubleAttr_X), zP2_.get(GRB_DoubleAttr_X));
				vec3 p3_(xP3_.get(GRB_DoubleAttr_X), yP3_.get(GRB_DoubleAttr_X), zP3_.get(GRB_DoubleAttr_X));

				switch (dIndex) {
				case 0:initTet(pTetAll.pTets[pairIndex], p0_, p1_, p2_, p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
				case 1:initTet(pTetAll.pTets[pairIndex], p0_, p2_, p1_, p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
				case 2:initTet(pTetAll.pTets[pairIndex], p0_, p2_, p3_, p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
				case 3:initTet(pTetAll.pTets[pairIndex], p2_, p0_, p1_, p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
				case 4:initTet(pTetAll.pTets[pairIndex], p2_, p0_, p3_, p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
				case 5:initTet(pTetAll.pTets[pairIndex], p2_, p3_, p0_, p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
				default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
				}
			}
		}
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

}

void DefDefPD::resolveDefDefPenetration() {
	calculateMidPoint();
	sumConstantCalculation(sTet, sSum, sConstant);

	//4 Deforming faces from first tet
	for (int i = 0; i < 4; i++) {
		pTetAll2.index[i] = i;
		optDefDefFace(i, i);
		totalOptTime += pTetAll2.optTime[i];
		if (minOptValue >pTetAll2.optValue[i]) {
			minOptValue = pTetAll2.optValue[i];
			minOptIndex = i;
		}
	}

	//4 deforming faces from second tet
	for (int i = 0; i < 4; i++) {
		int index = i + 4;
		pTetAll2.index[index] = index;
		optDefDefFace(i, index);
		totalOptTime += pTetAll2.optTime[index];
		if (minOptValue > pTetAll2.optValue[index]) {
			minOptValue = pTetAll2.optValue[index];
			minOptIndex = index;
		}

	}

	//36 deforming faces from edge-edge case
	for (int s = 0; s < 6; s++) {
		for (int d = 0; d < 6; d++) { //deforming edge
			int index = s * 6 + d + 8;
			pTetAll2.index[index] = index;
			optDefDefEdge(s, d, index);
			totalOptTime += pTetAll2.optTime[index];
			if (minOptValue > pTetAll2.optValue[index]) {
				minOptValue = pTetAll2.optValue[index];
				minOptIndex = index;
			}
		}
	}
	//find minimum metric value and that case.
	pTet = pTetAll2.pTets1[minOptIndex];
	pTet2 = pTetAll2.pTets2[minOptIndex];
	optimized = true;

}




void DefDefPD::optDefDefFace(int fIndex, int pairIndex) {
	//calculate normal

	cout << "\n =========optDef Def Face " << fIndex << endl;


	int vIndex[4][4] = { { 1,3,2,0 },{ 0,2,3,1 },{ 0,3,1,2 },{ 0,1,2,3 } };
	try {
		cout << "1" << endl;
		GRBModel model = GRBModel(*env);

		// Create variables
		cout << "2" << endl;
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
		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");

		GRBVar xS0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS0");
		GRBVar xS1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS1");
		GRBVar xS2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS2");
		GRBVar xS3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS3");

		GRBVar yS0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS0");
		GRBVar yS1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS1");
		GRBVar yS2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS2");
		GRBVar yS3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS3");

		GRBVar zS0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS0");
		GRBVar zS1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS1");
		GRBVar zS2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS2");
		GRBVar zS3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS3");
		cout << "3" << endl;
		// Set objective
		GRBQuadExpr obj1 =
			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3

				- xP0*(rSum.x + rTet.vertex[vIndex[fIndex][0]].x)
				- xP1*(rSum.x + rTet.vertex[vIndex[fIndex][1]].x)
				- xP2*(rSum.x + rTet.vertex[vIndex[fIndex][2]].x)
				- xP3*(rSum.x + rTet.vertex[vIndex[fIndex][3]].x)

				- yP0*(rSum.y + rTet.vertex[vIndex[fIndex][0]].y)
				- yP1*(rSum.y + rTet.vertex[vIndex[fIndex][1]].y)
				- yP2*(rSum.y + rTet.vertex[vIndex[fIndex][2]].y)
				- yP3*(rSum.y + rTet.vertex[vIndex[fIndex][3]].y)

				- zP0*(rSum.z + rTet.vertex[vIndex[fIndex][0]].z)
				- zP1*(rSum.z + rTet.vertex[vIndex[fIndex][1]].z)
				- zP2*(rSum.z + rTet.vertex[vIndex[fIndex][2]].z)
				- zP3*(rSum.z + rTet.vertex[vIndex[fIndex][3]].z)

				+ rConstant) / (60.f*rVolume)
			+
			(xS0*xS0 + xS0*xS1 + xS1*xS1 + xS0*xS2 + xS1*xS2 + xS2*xS2 + xS0*xS3 + xS1*xS3 + xS2*xS3 + xS3*xS3
				+ yS0*yS0 + yS0*yS1 + yS1*yS1 + yS0*yS2 + yS1*yS2 + yS2*yS2 + yS0*yS3 + yS1*yS3 + yS2*yS3 + yS3*yS3
				+ zS0*zS0 + zS0*zS1 + zS1*zS1 + zS0*zS2 + zS1*zS2 + zS2*zS2 + zS0*zS3 + zS1*zS3 + zS2*zS3 + zS3*zS3

				- xS0*(sSum.x + sTet.vertex[vIndex[fIndex][0]].x)
				- xS1*(sSum.x + sTet.vertex[vIndex[fIndex][1]].x)
				- xS2*(sSum.x + sTet.vertex[vIndex[fIndex][2]].x)
				- xS3*(sSum.x + sTet.vertex[vIndex[fIndex][3]].x)

				- yS0*(sSum.y + sTet.vertex[vIndex[fIndex][0]].y)
				- yS1*(sSum.y + sTet.vertex[vIndex[fIndex][1]].y)
				- yS2*(sSum.y + sTet.vertex[vIndex[fIndex][2]].y)
				- yS3*(sSum.y + sTet.vertex[vIndex[fIndex][3]].y)

				- zS0*(sSum.z + sTet.vertex[vIndex[fIndex][0]].z)
				- zS1*(sSum.z + sTet.vertex[vIndex[fIndex][1]].z)
				- zS2*(sSum.z + sTet.vertex[vIndex[fIndex][2]].z)
				- zS3*(sSum.z + sTet.vertex[vIndex[fIndex][3]].z)

				+ sConstant) / (60.f*sVolume);
		cout << "4" << endl;
		cout << "rConst:" << rConstant << endl;
		cout << "rSum: (" << rSum.x << "," << rSum.y << "," << rSum.z << ")" << endl;
		cout << "sConst:" << sConstant << endl;
		cout << "sSum: (" << sSum.x << "," << sSum.y << "," << sSum.z << ")" << endl;

		model.setObjective(obj1, GRB_MINIMIZE);
		cout << "5" << endl;
		//Calculate the static normal vector
		//n=-r01xr02 : face normal for rest state face
		//d=n dot p0
		glm::vec3 normal;
		double constraintValue = 0.0;
		vec3 sepDir(0, 0, 0); //separating direction
		if (pairIndex<4) {//def tet1
			cout << "6" << endl;
			separatingPlaneCalculation(sTet.face[fIndex], &normal, &constraintValue);
			sepDir = normal;

		}
		else {
			cout << "7" << endl;
			separatingPlaneCalculation(rTet.face[fIndex], &normal, &constraintValue);
			sepDir = -normal;
		}
		cout << "8" << endl;
		// Add constraints: Face에포함된 vertex를 순서대로 p0 p1 p2라 하고, face에 포함되지 않은 vertex를 p3라 하자. 즉 faceIndex를 가진 vertex 자리에 p3를 넣어주어야 한다.

		//mid point MP가 있다고 하면
		//
		// n·(s0-mp)<=0   --->
		// n·(s1-mp)<=0
		// n·(s2-mp)<=0
		// n·(s3-mp)<=0

		// n·(p0-mp)>=0   --->
		// n·(p1-mp)>=0
		// n·(p2-mp)>=0
		// n·(p3-mp)>=0
		model.addConstr(sepDir.x*xS0 + sepDir.y * yS0 + sepDir.z * zS0 <= dot(sepDir, midPoint), "c0");
		model.addConstr(sepDir.x*xS1 + sepDir.y * yS1 + sepDir.z * zS1 <= dot(sepDir, midPoint), "c1");
		model.addConstr(sepDir.x*xS2 + sepDir.y * yS2 + sepDir.z * zS2 <= dot(sepDir, midPoint), "c2");
		model.addConstr(sepDir.x*xS3 + sepDir.y * yS3 + sepDir.z * zS3 <= dot(sepDir, midPoint), "c3");

		model.addConstr(sepDir.x*xP0 + sepDir.y * yP0 + sepDir.z * zP0 >= dot(sepDir, midPoint), "c4");
		model.addConstr(sepDir.x*xP1 + sepDir.y * yP1 + sepDir.z * zP1 >= dot(sepDir, midPoint), "c5");
		model.addConstr(sepDir.x*xP2 + sepDir.y * yP2 + sepDir.z * zP2 >= dot(sepDir, midPoint), "c6");
		model.addConstr(sepDir.x*xP3 + sepDir.y * yP3 + sepDir.z * zP3 >= dot(sepDir, midPoint), "c7");

		cout << "9" << endl;


		//Optimization
		model.optimize();
		cout << "10" << endl;
		//get Result;
		pTetAll2.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		pTetAll2.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);



		vec3 p0(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X));
		vec3 p1(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X));
		vec3 p2(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X));
		vec3 p3(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X));

		vec3 s0(xS0.get(GRB_DoubleAttr_X), yS0.get(GRB_DoubleAttr_X), zS0.get(GRB_DoubleAttr_X));
		vec3 s1(xS1.get(GRB_DoubleAttr_X), yS1.get(GRB_DoubleAttr_X), zS1.get(GRB_DoubleAttr_X));
		vec3 s2(xS2.get(GRB_DoubleAttr_X), yS2.get(GRB_DoubleAttr_X), zS2.get(GRB_DoubleAttr_X));
		vec3 s3(xS3.get(GRB_DoubleAttr_X), yS3.get(GRB_DoubleAttr_X), zS3.get(GRB_DoubleAttr_X));

		if (fIndex == 0) { initTet(pTetAll2.pTets2[pairIndex], s3, s0, s2, s1); initTet(pTetAll2.pTets1[pairIndex], p3, p0, p2, p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음.p3=v0, p0=v1 p1=v3 p2=v2
		else if (fIndex == 1) { initTet(pTetAll2.pTets2[pairIndex], s0, s3, s1, s2); initTet(pTetAll2.pTets1[pairIndex], p0, p3, p1, p2); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0,v2=p1,v3=p2
		else if (fIndex == 2) { initTet(pTetAll2.pTets2[pairIndex], s0, s2, s3, s1); initTet(pTetAll2.pTets1[pairIndex], p0, p2, p3, p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v3=p1 v1=p2
		else if (fIndex == 3) { initTet(pTetAll2.pTets2[pairIndex], s0, s1, s2, s3); initTet(pTetAll2.pTets1[pairIndex], p0, p1, p2, p3); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v1=p1 v2=p2
		else { cout << "Wrong Face Index" << endl; }//p3이 not contact, p0p1p2이 face를 이루고 있음.

	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

}

void DefDefPD::optDefDefEdge(int sIndex, int dIndex, int pairIndex) {
	cout << "\n =========optEdgEdge staticEdge " << sIndex << "deforming Edge" << dIndex << endl;

	calculateMidPoint();
	sumConstantCalculation(sTet, sSum, sConstant);


	int e[6][2] = { { 0,1 },{ 0,2 },{ 0,3 },{ 1,2 },{ 1,3 },{ 2,3 } };

	int dI0 = e[dIndex][0];
	int dI1 = e[dIndex][1];
	int dI2 = e[5 - dIndex][0];
	int dI3 = e[5 - dIndex][1];
	int sI0 = e[sIndex][0];
	int sI1 = e[sIndex][1];
	int sI2 = e[5 - sIndex][0];
	int sI3 = e[5 - sIndex][1];

	vec3 s0 = sTet.edge[sIndex][0];
	vec3 s1 = sTet.edge[sIndex][1];
	vec3 s2 = sTet.edge[5 - sIndex][0];
	vec3 s3 = sTet.edge[5 - sIndex][1];

	vec3 r0 = rTet.edge[dIndex][0];
	vec3 r1 = rTet.edge[dIndex][1];
	vec3 r01 = r1 - r0; //contact edge vector in rest state
	vec3 s01 = s1 - s0; //contact edge vector in static tet.
	vec3 s02 = s2 - s0;//non-contact edge vector in static tet.
	vec3 s03 = s3 - s0;//non-contact edge vector in static tet.
	vec3 n_cross = cross(r01, s01);
	vec3 n = normalize(cross(r01, s01));// normal vector 두 contact edge에 동시에 수직인 벡터, direction should be decided.

	bool nDecided = false;

	//Decide the normal direction	
	float nDotS02 = dot(n, s02);// n·(s2 - s0),
	float nDotS03 = dot(n, s03);// n·(s3 - s0), 

								//cout << "r01 = (" << r01.x << ", " << r01.y << "," << r01.z << ")" << endl;
								//cout << "s01 = (" << s01.x << ", " << s01.y << "," << s01.z << ")" << endl;

								//cout << "s02 = (" << s02.x << ", " << s02.y << "," << s02.z << ")" << endl;
								//cout << "s03 = (" << s03.x << ", " << s03.y << "," << s03.z << ")" << endl;
								//cout << "n = (" << n.x << ", " << n.y << "," << n.z << ")" << endl;
								//cout << "n_cross = (" << n_cross.x << ", " << n_cross.y << "," << n_cross.z << ")" << endl;

								//cout << "n dot s02=" << nDotS02 << endl;
								//cout << "n dot s03=" << nDotS03 << endl;
	if (dot(n_cross, n_cross) == 0) {
		//	when two edges are parallel
		// separation direction should be calculated in different way
		// n is the shortest distance vector from the point r0 to static edge s01.
		vec3 r0s0 = s0 - r0;
		float s01_length = sqrt(dot(s01, s01));
		vec3 direction = normalize(s01);
		vec3 s0h = sqrt(dot(r0s0, r0s0) - dot(cross(direction, -r0s0), cross(direction, -r0s0)))*direction;
		//cout << "r0s0 = (" << r0s0.x << ", " << r0s0.y << "," << r0s0.z << ")" << endl;
		//cout << "s01_length = (" << s01_length << ")" << endl;

		n = r0s0 + s0h;
		n = normalize(n);

		nDecided = false;// both normal has to be tested in this case
						 /*cout<<"WHAT IS GOING ON"<<endl;
						 cout << "n = (" << n.x << ", " << n.y << "," << n.z << ")" << endl;
						 cout << "n_cross = (" << n_cross.x << ", " << n_cross.y << "," << n_cross.z << ")" << endl;*/
	}
	else if ((nDotS02 * nDotS03) < 0) { // n·(s2 - s0) <0 or  n·(s3 - s0)<0
										// if s2, s3 are in different direction, then normal cannot be decided
		nDecided = false;
		cout << "\n\n\n\n[FASLE]: 1. Normal not decided \n\n\n" << endl;
	}
	else if ((nDotS02 >= 0) && (nDotS03 >= 0)) { // n·(s2 - s0) >=0 and  n·(s3 - s0) >=0
												 //if both s2, s3 are in the same direction of normal, then the normal direction should be inverted
		n = -n;
		nDecided = true;
		cout << "\n\n\n\n[true]: 2 . normal Inverted\n\n\n" << endl;
	}
	else if ((nDotS02 <= 0) && (nDotS03 <= 0)) {// n·(s2 - s0) <=0 and  n·(s3 - s0) <=0
												//if both s2, s3 are in the opposite direction of normal, then the normal direction is correct
		nDecided = true;
		cout << "\n\n\n\n[true]: 3. normal unchanged\n\n\n" << endl;
	}
	else {
		nDecided = false; 		cout << "\n\n\n\n[FASLE]: 4. this cannot be reached something wrong\n\n\n" << endl;
	}


	try {
		GRBModel model = GRBModel(*env);

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
		GRBVar zP3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");


		//cout << "-----------------0----------------------------------------------------------" << endl;

		GRBVar xS0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS0");
		GRBVar xS1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS1");
		GRBVar xS2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS2");
		GRBVar xS3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS3");

		GRBVar yS0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS0");
		GRBVar yS1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS1");
		GRBVar yS2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS2");
		GRBVar yS3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS3");

		GRBVar zS0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS0");
		GRBVar zS1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS1");
		GRBVar zS2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS2");
		GRBVar zS3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS3");

		// Set objective
		GRBQuadExpr obj1 =
			(xP0*xP0 + xP0*xP1 + xP1*xP1 + xP0*xP2 + xP1*xP2 + xP2*xP2 + xP0*xP3 + xP1*xP3 + xP2*xP3 + xP3*xP3
				+ yP0*yP0 + yP0*yP1 + yP1*yP1 + yP0*yP2 + yP1*yP2 + yP2*yP2 + yP0*yP3 + yP1*yP3 + yP2*yP3 + yP3*yP3
				+ zP0*zP0 + zP0*zP1 + zP1*zP1 + zP0*zP2 + zP1*zP2 + zP2*zP2 + zP0*zP3 + zP1*zP3 + zP2*zP3 + zP3*zP3

				- xP0*(rSum.x + rTet.vertex[dI0].x)
				- xP1*(rSum.x + rTet.vertex[dI1].x)
				- xP2*(rSum.x + rTet.vertex[dI2].x)
				- xP3*(rSum.x + rTet.vertex[dI3].x)

				- yP0*(rSum.y + rTet.vertex[dI0].y)
				- yP1*(rSum.y + rTet.vertex[dI1].y)
				- yP2*(rSum.y + rTet.vertex[dI2].y)
				- yP3*(rSum.y + rTet.vertex[dI3].y)

				- zP0*(rSum.z + rTet.vertex[dI0].z)
				- zP1*(rSum.z + rTet.vertex[dI1].z)
				- zP2*(rSum.z + rTet.vertex[dI2].z)
				- zP3*(rSum.z + rTet.vertex[dI3].z)

				+ rConstant) / (60.f*rVolume);

		GRBQuadExpr obj2 =
			(xS0*xS0 + xS0*xS1 + xS1*xS1 + xS0*xS2 + xS1*xS2 + xS2*xS2 + xS0*xS3 + xS1*xS3 + xS2*xS3 + xS3*xS3
				+ yS0*yS0 + yS0*yS1 + yS1*yS1 + yS0*yS2 + yS1*yS2 + yS2*yS2 + yS0*yS3 + yS1*yS3 + yS2*yS3 + yS3*yS3
				+ zS0*zS0 + zS0*zS1 + zS1*zS1 + zS0*zS2 + zS1*zS2 + zS2*zS2 + zS0*zS3 + zS1*zS3 + zS2*zS3 + zS3*zS3

				- xS0*(sSum.x + sTet.vertex[sI0].x)
				- xS1*(sSum.x + sTet.vertex[sI1].x)
				- xS2*(sSum.x + sTet.vertex[sI2].x)
				- xS3*(sSum.x + sTet.vertex[sI3].x)

				- yS0*(sSum.y + sTet.vertex[sI0].y)
				- yS1*(sSum.y + sTet.vertex[sI1].y)
				- yS2*(sSum.y + sTet.vertex[sI2].y)
				- yS3*(sSum.y + sTet.vertex[sI3].y)

				- zS0*(sSum.z + sTet.vertex[sI0].z)
				- zS1*(sSum.z + sTet.vertex[sI1].z)
				- zS2*(sSum.z + sTet.vertex[sI2].z)
				- zS3*(sSum.z + sTet.vertex[sI3].z)

				+ sConstant) / (60.f*sVolume);

		model.setObjective(obj1 + obj2, GRB_MINIMIZE);


		//cout << "-----------------1----------------------------------------------------------" << endl;

		//Add Constraints

		//mid point MP가 있다고 하면
		//
		// n·(s0-mp)<=0   --->
		// n·(s1-mp)<=0
		// n·(s2-mp)<=0
		// n·(s3-mp)<=0

		// n·(p0-mp)>=0   --->
		// n·(p1-mp)>=0
		// n·(p2-mp)>=0
		// n·(p3-mp)>=0
		model.addConstr(n.x*xS0 + n.y * yS0 + n.z * zS0 <= dot(n, midPoint), "c0");
		model.addConstr(n.x*xS1 + n.y * yS1 + n.z * zS1 <= dot(n, midPoint), "c1");
		model.addConstr(n.x*xS2 + n.y * yS2 + n.z * zS2 <= dot(n, midPoint), "c2");
		model.addConstr(n.x*xS3 + n.y * yS3 + n.z * zS3 <= dot(n, midPoint), "c3");

		model.addConstr(n.x*xP0 + n.y * yP0 + n.z * zP0 >= dot(n, midPoint), "c4");
		model.addConstr(n.x*xP1 + n.y * yP1 + n.z * zP1 >= dot(n, midPoint), "c5");
		model.addConstr(n.x*xP2 + n.y * yP2 + n.z * zP2 >= dot(n, midPoint), "c6");
		model.addConstr(n.x*xP3 + n.y * yP3 + n.z * zP3 >= dot(n, midPoint), "c7");



		//Optimization
		model.optimize();
		//cout << "-----------------5----------------------------------------------------------" << endl;

		//Results
		// Time, value, pTet.
		pTetAll2.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);
		pTetAll2.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		//cout << "-----------------6----------------------------------------------------------" << endl;

		vec3 p0_(xP0.get(GRB_DoubleAttr_X), yP0.get(GRB_DoubleAttr_X), zP0.get(GRB_DoubleAttr_X));
		vec3 p1_(xP1.get(GRB_DoubleAttr_X), yP1.get(GRB_DoubleAttr_X), zP1.get(GRB_DoubleAttr_X));
		vec3 p2_(xP2.get(GRB_DoubleAttr_X), yP2.get(GRB_DoubleAttr_X), zP2.get(GRB_DoubleAttr_X));
		vec3 p3_(xP3.get(GRB_DoubleAttr_X), yP3.get(GRB_DoubleAttr_X), zP3.get(GRB_DoubleAttr_X));

		vec3 s0_(xS0.get(GRB_DoubleAttr_X), yS0.get(GRB_DoubleAttr_X), zS0.get(GRB_DoubleAttr_X));
		vec3 s1_(xS1.get(GRB_DoubleAttr_X), yS1.get(GRB_DoubleAttr_X), zS1.get(GRB_DoubleAttr_X));
		vec3 s2_(xS2.get(GRB_DoubleAttr_X), yS2.get(GRB_DoubleAttr_X), zS2.get(GRB_DoubleAttr_X));
		vec3 s3_(xS3.get(GRB_DoubleAttr_X), yS3.get(GRB_DoubleAttr_X), zS3.get(GRB_DoubleAttr_X));

		//cout << "-----------------7----------------------------------------------------------" << endl;

		switch (dIndex) {
		case 0:initTet(pTetAll2.pTets1[pairIndex], p0_, p1_, p2_, p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
		case 1:initTet(pTetAll2.pTets1[pairIndex], p0_, p2_, p1_, p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
		case 2:initTet(pTetAll2.pTets1[pairIndex], p0_, p2_, p3_, p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
		case 3:initTet(pTetAll2.pTets1[pairIndex], p2_, p0_, p1_, p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
		case 4:initTet(pTetAll2.pTets1[pairIndex], p2_, p0_, p3_, p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
		case 5:initTet(pTetAll2.pTets1[pairIndex], p2_, p3_, p0_, p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
		default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
		}
		switch (sIndex) {
		case 0:initTet(pTetAll2.pTets2[pairIndex], s0_, s1_, s2_, p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
		case 1:initTet(pTetAll2.pTets2[pairIndex], s0_, s2_, s1_, p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
		case 2:initTet(pTetAll2.pTets2[pairIndex], s0_, s2_, s3_, p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
		case 3:initTet(pTetAll2.pTets2[pairIndex], s2_, s0_, s1_, p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
		case 4:initTet(pTetAll2.pTets2[pairIndex], s2_, s0_, s3_, p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
		case 5:initTet(pTetAll2.pTets2[pairIndex], s2_, s3_, s0_, p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
		default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
		}
		if (nDecided == false) {
			cout << "-----------------seconde case----------------------------------------------------------" << endl;

			n = -n; //do the same thing on the opposite direction

			GRBModel model_second = GRBModel(*env);

			// Create variables

			GRBVar xP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP0");
			GRBVar xP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP1");
			GRBVar xP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP2");
			GRBVar xP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xP3");

			GRBVar yP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP0");
			GRBVar yP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP1");
			GRBVar yP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP2");
			GRBVar yP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yP3");

			GRBVar zP0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP0");
			GRBVar zP1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP1");
			GRBVar zP2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP2");
			GRBVar zP3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zP3");

			GRBVar xS0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS0");
			GRBVar xS1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS1");
			GRBVar xS2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS2");
			GRBVar xS3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xS3");

			GRBVar yS0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS0");
			GRBVar yS1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS1");
			GRBVar yS2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS2");
			GRBVar yS3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "yS3");

			GRBVar zS0_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS0");
			GRBVar zS1_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS1");
			GRBVar zS2_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS2");
			GRBVar zS3_ = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "zS3");

			// Set objective
			GRBQuadExpr obj_1 =
				(xP0_*xP0_ + xP0_*xP1_ + xP1_*xP1_ + xP0_*xP2_ + xP1_*xP2_ + xP2_*xP2_ + xP0_*xP3_ + xP1_*xP3_ + xP2_*xP3_ + xP3_*xP3_
					+ yP0_*yP0_ + yP0_*yP1_ + yP1_*yP1_ + yP0_*yP2_ + yP1_*yP2_ + yP2_*yP2_ + yP0_*yP3_ + yP1_*yP3_ + yP2_*yP3_ + yP3_*yP3_
					+ zP0_*zP0_ + zP0_*zP1_ + zP1_*zP1_ + zP0_*zP2_ + zP1_*zP2_ + zP2_*zP2_ + zP0_*zP3_ + zP1_*zP3_ + zP2_*zP3_ + zP3_*zP3_

					- xP0_*(rSum.x + rTet.vertex[dI0].x)
					- xP1_*(rSum.x + rTet.vertex[dI1].x)
					- xP2_*(rSum.x + rTet.vertex[dI2].x)
					- xP3_*(rSum.x + rTet.vertex[dI3].x)

					- yP0_*(rSum.y + rTet.vertex[dI0].y)
					- yP1_*(rSum.y + rTet.vertex[dI1].y)
					- yP2_*(rSum.y + rTet.vertex[dI2].y)
					- yP3_*(rSum.y + rTet.vertex[dI3].y)

					- zP0_*(rSum.z + rTet.vertex[dI0].z)
					- zP1_*(rSum.z + rTet.vertex[dI1].z)
					- zP2_*(rSum.z + rTet.vertex[dI2].z)
					- zP3_*(rSum.z + rTet.vertex[dI3].z)

					+ rConstant) / (60.f*rVolume);
			GRBQuadExpr obj_2 =
				(xS0_*xS0_ + xS0_*xS1_ + xS1_*xS1_ + xS0_*xS2_ + xS1_*xS2_ + xS2_*xS2_ + xS0_*xS3_ + xS1_*xS3_ + xS2_*xS3_ + xS3_*xS3_
					+ yS0_*yS0_ + yS0_*yS1_ + yS1_*yS1_ + yS0_*yS2_ + yS1_*yS2_ + yS2_*yS2_ + yS0_*yS3_ + yS1_*yS3_ + yS2_*yS3_ + yS3_*yS3_
					+ zS0_*zS0_ + zS0_*zS1_ + zS1_*zS1_ + zS0_*zS2_ + zS1_*zS2_ + zS2_*zS2_ + zS0_*zS3_ + zS1_*zS3_ + zS2_*zS3_ + zS3_*zS3_

					- xS0_*(rSum.x + rTet.vertex[dI0].x)
					- xS1_*(rSum.x + rTet.vertex[dI1].x)
					- xS2_*(rSum.x + rTet.vertex[dI2].x)
					- xS3_*(rSum.x + rTet.vertex[dI3].x)

					- yS0_*(rSum.y + rTet.vertex[dI0].y)
					- yS1_*(rSum.y + rTet.vertex[dI1].y)
					- yS2_*(rSum.y + rTet.vertex[dI2].y)
					- yS3_*(rSum.y + rTet.vertex[dI3].y)

					- zS0_*(rSum.z + rTet.vertex[dI0].z)
					- zS1_*(rSum.z + rTet.vertex[dI1].z)
					- zS2_*(rSum.z + rTet.vertex[dI2].z)
					- zP3_*(rSum.z + rTet.vertex[dI3].z)

					+ sConstant) / (60.f*sVolume);

			model_second.setObjective(obj_1 + obj_2, GRB_MINIMIZE);

			//Add Constraints

			//Add Constraints

			//mid point MP가 있다고 하면
			//
			// n·(s0-mp)<=0   --->
			// n·(s1-mp)<=0
			// n·(s2-mp)<=0
			// n·(s3-mp)<=0

			// n·(p0-mp)>=0   --->
			// n·(p1-mp)>=0
			// n·(p2-mp)>=0
			// n·(p3-mp)>=0
			model.addConstr(n.x*xS0 + n.y * yS0 + n.z * zS0 <= dot(n, midPoint), "c0");
			model.addConstr(n.x*xS1 + n.y * yS1 + n.z * zS1 <= dot(n, midPoint), "c1");
			model.addConstr(n.x*xS2 + n.y * yS2 + n.z * zS2 <= dot(n, midPoint), "c2");
			model.addConstr(n.x*xS3 + n.y * yS3 + n.z * zS3 <= dot(n, midPoint), "c3");

			model.addConstr(n.x*xP0 + n.y * yP0 + n.z * zP0 >= dot(n, midPoint), "c4");
			model.addConstr(n.x*xP1 + n.y * yP1 + n.z * zP1 >= dot(n, midPoint), "c5");
			model.addConstr(n.x*xP2 + n.y * yP2 + n.z * zP2 >= dot(n, midPoint), "c6");
			model.addConstr(n.x*xP3 + n.y * yP3 + n.z * zP3 >= dot(n, midPoint), "c7");

			//Optimization
			model_second.optimize();

			//Results
			// Time, value, pTet.
			pTetAll2.optTime[pairIndex] += model_second.get(GRB_DoubleAttr_Runtime);
			if (pTetAll2.optValue[pairIndex] > model_second.get(GRB_DoubleAttr_ObjVal)) {

				pTetAll2.optValue[pairIndex] = model_second.get(GRB_DoubleAttr_ObjVal);

				vec3 p0__(xP0_.get(GRB_DoubleAttr_X), yP0_.get(GRB_DoubleAttr_X), zP0_.get(GRB_DoubleAttr_X));
				vec3 p1__(xP1_.get(GRB_DoubleAttr_X), yP1_.get(GRB_DoubleAttr_X), zP1_.get(GRB_DoubleAttr_X));
				vec3 p2__(xP2_.get(GRB_DoubleAttr_X), yP2_.get(GRB_DoubleAttr_X), zP2_.get(GRB_DoubleAttr_X));
				vec3 p3__(xP3_.get(GRB_DoubleAttr_X), yP3_.get(GRB_DoubleAttr_X), zP3_.get(GRB_DoubleAttr_X));

				vec3 s0__(xS0_.get(GRB_DoubleAttr_X), yS0_.get(GRB_DoubleAttr_X), zS0_.get(GRB_DoubleAttr_X));
				vec3 s1__(xS1_.get(GRB_DoubleAttr_X), yS1_.get(GRB_DoubleAttr_X), zS1_.get(GRB_DoubleAttr_X));
				vec3 s2__(xS2_.get(GRB_DoubleAttr_X), yS2_.get(GRB_DoubleAttr_X), zS2_.get(GRB_DoubleAttr_X));
				vec3 s3__(xS3_.get(GRB_DoubleAttr_X), yS3_.get(GRB_DoubleAttr_X), zS3_.get(GRB_DoubleAttr_X));

				switch (dIndex) {
				case 0:initTet(pTetAll2.pTets1[pairIndex], p0_, p1_, p2_, p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
				case 1:initTet(pTetAll2.pTets1[pairIndex], p0_, p2_, p1_, p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
				case 2:initTet(pTetAll2.pTets1[pairIndex], p0_, p2_, p3_, p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
				case 3:initTet(pTetAll2.pTets1[pairIndex], p2_, p0_, p1_, p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
				case 4:initTet(pTetAll2.pTets1[pairIndex], p2_, p0_, p3_, p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
				case 5:initTet(pTetAll2.pTets1[pairIndex], p2_, p3_, p0_, p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
				default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
				}
				switch (sIndex) {
				case 0:initTet(pTetAll2.pTets2[pairIndex], s0_, s1_, s2_, s3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
				case 1:initTet(pTetAll2.pTets2[pairIndex], s0_, s2_, s1_, s3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
				case 2:initTet(pTetAll2.pTets2[pairIndex], s0_, s2_, s3_, s1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
				case 3:initTet(pTetAll2.pTets2[pairIndex], s2_, s0_, s1_, s3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
				case 4:initTet(pTetAll2.pTets2[pairIndex], s2_, s0_, s3_, s1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
				case 5:initTet(pTetAll2.pTets2[pairIndex], s2_, s3_, s0_, s1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
				default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
				}
			}
		}
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

}












void DefDefPD::calculateMidPoint() {
	//calculate mid point from two tets
	vec3 sum(0, 0, 0);
	for (int i = 0; i < 4; i++) {
		sum += rTet.vertex[i] + sTet.vertex[i];
	}
	midPoint = vec3(sum.x / 8, sum.y / 8, sum.z / 8);
	cout << "midPoint= (" << sum.x << "," << sum.y << "," << sum.z << ")" << endl;
}

void DefDefPD::printV3(vec3 v) {
	cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
}
void DefDefPD::printResult(int pairIndex) {

	tet pTet = pTetAll.pTets[pairIndex];
	double minOptValue = pTetAll.optValue[pairIndex];
	double optTime = pTetAll.optTime[pairIndex];

	int minOptIndex = pairIndex;
	if (minOptIndex != -1) {

		cout << endl;
		cout << "[Optimization Result]" << endl;
		cout << "1. Minimum metric value: " << minOptValue << endl;
		cout << "2. Optimization time for this pair:" << optTime << endl;
		cout << "3. Total Optimization time: " << totalOptTime << endl;
		cout << "4. Separating plane from: " << minOptIndex << endl;

		//cout << "\n1) tetStatic:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "s" << i << "=";
			printV3(sTet.vertex[i]);
		}

		//cout << "\n2) tetRest:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "r" << i << "=";
			printV3(rTet.vertex[i]);
		}
		//cout << "\n3) tetDeformed:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "p" << i << "=";
			printV3(pTet.vertex[i]);
		}

	}
	else {
		cout << "ERROR: Not Appropriate Tet Deform calculation " << endl;
	}

	if (minOptIndex < 4) {
		cout << "5. Separating plane from static tetrahedron face " << minOptIndex << ":" << endl;;
		for (int i = 0; i < 3; i++) {
			printV3(sTet.face[minOptIndex][i]);
		}
	}
	else if (minOptIndex < 8) {
		int index = minOptIndex - 4;

		cout << "5. Separating Plane from deforming tetrahedron face " << index << ":" << endl;;
		for (int i = 0; i < 3; i++) {
			printV3(pTet.face[index][i]);
		}
	}
	else {
		int index = minOptIndex - 8;
		int indexS = index / 6;
		int indexP = index % 6;
		cout << "5. Separating Plane from static edge :" << indexS;
		for (int i = 0; i < 2; i++) {
			printV3(sTet.edge[indexS][i]);
		}
		cout << "  Separating Plane from deforming edge :" << indexP;
		for (int i = 0; i < 2; i++) {
			printV3(pTet.edge[indexP][i]);
		}

	}
	cout << "6. Volume Changes :" << endl;
	float rVolume = calculateTetVolume(rTet);
	float pVolume = calculateTetVolume(pTet);
	cout << "Rest Volume :" << rVolume << "--> Deformed Volume : " << pVolume << endl;

}
void DefDefPD::printResult2(int pairIndex) {

	tet pTet1 = pTetAll2.pTets1[pairIndex];
	tet pTet2 = pTetAll2.pTets2[pairIndex];

	double minOptValue = pTetAll2.optValue[pairIndex];
	double optTime = pTetAll2.optTime[pairIndex];

	int minOptIndex = pairIndex;
	if (minOptIndex != -1) {

		cout << endl;
		cout << "[Optimization Result]" << endl;
		cout << "1. Minimum metric value: " << minOptValue << endl;
		cout << "2. Optimization time for this pair:" << optTime << endl;
		cout << "3. Total Optimization time: " << totalOptTime << endl;
		cout << "4. Separating plane from: " << minOptIndex << endl;

		//cout << "\n1) tetStatic:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "s" << i << "=";
			printV3(sTet.vertex[i]);
		}

		//cout << "\n2) tetRest:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "r" << i << "=";
			printV3(rTet.vertex[i]);
		}
		//cout << "\n3) tetDeformed1:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "p1_" << i << "=";
			printV3(pTet1.vertex[i]);
		}
		//cout << "\n3) tetDeformed2:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "p2_" << i << "=";
			printV3(pTet2.vertex[i]);
		}

	}
	else {
		cout << "ERROR: Not Appropriate Tet Deform calculation " << endl;
	}

	if (minOptIndex < 4) {
		cout << "5. Separating plane from first def tetrahedron face " << minOptIndex << ":" << endl;;
		for (int i = 0; i < 3; i++) {
			printV3(pTet2.face[minOptIndex][i]);
		}
	}
	else if (minOptIndex < 8) {
		int index = minOptIndex - 4;

		cout << "5. Separating Plane from second deforming tetrahedron face " << index << ":" << endl;;
		for (int i = 0; i < 3; i++) {
			printV3(pTet1.face[index][i]);
		}
	}
	else {
		int index = minOptIndex - 8;
		int indexS = index / 6;
		int indexP = index % 6;
		cout << "5. Separating Plane from static edge :" << indexS;
		for (int i = 0; i < 2; i++) {
			printV3(pTet2.edge[indexS][i]);
		}
		cout << "  Separating Plane from deforming edge :" << indexP;
		for (int i = 0; i < 2; i++) {
			printV3(pTet1.edge[indexP][i]);
		}

	}
	cout << "6. Volume Changes :" << endl;
	float rVolume = calculateTetVolume(rTet);
	float p1Volume = calculateTetVolume(pTet1);
	float sVolume = calculateTetVolume(sTet);
	float p2Volume = calculateTetVolume(pTet2);
	cout << "Rest Volume :" << rVolume << "--> Deformed Volume : " << p1Volume << endl;
	cout << "Rest Volume :" << sVolume << "--> Deformed Volume : " << p2Volume << endl;


}
float DefDefPD::calculateTetVolume(tet t) {
	float volume = abs(dot((t.vertex[0] - t.vertex[3]), cross(t.vertex[1] - t.vertex[3], t.vertex[2] - t.vertex[3])) / 6.0f);
	cout << "tet's volume: " << volume << endl;
	if (volume == 0) {
		cout << "Volume is Zero!" << endl;
		(volume = 0.0000000000000000001);
	}
	return volume;
}

void DefDefPD::sumConstantCalculation(tet t, vec3 &sum, float &constant) {

	//precompute the sum of coordinate values of rest pose tet. 
	sum = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++) {
		sum.x += t.vertex[i].x;
		sum.z += t.vertex[i].z;
		sum.y += t.vertex[i].y;
	}

	//precompute constant term of the objective function
	vec3 constant_ = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < (i + 1); j++) {
			constant_.z += t.vertex[i].z * t.vertex[j].z;
			constant_.x += t.vertex[i].x * t.vertex[j].x;
			constant_.y += t.vertex[i].y * t.vertex[j].y;
		}
	}
	constant = (float)(constant_.x + constant_.y + constant_.z);
}


void DefDefPD::separatingPlaneCalculation(vec3 faceVrtx[3], glm::vec3 *normal, double *d) {
	//compute the normal vector and constant of the plane equation for 3 vertices
	glm::vec3 v01 = faceVrtx[1] - faceVrtx[0];
	glm::vec3 v02 = faceVrtx[2] - faceVrtx[0];
	std::cout << "v01: (" << (v01).x << "," << (v01).y << "," << (v01).z << ")" << std::endl;
	std::cout << "v02: (" << (v02).x << "," << (v02).y << "," << (v02).z << ")" << std::endl;

	(*normal) = glm::normalize(glm::cross(v01, v02));
	*(d) = glm::dot(*normal, faceVrtx[0]);
}

