#include "DefDefPD.h"

//constructor
DefDefPD::DefDefPD() {
	try {
		//environment and model is allocated in the heap space, need to be deleted after using it.
		env = new GRBEnv();
		env->set(GRB_IntParam_OutputFlag, 0);
		numOpt = 0;
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
	setRTet(0,vec3(0.0, 0.0, 0.0),
		vec3(1.0, 0.0, 0.0),
		vec3(0.0, 1.0, 0.0),
		vec3(0.0, 0.0, 1.0));

	//rest pose tetrahedron position
	setRTet(1,vec3(0.2, 0.2, 0.2),
		vec3(-1.2, 0.2, 0.2),
		vec3(0.2, 1.2, 0.2),
		vec3(0.7, 0.7, 1.2));

	//2. Preprocessing
	for (int i = 0; i < 2; i++) {
		rVolume[i] = calculateTetVolume(rTet[i]);
		sumConstantCalculation(rTet[i], rSum[i], rConstant[i]);
		//printV3(rSum[i]);
		//cout << rConstant[i] << endl;
	}

	calculateMidPoint();

	optimized = false;
	minOptValue = 1000.f;
	minOptIndex = -1;
	totalOptTime = 0.f;
	return;

}
void DefDefPD::init(tet tet1, tet tet2) {
	//1. Initialize each tetrahedon
	setRTet(0,tet1);
	setRTet(1,tet2);

	//2. Preprocessing
	for (int i = 0; i < 2; i++) {
		rVolume[i] = calculateTetVolume(rTet[i]);
		sumConstantCalculation(rTet[i], rSum[i], rConstant[i]);
	}
	calculateMidPoint();

	optimized = false;
	minOptValue = 1000.f;
	minOptIndex = -1;

	totalOptTime = 0.f;

}

// given input sTet and rTet, find penetration depth and resolved position pTet

void DefDefPD::resolveDefDefPenetration() {
	
	//4 Deforming faces from first tet
	clock_t start = clock();
//#pragma omp parallel for
	for (int i = 0; i < 4; i++) {
		//pTetAll2.index[i] = i;
		optDefDefFace(i, i);
		/*totalOptTime += pTetAll2.optTime[i];
		if (minOptValue >pTetAll2.optValue[i]) {
			minOptValue = pTetAll2.optValue[i];
			minOptIndex = i;
		}*/
	}

	//4 deforming faces from second tet
//#pragma omp parallel for
	for (int i = 0; i < 4; i++) {
		int index = i + 4;
		//pTetAll2.index[index] = index;
		optDefDefFace(i, index);
		/*totalOptTime += pTetAll2.optTime[index];
		if (minOptValue > pTetAll2.optValue[index]) {
			minOptValue = pTetAll2.optValue[index];
			minOptIndex = index;
		}*/

	}

	//36 deforming faces from edge-edge case
//#pragma omp parallel for
	for (int s = 0; s < 6; s++) {
		for (int d = 0; d < 6; d++) { //deforming edge
			int index = s * 6 + d + 8;
			//pTetAll2.index[index] = index;
			optDefDefEdge(s, d, index);
			
		}
	}

	for (int i = 0; i < 44; i++) {
		pTetAll2.index[i] = i;
		totalOptTime += pTetAll2.optTime[i];
		if (minOptValue > pTetAll2.optValue[i]) {
			minOptValue = pTetAll2.optValue[i];
			minOptIndex = i;
		}
	}
	//find minimum metric value and that case.
	pTet[0] = pTetAll2.pTets1[minOptIndex];
	pTet[1] = pTetAll2.pTets2[minOptIndex];
	optPlanePoint = pTetAll2.planePoint[minOptIndex];
	optimized = true;
	numOpt++;
	clock_t end = clock();
	double time = (double)(end - start) / (double)CLOCKS_PER_SEC;
	//cout << time << endl;
}

//mask[44]: contains sortedIndex of rigid PD calculation result
//limit : decides how many directions to be tested?
void DefDefPD::resolveDefDefPenetrationWithCulling(int mask[44], int limit) {
//#pragma omp parallel for
	for (int i = 0; i < limit; i++) {
		int index = mask[i];
		if (index < 4) {
			//1st tet face case

			optDefDefFace(index, index);
		}
		else if (index < 8) {
			//2nd tet face cases
			pTetAll2.index[index] = index;
			optDefDefFace(index - 4, index);
		}
		else {
			//edge edge casese
			int s = (index - 8) / 6;
			int d = (index - 8) % 6;
			pTetAll2.index[index] = index;
			optDefDefEdge(s, d, index);
		}


	}

	for (int i = 0; i < limit; i++) {
		int index = mask[i];
		pTetAll2.index[index] = index;
		totalOptTime += pTetAll2.optTime[index];
		if (minOptValue > pTetAll2.optValue[index]) {
			minOptValue = pTetAll2.optValue[index];
			minOptIndex = index;
		}
	}
	//find minimum metric value and that case.
	pTet[0] = pTetAll2.pTets1[minOptIndex];
	pTet[1] = pTetAll2.pTets2[minOptIndex];
	optPlanePoint = pTetAll2.planePoint[minOptIndex];
	optimized = true;
	numOpt++;
}

void DefDefPD::resolveDefDefPenetrationWithCulling(int index) {
	if (index < 4) {
		//1st tet face case
		optDefDefFace(index, index);
	}
	else if (index < 8) {
		//2nd tet face cases
		
		optDefDefFace(index - 4, index);
	}
	else if(index<44) {
		//edge edge casese
		int s = (index - 8) / 6;
		int d = (index - 8) % 6;
		optDefDefEdge(s, d, index);
	}
	else {

	}
	totalOptTime = pTetAll2.optTime[index];
	minOptValue = pTetAll2.optValue[index];
	minOptIndex = index;
	pTet[0] = pTetAll2.pTets1[minOptIndex];
	pTet[1] = pTetAll2.pTets2[minOptIndex];
	optPlanePoint = pTetAll2.planePoint[minOptIndex];
	optimized = true;
	numOpt++;

}

void DefDefPD::optDefDefFace(int fIndex, int pairIndex) {
	//calculate normal

	//cout << "\n =========optDef Def Face " << fIndex << endl;


	int vIndex[4][4] = { { 1,2,3,0 },{ 0,3,2,1 },{ 0,1,3,2 },{ 0,2,1,3 } };
	try {
		//cout << "1" << endl;
		GRBModel model = GRBModel(*env);

		// Create variables
		//cout << "2" << endl;
		Var t1[4];
		Var t2[4];
		Var mp;

		mp.x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPx");
		mp.y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPy");
		mp.z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPz");

		t1[0].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0x");
		t1[0].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0y");
		t1[0].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0z");

		t1[1].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1x");
		t1[1].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1y");
		t1[1].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1z");

		t1[2].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2x");
		t1[2].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2y");
		t1[2].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2z");

		t1[3].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3x");
		t1[3].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3y");
		t1[3].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3z");


		t2[0].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0x");
		t2[0].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0y");
		t2[0].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0z");

		t2[1].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1x");
		t2[1].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1y");
		t2[1].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1z");

		t2[2].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2x");
		t2[2].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2y");
		t2[2].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2z");

		t2[3].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3x");
		t2[3].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3y");
		t2[3].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3z");

		//cout << "3" << endl;
		// Set objective
		GRBQuadExpr obj =
			(t1[0].x*t1[0].x + t1[0].x*t1[1].x + t1[1].x*t1[1].x + t1[0].x*t1[2].x + t1[1].x*t1[2].x + t1[2].x*t1[2].x + t1[0].x*t1[3].x + t1[1].x*t1[3].x + t1[2].x*t1[3].x + t1[3].x*t1[3].x
				+ t1[0].y*t1[0].y + t1[0].y*t1[1].y + t1[1].y*t1[1].y + t1[0].y*t1[2].y + t1[1].y*t1[2].y + t1[2].y*t1[2].y + t1[0].y*t1[3].y + t1[1].y*t1[3].y + t1[2].y*t1[3].y + t1[3].y*t1[3].y
				+ t1[0].z*t1[0].z + t1[0].z*t1[1].z + t1[1].z*t1[1].z + t1[0].z*t1[2].z + t1[1].z*t1[2].z + t1[2].z*t1[2].z + t1[0].z*t1[3].z + t1[1].z*t1[3].z + t1[2].z*t1[3].z + t1[3].z*t1[3].z

				- t1[0].x*(rSum[0].x + rTet[0].vertex[vIndex[fIndex][0]].x)
				- t1[1].x*(rSum[0].x + rTet[0].vertex[vIndex[fIndex][1]].x)
				- t1[2].x*(rSum[0].x + rTet[0].vertex[vIndex[fIndex][2]].x)
				- t1[3].x*(rSum[0].x + rTet[0].vertex[vIndex[fIndex][3]].x)

				- t1[0].y*(rSum[0].y + rTet[0].vertex[vIndex[fIndex][0]].y)
				- t1[1].y*(rSum[0].y + rTet[0].vertex[vIndex[fIndex][1]].y)
				- t1[2].y*(rSum[0].y + rTet[0].vertex[vIndex[fIndex][2]].y)
				- t1[3].y*(rSum[0].y + rTet[0].vertex[vIndex[fIndex][3]].y)

				- t1[0].z*(rSum[0].z + rTet[0].vertex[vIndex[fIndex][0]].z)
				- t1[1].z*(rSum[0].z + rTet[0].vertex[vIndex[fIndex][1]].z)
				- t1[2].z*(rSum[0].z + rTet[0].vertex[vIndex[fIndex][2]].z)
				- t1[3].z*(rSum[0].z + rTet[0].vertex[vIndex[fIndex][3]].z)

				+ rConstant[0]

			    +t2[0].x*t2[0].x + t2[0].x*t2[1].x + t2[1].x*t2[1].x + t2[0].x*t2[2].x + t2[1].x*t2[2].x + t2[2].x*t2[2].x + t2[0].x*t2[3].x + t2[1].x*t2[3].x + t2[2].x*t2[3].x + t2[3].x*t2[3].x
				+ t2[0].y*t2[0].y + t2[0].y*t2[1].y + t2[1].y*t2[1].y + t2[0].y*t2[2].y + t2[1].y*t2[2].y + t2[2].y*t2[2].y + t2[0].y*t2[3].y + t2[1].y*t2[3].y + t2[2].y*t2[3].y + t2[3].y*t2[3].y
				+ t2[0].z*t2[0].z + t2[0].z*t2[1].z + t2[1].z*t2[1].z + t2[0].z*t2[2].z + t2[1].z*t2[2].z + t2[2].z*t2[2].z + t2[0].z*t2[3].z + t2[1].z*t2[3].z + t2[2].z*t2[3].z + t2[3].z*t2[3].z

				- t2[0].x*(rSum[1].x + rTet[1].vertex[vIndex[fIndex][0]].x)
				- t2[1].x*(rSum[1].x + rTet[1].vertex[vIndex[fIndex][1]].x)
				- t2[2].x*(rSum[1].x + rTet[1].vertex[vIndex[fIndex][2]].x)
				- t2[3].x*(rSum[1].x + rTet[1].vertex[vIndex[fIndex][3]].x)

				- t2[0].y*(rSum[1].y + rTet[1].vertex[vIndex[fIndex][0]].y)
				- t2[1].y*(rSum[1].y + rTet[1].vertex[vIndex[fIndex][1]].y)
				- t2[2].y*(rSum[1].y + rTet[1].vertex[vIndex[fIndex][2]].y)
				- t2[3].y*(rSum[1].y + rTet[1].vertex[vIndex[fIndex][3]].y)

				- t2[0].z*(rSum[1].z + rTet[1].vertex[vIndex[fIndex][0]].z)
				- t2[1].z*(rSum[1].z + rTet[1].vertex[vIndex[fIndex][1]].z)
				- t2[2].z*(rSum[1].z + rTet[1].vertex[vIndex[fIndex][2]].z)
				- t2[3].z*(rSum[1].z + rTet[1].vertex[vIndex[fIndex][3]].z)

				+ rConstant[1]) / (10.f);

		//cout << "4" << endl;
		
		model.setObjective(obj, GRB_MINIMIZE);

		//cout << "5" << endl;
		//Calculate the static normal vector
		//n=-r01xr02 : face normal for rest state face
		//d=n dot p0
		vec3 normal;
		double constraintValue = 0.0;

		if (pairIndex<4) {//def tet1
			//cout << "6" << endl;
			separatingPlaneCalculation(rTet[0].face[fIndex], rTet[0].vertex[fIndex], &normal, &constraintValue);
			normal = normal;
		}
		else {
			//cout << "7" << endl;
			separatingPlaneCalculation(rTet[1].face[fIndex], rTet[1].vertex[fIndex], &normal, &constraintValue);
			normal = -normal;
		}

		//cout << "8" << endl;
		// Add constraints: Face에포함된 vertex를 순서대로 p0 p1 p2라 하고, face에 포함되지 않은 vertex를 p3라 하자. 즉 faceIndex를 가진 vertex 자리에 p3를 넣어주어야 한다.

		//mid point MP가 있다고 하면
		//
		// n·(t1_0-mp)<=0   --->
		// n·(t1_1-mp)<=0
		// n·(t1_2-mp)<=0
		// n·(t1_3-mp)<=0

		// n·(t2_0-mp)>=0   --->
		// n·(t2_1-mp)>=0
		// n·(t2_2-mp)>=0
		// n·(t2_3-mp)>=0

		////1.t1은 separating normal 반대방향에 있다.
		//model.addConstr(normal.x*t1[0].x + normal.y*t1[0].y + normal.z*t1[0].z - dot(normal, midPoint) <= 0, "c0");
		//model.addConstr(normal.x*t1[1].x + normal.y*t1[1].y + normal.z*t1[1].z - dot(normal, midPoint) <= 0, "c1");
		//model.addConstr(normal.x*t1[2].x + normal.y*t1[2].y + normal.z*t1[2].z - dot(normal, midPoint) <= 0, "c2");
		//model.addConstr(normal.x*t1[3].x + normal.y*t1[3].y + normal.z*t1[3].z - dot(normal, midPoint) <= 0, "c3");

		////2.t2는 separating normal 방향에 있다.
		//model.addConstr(normal.x*t2[0].x + normal.y*t2[0].y + normal.z*t2[0].z - dot(normal, midPoint) >= 0, "c4");
		//model.addConstr(normal.x*t2[1].x + normal.y*t2[1].y + normal.z*t2[1].z - dot(normal, midPoint) >= 0, "c5");
		//model.addConstr(normal.x*t2[2].x + normal.y*t2[2].y + normal.z*t2[2].z - dot(normal, midPoint) >= 0, "c6");
		//model.addConstr(normal.x*t2[3].x + normal.y*t2[3].y + normal.z*t2[3].z - dot(normal, midPoint) >= 0, "c7");


		if (pairIndex < 4) {

			//1.t1은 separating normal 반대방향에 있다.
			//model.addConstr(normal.x*t1[0].x + normal.y*t1[0].y + normal.z*t1[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c0");
			//model.addConstr(normal.x*t1[1].x + normal.y*t1[1].y + normal.z*t1[1].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c1");
			//model.addConstr(normal.x*t1[2].x + normal.y*t1[2].y + normal.z*t1[2].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c2");
			model.addConstr(normal.x*t1[3].x + normal.y*t1[3].y + normal.z*t1[3].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c3");

			//2.t2는 separating n 방향에 있다.						 
			model.addConstr(normal.x*t2[0].x + normal.y*t2[0].y + normal.z*t2[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c4");
			model.addConstr(normal.x*t2[1].x + normal.y*t2[1].y + normal.z*t2[1].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c5");
			model.addConstr(normal.x*t2[2].x + normal.y*t2[2].y + normal.z*t2[2].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c6");
			model.addConstr(normal.x*t2[3].x + normal.y*t2[3].y + normal.z*t2[3].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c7");

			//3. mp는 separating plane 위에 있다.
			model.addConstr(normal.x*t1[0].x + normal.y*t1[0].y + normal.z*t1[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0, "c8");
			model.addConstr(normal.x*t1[1].x + normal.y*t1[1].y + normal.z*t1[1].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0, "c9");
			model.addConstr(normal.x*t1[2].x + normal.y*t1[2].y + normal.z*t1[2].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0, "c10");

			//model.addConstr(normal.x*t2[0].x + normal.y*t2[0].y + normal.z*t2[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0, "c11");
		}
		else {

			//1.t1은 separating normal 반대방향에 있다.
			model.addConstr(normal.x*t1[0].x + normal.y*t1[0].y + normal.z*t1[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c0");
			model.addConstr(normal.x*t1[1].x + normal.y*t1[1].y + normal.z*t1[1].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c1");
			model.addConstr(normal.x*t1[2].x + normal.y*t1[2].y + normal.z*t1[2].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c2");
			model.addConstr(normal.x*t1[3].x + normal.y*t1[3].y + normal.z*t1[3].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) <= 0, "c3");

			//2.t2는 separating n 방향에 있다.						 
			//model.addConstr(normal.x*t2[0].x + normal.y*t2[0].y + normal.z*t2[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c4");
			//model.addConstr(normal.x*t2[1].x + normal.y*t2[1].y + normal.z*t2[1].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c5");
			//model.addConstr(normal.x*t2[2].x + normal.y*t2[2].y + normal.z*t2[2].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c6");
			model.addConstr(normal.x*t2[3].x + normal.y*t2[3].y + normal.z*t2[3].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) >= 0, "c7");

			//3. mp는 separating plane 위에 있다.
			//model.addConstr(normal.x*t1[0].x + normal.y*t1[0].y + normal.z*t1[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0,  "c8");
																																		    
			model.addConstr(normal.x*t2[0].x + normal.y*t2[0].y + normal.z*t2[0].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0, "c9");
			model.addConstr(normal.x*t2[1].x + normal.y*t2[1].y + normal.z*t2[1].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0, "c10");
			model.addConstr(normal.x*t2[2].x + normal.y*t2[2].y + normal.z*t2[2].z - (normal.x*mp.x + normal.y*mp.y + normal.z*mp.z) == 0,  "c11");
		}

		//cout << "9" << endl;


		//Optimization
		model.optimize();
		//cout << "10" << endl;
		//get Result;
		pTetAll2.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		pTetAll2.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);
		pTetAll2.normal[pairIndex] = normal;
		pTetAll2.planePoint[pairIndex] = vec3(mp.x.get(GRB_DoubleAttr_X), mp.y.get(GRB_DoubleAttr_X), mp.z.get(GRB_DoubleAttr_X));

		vec3 t1p0(t1[0].x.get(GRB_DoubleAttr_X), t1[0].y.get(GRB_DoubleAttr_X), t1[0].z.get(GRB_DoubleAttr_X));
		vec3 t1p1(t1[1].x.get(GRB_DoubleAttr_X), t1[1].y.get(GRB_DoubleAttr_X), t1[1].z.get(GRB_DoubleAttr_X));
		vec3 t1p2(t1[2].x.get(GRB_DoubleAttr_X), t1[2].y.get(GRB_DoubleAttr_X), t1[2].z.get(GRB_DoubleAttr_X));
		vec3 t1p3(t1[3].x.get(GRB_DoubleAttr_X), t1[3].y.get(GRB_DoubleAttr_X), t1[3].z.get(GRB_DoubleAttr_X));

		vec3 t2p0(t2[0].x.get(GRB_DoubleAttr_X), t2[0].y.get(GRB_DoubleAttr_X), t2[0].z.get(GRB_DoubleAttr_X));
		vec3 t2p1(t2[1].x.get(GRB_DoubleAttr_X), t2[1].y.get(GRB_DoubleAttr_X), t2[1].z.get(GRB_DoubleAttr_X));
		vec3 t2p2(t2[2].x.get(GRB_DoubleAttr_X), t2[2].y.get(GRB_DoubleAttr_X), t2[2].z.get(GRB_DoubleAttr_X));
		vec3 t2p3(t2[3].x.get(GRB_DoubleAttr_X), t2[3].y.get(GRB_DoubleAttr_X), t2[3].z.get(GRB_DoubleAttr_X));

		//                       p0 p1 p2 p3    p0 p1 p2 p3      p0 p1 p2 p3     p0 p1 p2 p3
		//int vIndex[4][4] = { { v1,v2,v3,v0 },{ v0,v3,v2,v1 },{ v0,v1,v3,v2 },{ v0,v2,v1,v3 } };
		if (fIndex == 0)	  { initTet(pTetAll2.pTets1[pairIndex], t1p3, t1p0, t1p1, t1p2); initTet(pTetAll2.pTets2[pairIndex], t2p3, t2p0, t2p1, t2p2); }//p3이 not contact, p0p1p2이 face를 이루고 있음.p3=v0, p0=v1 p1=v3 p2=v2
		else if (fIndex == 1) { initTet(pTetAll2.pTets1[pairIndex], t1p0, t1p3, t1p2, t1p1); initTet(pTetAll2.pTets2[pairIndex], t2p0, t2p3, t2p2, t2p1); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0,v2=p1,v3=p2
		else if (fIndex == 2) { initTet(pTetAll2.pTets1[pairIndex], t1p0, t1p1, t1p3, t1p2); initTet(pTetAll2.pTets2[pairIndex], t2p0, t2p1, t2p3, t2p2); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v3=p1 v1=p2
		else if (fIndex == 3) { initTet(pTetAll2.pTets1[pairIndex], t1p0, t1p2, t1p1, t1p3); initTet(pTetAll2.pTets2[pairIndex], t2p0, t2p2, t2p1, t2p3); }//p3이 not contact, p0p1p2이 face를 이루고 있음. v0=p0 v1=p1 v2=p2
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

void DefDefPD::optDefDefEdge(int t1Index, int t2Index, int pairIndex) {
	//cout << "\n =========optEdgEdge tet1 Edge " << t1Index << ", tet2 Edge" << t2Index << endl;
	

	int e[6][2] = { { 0,1 },{ 0,2 },{ 0,3 },{ 1,2 },{ 1,3 },{ 2,3 } };
	
	//index maching with variable
	//
	int i1[4] = { e[t1Index][0] ,e[t1Index][1],e[5 - t1Index][0],e[5 - t1Index][1] };
	int i2[4] = { e[t2Index][0] ,e[t2Index][1],e[5 - t2Index][0],e[5 - t2Index][1] };

	vec3 t1[4] = { rTet[0].vertex[i1[0]], rTet[0].vertex[i1[1]] ,rTet[0].vertex[i1[2]] ,rTet[0].vertex[i1[3]] };
	vec3 t2[4] = { rTet[1].vertex[i2[0]], rTet[1].vertex[i2[1]] ,rTet[1].vertex[i2[2]] ,rTet[1].vertex[i2[3]] };

	vec3 t1_e01 = t1[1] - t1[0];//contact edge vector in tet1's rest state
	vec3 t1_e02 = t1[2] - t1[0];//non-contact edge vector in tet1's rest tet.
	vec3 t1_e03 = t1[3] - t1[0];//non-contact edge vector in tet1's rest tet.

	vec3 t2_e01 = t2[1] - t2[0];//contact edge vector in tet2's rest state

	vec3 n_cross = cross(t1_e01, t2_e01);
	//printV3(n_cross);
	vec3 n = normalize(n_cross);// normal vector 두 contact edge에 동시에 수직인 벡터, direction should be decided.

	//Decide the normal direction	
	float nDote02 = dot(n, t1_e02);// n·(t1.r2 - t1.r0),
	float nDote03 = dot(n, t1_e03);// n·(t1.r3 - t1.r0), 

	bool nDecided = false;

	//	when two edges are parallel , separation direction should be calculated in different way
	//  n is the shortest distance vector from the point t2[0](A) to tet1 edge 01(BH).
	//  ---------t2[0](A)---------t2[1]----
	//          /     |    
	//  ---t1[0](B)---H------>t1[1](C)-------				
	if (dot(n_cross, n_cross) == 0) {	

		//vec3 AB = t1[0] - t2[0];
		//vec3 BCdirection = normalize(t1_e01);
		//vec3 BH = sqrt(dot(AB, AB) - dot(cross(BCdirection, -AB), cross(BCdirection, -AB)))*BCdirection; 		
		//n = AB + BH;
		//n = normalize(n);
		// both normal has to be tested in this case
		vec3 r0s0 = t2[0] - t1[0];
		n_cross = cross(r0s0, t1_e01);
		if (dot(n_cross, n_cross) == 0) {
			//if two edges are colinear, ignore
			cout << "ha" << endl;
			return;
		}
		n = normalize(n_cross);
		nDecided = false;		
		//cout << "\n\n\n\n[FASLE]: 0. Parallel Edges \n\n\n" << endl;
		//cout << "n: ";
		//printV3(n);
	}
	else if ((nDote02 * nDote03) < 0) { // n·(s2 - s0) <0 or  n·(s3 - s0)<0
										// if s2, s3 are in different direction, then normal cannot be decided
		nDecided = false;
	//	cout << "\n\n\n\n[FASLE]: 1. Normal not decided \n\n\n" << endl;
	}
	else if ((nDote02 >= 0) && (nDote03 >= 0)) { // n·(s2 - s0) >=0 and  n·(s3 - s0) >=0
												 //if both s2, s3 are in the same direction of normal, then the normal direction should be inverted
		n = -n;
		nDecided = true;
		//cout << "\n\n\n\n[true]: 2 . normal Inverted\n\n\n" << endl;
	}
	else if ((nDote02 <= 0) && (nDote03 <= 0)) {// n·(s2 - s0) <=0 and  n·(s3 - s0) <=0
												//if both s2, s3 are in the opposite direction of normal, then the normal direction is correct
		nDecided = true;
		//cout << "\n\n\n\n[true]: 3. normal unchanged\n\n\n" << endl;
	}
	else {
		nDecided = false; 		
		//cout << "\n\n\n\n[FASLE]: 4. this cannot be reached something wrong\n\n\n" << endl;
	}


	try {
		GRBModel model = GRBModel(*env);

		// Create variables

		Var t1[4];
		Var t2[4];
		Var mp;

		mp.x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPx"); 
		mp.y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPy"); 
		mp.z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPz"); 

		t1[0].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0x");
		t1[0].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0y");
		t1[0].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0z");

		t1[1].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1x");
		t1[1].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1y");
		t1[1].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1z");

		t1[2].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2x");
		t1[2].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2y");
		t1[2].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2z");

		t1[3].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3x");
		t1[3].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3y");
		t1[3].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3z");


		t2[0].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0x");
		t2[0].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0y");
		t2[0].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0z");

		t2[1].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1x");
		t2[1].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1y");
		t2[1].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1z");

		t2[2].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2x");
		t2[2].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2y");
		t2[2].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2z");

		t2[3].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3x");
		t2[3].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3y");
		t2[3].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3z");

		//cout << "3" << endl;
		// Set objective
		GRBQuadExpr obj =
			     (t1[0].x*t1[0].x + t1[0].x*t1[1].x + t1[1].x*t1[1].x + t1[0].x*t1[2].x + t1[1].x*t1[2].x + t1[2].x*t1[2].x + t1[0].x*t1[3].x + t1[1].x*t1[3].x + t1[2].x*t1[3].x + t1[3].x*t1[3].x
				+ t1[0].y*t1[0].y + t1[0].y*t1[1].y + t1[1].y*t1[1].y + t1[0].y*t1[2].y + t1[1].y*t1[2].y + t1[2].y*t1[2].y + t1[0].y*t1[3].y + t1[1].y*t1[3].y + t1[2].y*t1[3].y + t1[3].y*t1[3].y
				+ t1[0].z*t1[0].z + t1[0].z*t1[1].z + t1[1].z*t1[1].z + t1[0].z*t1[2].z + t1[1].z*t1[2].z + t1[2].z*t1[2].z + t1[0].z*t1[3].z + t1[1].z*t1[3].z + t1[2].z*t1[3].z + t1[3].z*t1[3].z

				- t1[0].x*(rSum[0].x + rTet[0].vertex[i1[0]].x)
				- t1[1].x*(rSum[0].x + rTet[0].vertex[i1[1]].x)
				- t1[2].x*(rSum[0].x + rTet[0].vertex[i1[2]].x)
				- t1[3].x*(rSum[0].x + rTet[0].vertex[i1[3]].x)

				- t1[0].y*(rSum[0].y + rTet[0].vertex[i1[0]].y)
				- t1[1].y*(rSum[0].y + rTet[0].vertex[i1[1]].y)
				- t1[2].y*(rSum[0].y + rTet[0].vertex[i1[2]].y)
				- t1[3].y*(rSum[0].y + rTet[0].vertex[i1[3]].y)

				- t1[0].z*(rSum[0].z + rTet[0].vertex[i1[0]].z)
				- t1[1].z*(rSum[0].z + rTet[0].vertex[i1[1]].z)
				- t1[2].z*(rSum[0].z + rTet[0].vertex[i1[2]].z)
				- t1[3].z*(rSum[0].z + rTet[0].vertex[i1[3]].z)

				+ rConstant[0]

			+ t2[0].x*t2[0].x + t2[0].x*t2[1].x + t2[1].x*t2[1].x + t2[0].x*t2[2].x + t2[1].x*t2[2].x + t2[2].x*t2[2].x + t2[0].x*t2[3].x + t2[1].x*t2[3].x + t2[2].x*t2[3].x + t2[3].x*t2[3].x
				+ t2[0].y*t2[0].y + t2[0].y*t2[1].y + t2[1].y*t2[1].y + t2[0].y*t2[2].y + t2[1].y*t2[2].y + t2[2].y*t2[2].y + t2[0].y*t2[3].y + t2[1].y*t2[3].y + t2[2].y*t2[3].y + t2[3].y*t2[3].y
				+ t2[0].z*t2[0].z + t2[0].z*t2[1].z + t2[1].z*t2[1].z + t2[0].z*t2[2].z + t2[1].z*t2[2].z + t2[2].z*t2[2].z + t2[0].z*t2[3].z + t2[1].z*t2[3].z + t2[2].z*t2[3].z + t2[3].z*t2[3].z

				- t2[0].x*(rSum[1].x + rTet[1].vertex[i2[0]].x)
				- t2[1].x*(rSum[1].x + rTet[1].vertex[i2[1]].x)
				- t2[2].x*(rSum[1].x + rTet[1].vertex[i2[2]].x)
				- t2[3].x*(rSum[1].x + rTet[1].vertex[i2[3]].x)

				- t2[0].y*(rSum[1].y + rTet[1].vertex[i2[0]].y)
				- t2[1].y*(rSum[1].y + rTet[1].vertex[i2[1]].y)
				- t2[2].y*(rSum[1].y + rTet[1].vertex[i2[2]].y)
				- t2[3].y*(rSum[1].y + rTet[1].vertex[i2[3]].y)

				- t2[0].z*(rSum[1].z + rTet[1].vertex[i2[0]].z)
				- t2[1].z*(rSum[1].z + rTet[1].vertex[i2[1]].z)
				- t2[2].z*(rSum[1].z + rTet[1].vertex[i2[2]].z)
				- t2[3].z*(rSum[1].z + rTet[1].vertex[i2[3]].z)

				+ rConstant[1]) / (10.f);

		model.setObjective(obj, GRB_MINIMIZE);


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

		////1.t1은 separating normal 반대방향에 있다.
		//model.addConstr(n.x*t1[0].x + n.y*t1[0].y + n.z*t1[0].z - dot(n, midPoint) <= 0, "c0");
		//model.addConstr(n.x*t1[1].x + n.y*t1[1].y + n.z*t1[1].z - dot(n, midPoint) <= 0, "c1");
		//model.addConstr(n.x*t1[2].x + n.y*t1[2].y + n.z*t1[2].z - dot(n, midPoint) <= 0, "c2");
		//model.addConstr(n.x*t1[3].x + n.y*t1[3].y + n.z*t1[3].z - dot(n, midPoint) <= 0, "c3");

		////2.t2는 separating n 방향에 있다.
		//model.addConstr(n.x*t2[0].x + n.y*t2[0].y + n.z*t2[0].z - dot(n, midPoint) >= 0, "c4");
		//model.addConstr(n.x*t2[1].x + n.y*t2[1].y + n.z*t2[1].z - dot(n, midPoint) >= 0, "c5");
		//model.addConstr(n.x*t2[2].x + n.y*t2[2].y + n.z*t2[2].z - dot(n, midPoint) >= 0, "c6");
		//model.addConstr(n.x*t2[3].x + n.y*t2[3].y + n.z*t2[3].z - dot(n, midPoint) >= 0, "c7");

		//1.t1은 separating normal 반대방향에 있다.
		//model.addConstr(n.x*t1[0].x + n.y*t1[0].y + n.z*t1[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c0");
		//model.addConstr(n.x*t1[1].x + n.y*t1[1].y + n.z*t1[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c1");
		model.addConstr(n.x*t1[2].x + n.y*t1[2].y + n.z*t1[2].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c2");
		model.addConstr(n.x*t1[3].x + n.y*t1[3].y + n.z*t1[3].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c3");
																  
		//2.t2는 separating n 방향에 있다.						 
		//model.addConstr(n.x*t2[0].x + n.y*t2[0].y + n.z*t2[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c4");
		//model.addConstr(n.x*t2[1].x + n.y*t2[1].y + n.z*t2[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c5");
		model.addConstr(n.x*t2[2].x + n.y*t2[2].y + n.z*t2[2].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c6");
		model.addConstr(n.x*t2[3].x + n.y*t2[3].y + n.z*t2[3].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c7");
		//3. mp는 separating plane 위에 있다.

		model.addConstr(n.x*t1[0].x + n.y*t1[0].y + n.z*t1[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c8");
		model.addConstr(n.x*t1[1].x + n.y*t1[1].y + n.z*t1[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c9");

		model.addConstr(n.x*t2[1].x + n.y*t2[1].y + n.z*t2[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c10");																									
		model.addConstr(n.x*t2[0].x + n.y*t2[0].y + n.z*t2[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c11");
		
		//Optimization
		model.optimize();
		//cout << "-----------------5----------------------------------------------------------" << endl;

		//Results
		// Time, value, pTet.
		pTetAll2.optTime[pairIndex] = model.get(GRB_DoubleAttr_Runtime);
		pTetAll2.optValue[pairIndex] = model.get(GRB_DoubleAttr_ObjVal);
		pTetAll2.normal[pairIndex] = n;
		pTetAll2.planePoint[pairIndex] = vec3(mp.x.get(GRB_DoubleAttr_X), mp.y.get(GRB_DoubleAttr_X), mp.z.get(GRB_DoubleAttr_X));

		//cout << "-----------------6----------------------------------------------------------" << endl;

		vec3 t1p0(t1[0].x.get(GRB_DoubleAttr_X), t1[0].y.get(GRB_DoubleAttr_X), t1[0].z.get(GRB_DoubleAttr_X));
		vec3 t1p1(t1[1].x.get(GRB_DoubleAttr_X), t1[1].y.get(GRB_DoubleAttr_X), t1[1].z.get(GRB_DoubleAttr_X));
		vec3 t1p2(t1[2].x.get(GRB_DoubleAttr_X), t1[2].y.get(GRB_DoubleAttr_X), t1[2].z.get(GRB_DoubleAttr_X));
		vec3 t1p3(t1[3].x.get(GRB_DoubleAttr_X), t1[3].y.get(GRB_DoubleAttr_X), t1[3].z.get(GRB_DoubleAttr_X));

		vec3 t2p0(t2[0].x.get(GRB_DoubleAttr_X), t2[0].y.get(GRB_DoubleAttr_X), t2[0].z.get(GRB_DoubleAttr_X));
		vec3 t2p1(t2[1].x.get(GRB_DoubleAttr_X), t2[1].y.get(GRB_DoubleAttr_X), t2[1].z.get(GRB_DoubleAttr_X));
		vec3 t2p2(t2[2].x.get(GRB_DoubleAttr_X), t2[2].y.get(GRB_DoubleAttr_X), t2[2].z.get(GRB_DoubleAttr_X));
		vec3 t2p3(t2[3].x.get(GRB_DoubleAttr_X), t2[3].y.get(GRB_DoubleAttr_X), t2[3].z.get(GRB_DoubleAttr_X));


		//cout << "-----------------7----------------------------------------------------------" << endl;

		switch (t1Index) {
		case 0:initTet(pTetAll2.pTets1[pairIndex], t1p0, t1p1, t1p2, t1p3); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
		case 1:initTet(pTetAll2.pTets1[pairIndex], t1p0, t1p2, t1p1, t1p3); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
		case 2:initTet(pTetAll2.pTets1[pairIndex], t1p0, t1p2, t1p3, t1p1); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
		case 3:initTet(pTetAll2.pTets1[pairIndex], t1p2, t1p0, t1p1, t1p3); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
		case 4:initTet(pTetAll2.pTets1[pairIndex], t1p2, t1p0, t1p3, t1p1); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
		case 5:initTet(pTetAll2.pTets1[pairIndex], t1p2, t1p3, t1p0, t1p1); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
		default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
		}
		switch (t2Index) {
		case 0:initTet(pTetAll2.pTets2[pairIndex], t2p0, t2p1, t2p2, t2p3); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
		case 1:initTet(pTetAll2.pTets2[pairIndex], t2p0, t2p2, t2p1, t2p3); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
		case 2:initTet(pTetAll2.pTets2[pairIndex], t2p0, t2p2, t2p3, t2p1); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
		case 3:initTet(pTetAll2.pTets2[pairIndex], t2p2, t2p0, t2p1, t2p3); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
		case 4:initTet(pTetAll2.pTets2[pairIndex], t2p2, t2p0, t2p3, t2p1); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
		case 5:initTet(pTetAll2.pTets2[pairIndex], t2p2, t2p3, t2p0, t2p1); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
		default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
		}
		if (nDecided == false) {
			//cout << "-----------------seconde case----------------------------------------------------------" << endl;

			n = -n; //do the same thing on the opposite direction

			GRBModel model_second = GRBModel(*env);

			// Create variables
			Var t1_[4];
			Var t2_[4];
			Var mp_;

			mp_.x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPx");
			mp_.y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPy");
			mp_.z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPz");

			t1_[0].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0x");
			t1_[0].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0y");
			t1_[0].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0z");

			t1_[1].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1x");
			t1_[1].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1y");
			t1_[1].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1z");

			t1_[2].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2x");
			t1_[2].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2y");
			t1_[2].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2z");

			t1_[3].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3x");
			t1_[3].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3y");
			t1_[3].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3z");


			t2_[0].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0x");
			t2_[0].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0y");
			t2_[0].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0z");

			t2_[1].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1x");
			t2_[1].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1y");
			t2_[1].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1z");

			t2_[2].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2x");
			t2_[2].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2y");
			t2_[2].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2z");

			t2_[3].x = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3x");
			t2_[3].y = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3y");
			t2_[3].z = model_second.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3z");

			//cout << "3" << endl;
			// Set objective
			GRBQuadExpr obj_ =
					 (t1_[0].x*t1_[0].x + t1_[0].x*t1_[1].x + t1_[1].x*t1_[1].x + t1_[0].x*t1_[2].x + t1_[1].x*t1_[2].x + t1_[2].x*t1_[2].x + t1_[0].x*t1_[3].x + t1_[1].x*t1_[3].x + t1_[2].x*t1_[3].x + t1_[3].x*t1_[3].x
					+ t1_[0].y*t1_[0].y + t1_[0].y*t1_[1].y + t1_[1].y*t1_[1].y + t1_[0].y*t1_[2].y + t1_[1].y*t1_[2].y + t1_[2].y*t1_[2].y + t1_[0].y*t1_[3].y + t1_[1].y*t1_[3].y + t1_[2].y*t1_[3].y + t1_[3].y*t1_[3].y
					+ t1_[0].z*t1_[0].z + t1_[0].z*t1_[1].z + t1_[1].z*t1_[1].z + t1_[0].z*t1_[2].z + t1_[1].z*t1_[2].z + t1_[2].z*t1_[2].z + t1_[0].z*t1_[3].z + t1_[1].z*t1_[3].z + t1_[2].z*t1_[3].z + t1_[3].z*t1_[3].z

					- t1_[0].x*(rSum[0].x + rTet[0].vertex[i1[0]].x)
					- t1_[1].x*(rSum[0].x + rTet[0].vertex[i1[1]].x)
					- t1_[2].x*(rSum[0].x + rTet[0].vertex[i1[2]].x)
					- t1_[3].x*(rSum[0].x + rTet[0].vertex[i1[3]].x)

					- t1_[0].y*(rSum[0].y + rTet[0].vertex[i1[0]].y)
					- t1_[1].y*(rSum[0].y + rTet[0].vertex[i1[1]].y)
					- t1_[2].y*(rSum[0].y + rTet[0].vertex[i1[2]].y)
					- t1_[3].y*(rSum[0].y + rTet[0].vertex[i1[3]].y)

					- t1_[0].z*(rSum[0].z + rTet[0].vertex[i1[0]].z)
					- t1_[1].z*(rSum[0].z + rTet[0].vertex[i1[1]].z)
					- t1_[2].z*(rSum[0].z + rTet[0].vertex[i1[2]].z)
					- t1_[3].z*(rSum[0].z + rTet[0].vertex[i1[3]].z)

					+ rConstant[0]

					+ t2_[0].x*t2_[0].x + t2_[0].x*t2_[1].x + t2_[1].x*t2_[1].x + t2_[0].x*t2_[2].x + t2_[1].x*t2_[2].x + t2_[2].x*t2_[2].x + t2_[0].x*t2_[3].x + t2_[1].x*t2_[3].x + t2_[2].x*t2_[3].x + t2_[3].x*t2_[3].x
					+ t2_[0].y*t2_[0].y + t2_[0].y*t2_[1].y + t2_[1].y*t2_[1].y + t2_[0].y*t2_[2].y + t2_[1].y*t2_[2].y + t2_[2].y*t2_[2].y + t2_[0].y*t2_[3].y + t2_[1].y*t2_[3].y + t2_[2].y*t2_[3].y + t2_[3].y*t2_[3].y
					+ t2_[0].z*t2_[0].z + t2_[0].z*t2_[1].z + t2_[1].z*t2_[1].z + t2_[0].z*t2_[2].z + t2_[1].z*t2_[2].z + t2_[2].z*t2_[2].z + t2_[0].z*t2_[3].z + t2_[1].z*t2_[3].z + t2_[2].z*t2_[3].z + t2_[3].z*t2_[3].z

					- t2_[0].x*(rSum[1].x + rTet[1].vertex[i2[0]].x)
					- t2_[1].x*(rSum[1].x + rTet[1].vertex[i2[1]].x)
					- t2_[2].x*(rSum[1].x + rTet[1].vertex[i2[2]].x)
					- t2_[3].x*(rSum[1].x + rTet[1].vertex[i2[3]].x)

					- t2_[0].y*(rSum[1].y + rTet[1].vertex[i2[0]].y)
					- t2_[1].y*(rSum[1].y + rTet[1].vertex[i2[1]].y)
					- t2_[2].y*(rSum[1].y + rTet[1].vertex[i2[2]].y)
					- t2_[3].y*(rSum[1].y + rTet[1].vertex[i2[3]].y)

					- t2_[0].z*(rSum[1].z + rTet[1].vertex[i2[0]].z)
					- t2_[1].z*(rSum[1].z + rTet[1].vertex[i2[1]].z)
					- t2_[2].z*(rSum[1].z + rTet[1].vertex[i2[2]].z)
					- t2_[3].z*(rSum[1].z + rTet[1].vertex[i2[3]].z)

					+ rConstant[1]) / (10.f);

			model_second.setObjective(obj_, GRB_MINIMIZE);

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
			////1.t1은 separating normal 반대방향에 있다.
			//model_second.addConstr(n.x*t1_[0].x + n.y*t1_[0].y + n.z*t1_[0].z - dot(n, midPoint) <= 0, "c0");
			//model_second.addConstr(n.x*t1_[1].x + n.y*t1_[1].y + n.z*t1_[1].z - dot(n, midPoint) <= 0, "c1");
			//model_second.addConstr(n.x*t1_[2].x + n.y*t1_[2].y + n.z*t1_[2].z - dot(n, midPoint) <= 0, "c2");
			//model_second.addConstr(n.x*t1_[3].x + n.y*t1_[3].y + n.z*t1_[3].z - dot(n, midPoint) <= 0, "c3");

			////2.t2는 separating n 방향에 있다.
			//model_second.addConstr(n.x*t2_[0].x + n.y*t2_[0].y + n.z*t2_[0].z - dot(n, midPoint) >= 0, "c4");
			//model_second.addConstr(n.x*t2_[1].x + n.y*t2_[1].y + n.z*t2_[1].z - dot(n, midPoint) >= 0, "c5");
			//model_second.addConstr(n.x*t2_[2].x + n.y*t2_[2].y + n.z*t2_[2].z - dot(n, midPoint) >= 0, "c6");
			//model_second.addConstr(n.x*t2_[3].x + n.y*t2_[3].y + n.z*t2_[3].z - dot(n, midPoint) >= 0, "c7");

			//1.t1은 separating normal 반대방향에 있다.
			//model_second.addConstr(n.x*t1_[0].x + n.y*t1_[0].y + n.z*t1_[0].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) <= 0, "c0");
			//model_second.addConstr(n.x*t1_[1].x + n.y*t1_[1].y + n.z*t1_[1].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) <= 0, "c1");
			model_second.addConstr(n.x*t1_[2].x + n.y*t1_[2].y + n.z*t1_[2].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) <= 0, "c2");
			model_second.addConstr(n.x*t1_[3].x + n.y*t1_[3].y + n.z*t1_[3].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) <= 0, "c3");

			//2.t2는 separating n 방향에 있다.						 
			//model_second.addConstr(n.x*t2_[0].x + n.y*t2_[0].y + n.z*t2_[0].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) >= 0, "c4");
			//model_second.addConstr(n.x*t2_[1].x + n.y*t2_[1].y + n.z*t2_[1].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) >= 0, "c5");
			model_second.addConstr(n.x*t2_[2].x + n.y*t2_[2].y + n.z*t2_[2].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) >= 0, "c6");
			model_second.addConstr(n.x*t2_[3].x + n.y*t2_[3].y + n.z*t2_[3].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) >= 0, "c7");
			//3. mp는 separating plane 위에 있다.

			model_second.addConstr(n.x*t1_[0].x + n.y*t1_[0].y + n.z*t1_[0].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) == 0, "c8");
			model_second.addConstr(n.x*t1_[1].x + n.y*t1_[1].y + n.z*t1_[1].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) == 0, "c9");

			model_second.addConstr(n.x*t2_[1].x + n.y*t2_[1].y + n.z*t2_[1].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) == 0, "c10");
			model_second.addConstr(n.x*t2_[0].x + n.y*t2_[0].y + n.z*t2_[0].z - (n.x*mp_.x + n.y*mp_.y + n.z*mp_.z) == 0, "c11");

			//Optimization
			model_second.optimize();

			//Results
			// Time, value, pTet.
			pTetAll2.optTime[pairIndex] += model_second.get(GRB_DoubleAttr_Runtime);
			if (pTetAll2.optValue[pairIndex] > model_second.get(GRB_DoubleAttr_ObjVal)) {

				pTetAll2.optValue[pairIndex] = model_second.get(GRB_DoubleAttr_ObjVal);
				pTetAll2.normal[pairIndex] = n;
				pTetAll2.planePoint[pairIndex] = vec3(mp_.x.get(GRB_DoubleAttr_X), mp_.y.get(GRB_DoubleAttr_X), mp_.z.get(GRB_DoubleAttr_X));


				vec3 t1p0_(t1_[0].x.get(GRB_DoubleAttr_X), t1_[0].y.get(GRB_DoubleAttr_X), t1_[0].z.get(GRB_DoubleAttr_X));
				vec3 t1p1_(t1_[1].x.get(GRB_DoubleAttr_X), t1_[1].y.get(GRB_DoubleAttr_X), t1_[1].z.get(GRB_DoubleAttr_X));
				vec3 t1p2_(t1_[2].x.get(GRB_DoubleAttr_X), t1_[2].y.get(GRB_DoubleAttr_X), t1_[2].z.get(GRB_DoubleAttr_X));
				vec3 t1p3_(t1_[3].x.get(GRB_DoubleAttr_X), t1_[3].y.get(GRB_DoubleAttr_X), t1_[3].z.get(GRB_DoubleAttr_X));
						 
				vec3 t2p0_(t2_[0].x.get(GRB_DoubleAttr_X), t2_[0].y.get(GRB_DoubleAttr_X), t2_[0].z.get(GRB_DoubleAttr_X));
				vec3 t2p1_(t2_[1].x.get(GRB_DoubleAttr_X), t2_[1].y.get(GRB_DoubleAttr_X), t2_[1].z.get(GRB_DoubleAttr_X));
				vec3 t2p2_(t2_[2].x.get(GRB_DoubleAttr_X), t2_[2].y.get(GRB_DoubleAttr_X), t2_[2].z.get(GRB_DoubleAttr_X));
				vec3 t2p3_(t2_[3].x.get(GRB_DoubleAttr_X), t2_[3].y.get(GRB_DoubleAttr_X), t2_[3].z.get(GRB_DoubleAttr_X));

				switch (t1Index) {
				case 0:initTet(pTetAll2.pTets1[pairIndex], t1p0_, t1p1_, t1p2_, t1p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
				case 1:initTet(pTetAll2.pTets1[pairIndex], t1p0_, t1p2_, t1p1_, t1p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
				case 2:initTet(pTetAll2.pTets1[pairIndex], t1p0_, t1p2_, t1p3_, t1p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
				case 3:initTet(pTetAll2.pTets1[pairIndex], t1p2_, t1p0_, t1p1_, t1p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
				case 4:initTet(pTetAll2.pTets1[pairIndex], t1p2_, t1p0_, t1p3_, t1p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
				case 5:initTet(pTetAll2.pTets1[pairIndex], t1p2_, t1p3_, t1p0_, t1p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
				default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
				}

				switch (t2Index) {
				case 0:initTet(pTetAll2.pTets2[pairIndex], t2p0_, t2p1_, t2p2_, t2p3_); break;//dIndex=0: p0=v0 p1=v1 p2=v2 p3=v3
				case 1:initTet(pTetAll2.pTets2[pairIndex], t2p0_, t2p2_, t2p1_, t2p3_); break;//dIndex=1: p0=v0 p1=v2 p2=v1 p3=v3
				case 2:initTet(pTetAll2.pTets2[pairIndex], t2p0_, t2p2_, t2p3_, t2p1_); break;//dIndex=2: p0=v0 p1=v3 p2=v1 p3=v2
				case 3:initTet(pTetAll2.pTets2[pairIndex], t2p2_, t2p0_, t2p1_, t2p3_); break;//dIndex=3: p0=v1 p1=v2 p2=v0 p3=v3
				case 4:initTet(pTetAll2.pTets2[pairIndex], t2p2_, t2p0_, t2p3_, t2p1_); break;//dIndex=4: p0=v1 p1=v3 p2=v0 p3=v2
				case 5:initTet(pTetAll2.pTets2[pairIndex], t2p2_, t2p3_, t2p0_, t2p1_); break;//dIndex=5: p0=v2 p1=v3 p2=v0 p3=v1
				default: cout << "[EdgeEdge Result]Wrong Edge Index \n" << endl; break;
				}
			}
		}
	}
	catch (GRBException ee) {
		cout << "Error code = " << ee.getErrorCode() << endl;
		cout << ee.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

}



void DefDefPD::calculateMidPoint() {
	//calculate mid point from two tets
	vec3 sum(0, 0, 0);
	for (int i = 0; i < 4; i++) {
		sum += rTet[0].vertex[i] + rTet[1].vertex[i];
	}
	vec3 midPoint = vec3(sum.x / 8, sum.y / 8, sum.z / 8);
	//cout << "midPoint= (" << sum.x << "," << sum.y << "," << sum.z << ")" << endl;
}

void DefDefPD::printV3(vec3 v) {
	cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
}
void DefDefPD::printResult(int pairIndex) {

	tet pTet[2] = { pTetAll2.pTets1[pairIndex], pTetAll2.pTets2[pairIndex] };
	double optValue = pTetAll2.optValue[pairIndex];	
	double optTime = pTetAll2.optTime[pairIndex];

	int minOptIndex = pairIndex;
	if (minOptIndex != -1) {

		cout << endl;
		cout << "[Optimization Result]" << endl;
		cout << "1. Minimum metric value: " << minOptValue << endl;
		cout << " metric value: " << optValue << endl;

		cout << "2. Optimization time for this pair:" << optTime << endl;
		cout << "3. Total Optimization time: " << totalOptTime << endl;
		cout << "4. Separating plane from: " << minOptIndex << endl;

		//cout << "\n1) tet1 Rest:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "s" << i << "=";
			printV3(rTet[0].vertex[i]);
		}

		//cout << "\n2) tet2 Rest:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "r" << i << "=";
			printV3(rTet[1].vertex[i]);
		}
		//cout << "\n3) tet1 Deformed:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "p" << i << "=";
			printV3(pTet[0].vertex[i]);
		}
		//cout << "\n3) tet2 Deformed:" << endl;
		for (int i = 0; i < 4; i++) {
			cout << "p" << i << "=";
			printV3(pTet[1].vertex[i]);
		}

	}
	else {
		cout << "ERROR: Not Appropriate Tet Deform calculation " << endl;
	}

	if (minOptIndex < 4) {
		cout << "5. Separating plane from tet1 face " << minOptIndex << ":" << endl;;
		for (int i = 0; i < 3; i++) {
			printV3(pTet[0].face[minOptIndex][i]);
		}
	}
	else if (minOptIndex < 8) {
		int index = minOptIndex - 4;

		cout << "5. Separating Plane from tet2 face " << index << ":" << endl;;
		for (int i = 0; i < 3; i++) {
			printV3(pTet[1].face[index][i]);
		}
	}
	else {
		int index = minOptIndex - 8;
		int indexS = index / 6;
		int indexP = index % 6;
		cout << "5. Separating Plane from tet1 edge :" << indexS;
		for (int i = 0; i < 2; i++) {
			printV3(pTet[0].edge[indexS][i]);
		}
		cout << "  Separating Plane from tet2 edge :" << indexP;
		for (int i = 0; i < 2; i++) {
			printV3(pTet[1].edge[indexP][i]);
		}

	}
	cout << "6. Volume Changes :" << endl;
	float rVolume[2] = { calculateTetVolume(rTet[0]), calculateTetVolume(rTet[1]) };
	float pVolume[2] = { calculateTetVolume(pTet[0]), calculateTetVolume(pTet[1]) };
	cout << "Rest Volume 1:" << rVolume[0] << "--> Deformed Volume : " << pVolume[0] << endl;
	cout << "Rest Volume 1:" << rVolume[1] << "--> Deformed Volume : " << pVolume[1] << endl;


}
float DefDefPD::calculateTetVolume(tet t) {
	float volume = abs(dot((t.vertex[0] - t.vertex[3]), cross(t.vertex[1] - t.vertex[3], t.vertex[2] - t.vertex[3])) / 6.0f);
	//cout << "tet's volume: " << volume << endl;
	if (volume == 0) {
		//cout << "Volume is Zero!" << endl;
		(volume = 0.0000000000000000001f);
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


void DefDefPD::separatingPlaneCalculation(vec3 faceVrtx[3], vec3 vrtx, vec3 *normal, double *d) {
	//compute the normal vector and constant of the plane equation for 3 vertices
	glm::vec3 v01 = faceVrtx[1] - faceVrtx[0];
	glm::vec3 v12 = faceVrtx[2] - faceVrtx[1];
	//std::cout << "v01: (" << (v01).x << "," << (v01).y << "," << (v01).z << ")" << std::endl;
	//std::cout << "v12: (" << (v12).x << "," << (v12).y << "," << (v12).z << ")" << std::endl;
	vec3 n = cross(v01, v12);
	if (dot(vrtx- faceVrtx[1] , n) > 0) {//if the calculated normal is inward, change it to outward
		n = -n;
	}
	try {
		(*normal) = glm::normalize(n);// divide by zero exception could occur
	}
	catch (exception e) {
		cout <<"exception while normal calculation"<< e.what() << endl;
	}
	*(d) = glm::dot(*normal, faceVrtx[0]);
}

void DefDefPD::fprintV3(ofstream &fp, vec3 v) {
	fp << v.x << "," << v.y << "," << v.z;

}
void DefDefPD::writeCSVHead(string fileName) {

	ofstream output(fileName + ".csv");

	output << " , INPUT,,,,,,,,OUTPUT,,,,,,,," << endl;
	output << "No. , Tet1,,,,Tet2,,,,DeformedTet1,,,,DeformedTet2,,,,PD(obj. norm),Opt. Time,Sep. Index,Sep. Pair,Sep. Normal,,, Volume Before1, Volme After1, Volume Before1, Volme After1" << endl;

	output.close();
}

void DefDefPD::writeCSV(string fileName) {
	ofstream output(fileName + ".csv", ios::app);
	string pair;
	if (minOptIndex < 4) {
		pair = ("tet1 Face" + to_string(minOptIndex));
	}
	else if (minOptIndex < 8) {
		pair = ("tet2 Face" + to_string(minOptIndex - 4));
	}
	else if (minOptIndex < 44) {
		pair = "t1E" + to_string((minOptIndex - 8) / 6) + " t2E" + to_string((minOptIndex - 8) % 6);
	}
	output << numOpt << ",t1.r0,";
	fprintV3(output, rTet[0].vertex[0]);
	output << ",t2.r0,";
	fprintV3(output, rTet[1].vertex[0]);
	output << ",t1.p0,";
	fprintV3(output, pTet[0].vertex[0]);
	output << ",t2.p0,";
	fprintV3(output, pTet[1].vertex[0]);
	output << ",";

	output << minOptValue << ",";
	output << totalOptTime << ",";
	output << minOptIndex << ",";
	output << pair << ",";


	fprintV3(output, pTetAll2.normal[minOptIndex]);
	output << "," << rVolume[0] <<
		"," << calculateTetVolume(pTet[0]) ;
	output << "," << rVolume[1] <<
		"," << calculateTetVolume(pTet[1]) << endl;

	output << ",t1.r1,"; 	fprintV3(output, rTet[0].vertex[1]);
	output << ",t2.r1,";	fprintV3(output, rTet[1].vertex[1]);
	output << ",t1.p1,";	fprintV3(output, pTet[0].vertex[1]);
	output << ",t2.p1,";	fprintV3(output, pTet[1].vertex[1]);
	output << endl;
	output << ",t1.r2,"; 	fprintV3(output, rTet[0].vertex[2]);
	output << ",t2.r2,";	fprintV3(output, rTet[1].vertex[2]);
	output << ",t1.p2,";	fprintV3(output, pTet[0].vertex[2]);
	output << ",t2.p2,";	fprintV3(output, pTet[1].vertex[2]);
	output << endl;
	output << ",t1.r3,"; 	fprintV3(output, rTet[0].vertex[3]);
	output << ",t2.r3,";	fprintV3(output, rTet[1].vertex[3]);
	output << ",t1.p3,";	fprintV3(output, pTet[0].vertex[3]);
	output << ",t2.p3,";	fprintV3(output, pTet[1].vertex[3]);
	output << endl;

	output.close();
}
