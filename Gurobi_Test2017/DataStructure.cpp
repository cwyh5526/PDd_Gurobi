#include "DefDefPD.h"
//#include"DataStructure.h";
//void Tetrahedron::initTet(vec3 v0, vec3 v1, vec3 v2, vec3 v3) {
//	vertex[0] = v0;
//	vertex[1] = v1;
//	vertex[2] = v2;
//	vertex[3] = v3;
//
//	//each face number represent the vertex that the face does not contain.
//	face[0][0] = v1;	face[0][1] = v2;	face[0][2] = v3; //Face 0 has vertex 123
//	face[1][0] = v0;	face[1][1] = v3;	face[1][2] = v2; //Face 1 has vertex 023
//	face[2][0] = v0;	face[2][1] = v1;	face[2][2] = v3;	//Face 2 has vertex 031
//	face[3][0] = v0;	face[3][1] = v2;	face[3][2] = v1; //Face 3 has vertex 012
//
//
//															 //edge (0,5) edge(1,4) edge (2,3) are pairs. if one edge is realted to the contact, then other pair edge will be included in the constraints.
//	edge[0][0] = v0;	edge[0][1] = v1;
//	edge[1][0] = v0;	edge[1][1] = v2;
//	edge[2][0] = v0;	edge[2][1] = v3;
//
//	edge[3][0] = v1;	edge[3][1] = v2;
//	edge[4][0] = v1;	edge[4][1] = v3;
//
//	edge[5][0] = v2;	edge[5][1] = v3;
//
//	volume = calculateTetVolume();
//}
//float Tetrahedron::calculateTetVolume() {
//	volume = abs(dot((vertex[0] - vertex[3]), cross(vertex[1] - vertex[3], vertex[2] - vertex[3])) / 6.0f);
//	//cout << "tet's volume: " << volume << endl;
//	if (volume == 0) {
//		//cout << "Volume is Zero!" << endl;
//		(volume = 0.0000000000000000001);
//	}
//	return volume;
//}
//tet Tetrahedron::toTet() {
//	tet t;
//	t.vertex[0] = vertex[0];
//	t.vertex[1] = vertex[1];
//	t.vertex[2] = vertex[2];
//	t.vertex[3] = vertex[3];
//
//	//each face number represent the vertex that the face does not contain.
//	t.face[0][0] = face[0][0];	t.face[0][1] = face[0][1];	t.face[0][2] = face[0][1]; //Face 0 has vertex 123
//	t.face[1][0] = face[1][0];	t.face[1][1] = face[1][1];	t.face[1][2] = face[1][1]; //Face 1 has vertex 023
//	t.face[2][0] = face[2][0];	t.face[2][1] = face[2][1];	t.face[2][2] = face[2][1];	//Face 2 has vertex 031
//	t.face[3][0] = face[3][0];	t.face[3][1] = face[3][1];	t.face[3][2] = face[3][1]; //Face 3 has vertex 012
//
//
//																					   //edge (0,5) edge(1,4) edge (2,3) are pairs. if one edge is realted to the contact, then other pair edge will be included in the constraints.
//	t.edge[0][0] = edge[0][0];	t.edge[0][1] = edge[0][1];
//	t.edge[1][0] = edge[1][0];	t.edge[1][1] = edge[1][1];
//	t.edge[2][0] = edge[2][0];	t.edge[2][1] = edge[2][1];
//
//	t.edge[3][0] = edge[3][0];	t.edge[3][1] = edge[3][1];
//	t.edge[4][0] = edge[4][0];	t.edge[4][1] = edge[4][1];
//
//	t.edge[5][0] = edge[5][0];	t.edge[5][1] = edge[5][1];
//
//	return t;
//}
//void d(){
//GRBModel model = GRBModel(*env);
//
//// Create variables
//
//Var t1[4];
//Var t2[4];
//Var mp;
//
//mp.x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPx");
//mp.y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPy");
//mp.z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "MPz");
//
//t1[0].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0x");
//t1[0].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0y");
//t1[0].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P0z");
//
//t1[1].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1x");
//t1[1].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1y");
//t1[1].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P1z");
//
//t1[2].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2x");
//t1[2].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2y");
//t1[2].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P2z");
//
//t1[3].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3x");
//t1[3].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3y");
//t1[3].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t1_P3z");
//
//
//t2[0].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0x");
//t2[0].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0y");
//t2[0].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P0z");
//
//t2[1].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1x");
//t2[1].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1y");
//t2[1].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P1z");
//
//t2[2].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2x");
//t2[2].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2y");
//t2[2].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P2z");
//
//t2[3].x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3x");
//t2[3].y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3y");
//t2[3].z = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t2_P3z");
//
//cout << "3" << endl;
//// Set objective
//GRBQuadExpr obj =
//(t1[0].x*t1[0].x + t1[0].x*t1[1].x + t1[1].x*t1[1].x + t1[0].x*t1[2].x + t1[1].x*t1[2].x + t1[2].x*t1[2].x + t1[0].x*t1[3].x + t1[1].x*t1[3].x + t1[2].x*t1[3].x + t1[3].x*t1[3].x
//	+ t1[0].y*t1[0].y + t1[0].y*t1[1].y + t1[1].y*t1[1].y + t1[0].y*t1[2].y + t1[1].y*t1[2].y + t1[2].y*t1[2].y + t1[0].y*t1[3].y + t1[1].y*t1[3].y + t1[2].y*t1[3].y + t1[3].y*t1[3].y
//	+ t1[0].z*t1[0].z + t1[0].z*t1[1].z + t1[1].z*t1[1].z + t1[0].z*t1[2].z + t1[1].z*t1[2].z + t1[2].z*t1[2].z + t1[0].z*t1[3].z + t1[1].z*t1[3].z + t1[2].z*t1[3].z + t1[3].z*t1[3].z
//
//	- t1[0].x*(rSum[0].x + rTet[0].vertex[i1[0]].x)
//	- t1[1].x*(rSum[0].x + rTet[0].vertex[i1[1]].x)
//	- t1[2].x*(rSum[0].x + rTet[0].vertex[i1[2]].x)
//	- t1[3].x*(rSum[0].x + rTet[0].vertex[i1[3]].x)
//
//	- t1[0].y*(rSum[0].y + rTet[0].vertex[i1[0]].y)
//	- t1[1].y*(rSum[0].y + rTet[0].vertex[i1[1]].y)
//	- t1[2].y*(rSum[0].y + rTet[0].vertex[i1[2]].y)
//	- t1[3].y*(rSum[0].y + rTet[0].vertex[i1[3]].y)
//
//	- t1[0].z*(rSum[0].z + rTet[0].vertex[i1[0]].z)
//	- t1[1].z*(rSum[0].z + rTet[0].vertex[i1[1]].z)
//	- t1[2].z*(rSum[0].z + rTet[0].vertex[i1[2]].z)
//	- t1[3].z*(rSum[0].z + rTet[0].vertex[i1[3]].z)
//
//	+ rConstant[0]
//
//	+ t2[0].x*t2[0].x + t2[0].x*t2[1].x + t2[1].x*t2[1].x + t2[0].x*t2[2].x + t2[1].x*t2[2].x + t2[2].x*t2[2].x + t2[0].x*t2[3].x + t2[1].x*t2[3].x + t2[2].x*t2[3].x + t2[3].x*t2[3].x
//	+ t2[0].y*t2[0].y + t2[0].y*t2[1].y + t2[1].y*t2[1].y + t2[0].y*t2[2].y + t2[1].y*t2[2].y + t2[2].y*t2[2].y + t2[0].y*t2[3].y + t2[1].y*t2[3].y + t2[2].y*t2[3].y + t2[3].y*t2[3].y
//	+ t2[0].z*t2[0].z + t2[0].z*t2[1].z + t2[1].z*t2[1].z + t2[0].z*t2[2].z + t2[1].z*t2[2].z + t2[2].z*t2[2].z + t2[0].z*t2[3].z + t2[1].z*t2[3].z + t2[2].z*t2[3].z + t2[3].z*t2[3].z
//
//	- t2[0].x*(rSum[1].x + rTet[1].vertex[i2[0]].x)
//	- t2[1].x*(rSum[1].x + rTet[1].vertex[i2[1]].x)
//	- t2[2].x*(rSum[1].x + rTet[1].vertex[i2[2]].x)
//	- t2[3].x*(rSum[1].x + rTet[1].vertex[i2[3]].x)
//
//	- t2[0].y*(rSum[1].y + rTet[1].vertex[i2[0]].y)
//	- t2[1].y*(rSum[1].y + rTet[1].vertex[i2[1]].y)
//	- t2[2].y*(rSum[1].y + rTet[1].vertex[i2[2]].y)
//	- t2[3].y*(rSum[1].y + rTet[1].vertex[i2[3]].y)
//
//	- t2[0].z*(rSum[1].z + rTet[1].vertex[i2[0]].z)
//	- t2[1].z*(rSum[1].z + rTet[1].vertex[i2[1]].z)
//	- t2[2].z*(rSum[1].z + rTet[1].vertex[i2[2]].z)
//	- t2[3].z*(rSum[1].z + rTet[1].vertex[i2[3]].z)
//
//	+ rConstant[1]) / (10.f);
//
//model.setObjective(obj, GRB_MINIMIZE);
//
//
////cout << "-----------------1----------------------------------------------------------" << endl;
//
////Add Constraints
//
////mid point MP가 있다고 하면
////
//// n·(s0-mp)<=0   --->
//// n·(s1-mp)<=0
//// n·(s2-mp)<=0
//// n·(s3-mp)<=0
//
//// n·(p0-mp)>=0   --->
//// n·(p1-mp)>=0
//// n·(p2-mp)>=0
//// n·(p3-mp)>=0
//
//////1.t1은 separating normal 반대방향에 있다.
////model.addConstr(n.x*t1[0].x + n.y*t1[0].y + n.z*t1[0].z - dot(n, midPoint) <= 0, "c0");
////model.addConstr(n.x*t1[1].x + n.y*t1[1].y + n.z*t1[1].z - dot(n, midPoint) <= 0, "c1");
////model.addConstr(n.x*t1[2].x + n.y*t1[2].y + n.z*t1[2].z - dot(n, midPoint) <= 0, "c2");
////model.addConstr(n.x*t1[3].x + n.y*t1[3].y + n.z*t1[3].z - dot(n, midPoint) <= 0, "c3");
//
//////2.t2는 separating n 방향에 있다.
////model.addConstr(n.x*t2[0].x + n.y*t2[0].y + n.z*t2[0].z - dot(n, midPoint) >= 0, "c4");
////model.addConstr(n.x*t2[1].x + n.y*t2[1].y + n.z*t2[1].z - dot(n, midPoint) >= 0, "c5");
////model.addConstr(n.x*t2[2].x + n.y*t2[2].y + n.z*t2[2].z - dot(n, midPoint) >= 0, "c6");
////model.addConstr(n.x*t2[3].x + n.y*t2[3].y + n.z*t2[3].z - dot(n, midPoint) >= 0, "c7");
//
////1.t1은 separating normal 반대방향에 있다.
////model.addConstr(n.x*t1[0].x + n.y*t1[0].y + n.z*t1[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c0");
////model.addConstr(n.x*t1[1].x + n.y*t1[1].y + n.z*t1[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c1");
//model.addConstr(n.x*t1[2].x + n.y*t1[2].y + n.z*t1[2].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c2");
//model.addConstr(n.x*t1[3].x + n.y*t1[3].y + n.z*t1[3].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) <= 0, "c3");
//
////2.t2는 separating n 방향에 있다.						 
////model.addConstr(n.x*t2[0].x + n.y*t2[0].y + n.z*t2[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c4");
////model.addConstr(n.x*t2[1].x + n.y*t2[1].y + n.z*t2[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c5");
//model.addConstr(n.x*t2[2].x + n.y*t2[2].y + n.z*t2[2].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c6");
//model.addConstr(n.x*t2[3].x + n.y*t2[3].y + n.z*t2[3].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) >= 0, "c7");
////3. mp는 separating plane 위에 있다.
//
//model.addConstr(n.x*t1[0].x + n.y*t1[0].y + n.z*t1[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c8");
//model.addConstr(n.x*t1[1].x + n.y*t1[1].y + n.z*t1[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c9");
//
//model.addConstr(n.x*t2[1].x + n.y*t2[1].y + n.z*t2[1].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c10");
//model.addConstr(n.x*t2[0].x + n.y*t2[0].y + n.z*t2[0].z - (n.x*mp.x + n.y*mp.y + n.z*mp.z) == 0, "c11");
//
////Optimization
//model.optimize();
//}
//
//#include "RigidPD.h"
//#include "DefPD.h"
//#include "DefDefPD.h"
//
//#define rN() ((float)rand() / (RAND_MAX + 1)) //random number 0 ~ 1.0
//
//void quick_sort(double *data, int *index, int start, int end) {
//	if (start >= end) {
//		// 원소가 1개인 경우
//		return;
//	}
//
//	int pivot = start;
//	int i = pivot + 1; // 왼쪽 출발 지점 
//	int j = end; // 오른쪽 출발 지점
//	double temp;
//	int tempI;
//
//	while (i <= j) {
//		// 포인터가 엇갈릴때까지 반복
//		while (i <= end && data[i] <= data[pivot]) {
//			i++;
//		}
//		while (j > start && data[j] >= data[pivot]) {
//			j--;
//		}
//
//		if (i > j) {
//			// 엇갈림
//			temp = data[j];
//			data[j] = data[pivot];
//			data[pivot] = temp;
//
//			tempI = index[j];
//			index[j] = index[pivot];
//			index[pivot] = tempI;
//		}
//		else {
//			// i번째와 j번째를 스왑
//			temp = data[i];
//			data[i] = data[j];
//			data[j] = temp;
//
//			tempI = index[i];
//			index[i] = index[j];
//			index[j] = tempI;
//
//		}
//	}
//
//	// 분할 계산
//	quick_sort(data, index, start, j - 1);
//	quick_sort(data, index, j + 1, end);
//
//	return;
//}
//
//void randomTetGeneration(tet& t) {
//
//	for (int i = 0; i < 4; i++) {
//		t.vertex[i] = vec3(rN(), rN(), rN());
//	}
//}
//void randomVertexGeneration(float* v) {
//	//for (int i = 0; i < 4; i++) {
//	v[0] = rN();
//	v[1] = rN();
//	v[2] = rN();
//	//}
//}
//
//int main(int argc, const char *argv[])
//{
//	float v[2][1000][4][3];
//	tet rTet[2]; //rest tetrahedron
//	tet pTet[2]; //deformed pose tetrahedron
//	double minOptValue[3][1000];
//
//	double sortedPD[1000][44];
//	int sortedPDIndex[1000][44];
//
//	int minOptIndex[3][1000] = { -1 };
//	double totalOptTime[3][1000] = { 0.0 };
//	double meanOptTime[4] = { 0.0 };
//	double maxTime[3] = { 0.0 };
//	double minTime[3] = { INFINITY };
//	double comparePercent = 0.0;
//	int metricComparison = 0;
//	int directionCheck = 0;
//	int directionChecks[44] = { 0 };
//	int directionRank[1000] = { 0 };
//	double averageDirectoinRank = 0;
//	RigidPD rigidPD;
//	DefPD defPD;
//	DefDefPD defdefPD;
//	int numTest = 1000;
//	double totalRigidTime = 0;
//	srand(time(NULL));
//
//	string filename_def;
//	string filename_rigid;
//	string filename_cullingDef;
//
//	time_t totalSec;
//	time(&totalSec);
//	tm *pt = localtime(&totalSec);
//	string fileMadeTime = "_" + to_string(pt->tm_year + 1900) + "_" + to_string(pt->tm_mon + 1) + "_" + to_string(pt->tm_mday) + "_" + to_string(pt->tm_hour) + "_" + to_string(pt->tm_min) + "_" + to_string(pt->tm_sec);
//
//	filename_def = "test" + fileMadeTime + "def";
//	filename_rigid = "test" + fileMadeTime + "rigid";
//
//	defdefPD.writeCSVHead(filename_def);
//	rigidPD.writeCSVHead(filename_rigid);
//
//	int count = 0;
//	for (int i = 0; i < numTest; i++) {
//		//tet generation, volume!=0, penetrated tet pairs;
//		do
//		{
//			for (int j = 0; j < 2; j++) {
//				do
//				{
//					for (int vN = 0; vN < 4; vN++) {
//						randomVertexGeneration(&v[j][i][vN][0]);
//
//						rTet[j].vertex[vN].x = (v[j][i][vN][0]);
//						rTet[j].vertex[vN].y = (v[j][i][vN][1]);
//						rTet[j].vertex[vN].z = (v[j][i][vN][2]);
//					}
//					//randomTetGeneration(rTet[j]);
//				} while (rigidPD.calculateTetVolume(rTet[j]) == 0);
//
//			}
//			rigidPD.init(rTet[0], rTet[1]);
//			if (rigidPD.resolveRigidPenetration() == false) count++;
//
//		} while (rigidPD.resolveRigidPenetration() == false);//if it is separated, generate tet again.
//	}
//
//	/*Rigid PD calculation*/
//	//only calculation time is counted here
//	clock_t start = clock();
//	for (int i = 0; i < numTest; i++) {
//		for (int j = 0; j < 2; j++) {
//			for (int vN = 0; vN < 4; vN++) {
//				rTet[j].vertex[vN].x = (v[j][i][vN][0]);
//				rTet[j].vertex[vN].y = (v[j][i][vN][1]);
//				rTet[j].vertex[vN].z = (v[j][i][vN][2]);
//			}
//		}
//		rigidPD.init(rTet[0], rTet[1]);
//		rigidPD.resolveRigidPenetration();
//	}
//	clock_t end = clock();
//
//	totalRigidTime = (double)((double)end - (double)start) / (double)CLOCKS_PER_SEC;
//	meanOptTime[0] = totalRigidTime;
//	for (int i = 0; i < numTest; i++) {
//		// rigid pd again, for the values to print out, & to store;
//		for (int j = 0; j < 2; j++) {
//			for (int vN = 0; vN < 4; vN++) {
//				rTet[j].vertex[vN].x = (v[j][i][vN][0]);
//				rTet[j].vertex[vN].y = (v[j][i][vN][1]);
//				rTet[j].vertex[vN].z = (v[j][i][vN][2]);
//			}
//		}
//
//		/*for (int k = 0; k < 44; k++) {
//		cout << i << " : [index]=" << sortedPDIndex[i][k] << "[rigid PD]=" << sortedPD[i][k] << endl;
//		}*/
//		//meanOptTime[0] += totalOptTime[0][i];
//
//
//		/*defPD.init(rTet[0], rTet[1]);
//		defPD.resolveStaticDefPenetration();
//		totalOptTime[1][i] = defPD.getOptTime();
//		minOptValue[1][i] = defPD.getPD();
//		minOptIndex[1][i] = defPD.getMinOptIndex();
//		*/
//
//		defdefPD.init(rTet[0], rTet[1]);
//		defdefPD.resolveDefDefPenetration();
//
//		totalOptTime[0][i] = rigidPD.getOptTime();
//		minOptValue[0][i] = rigidPD.getPD();
//		minOptIndex[0][i] = rigidPD.getMinOptIndex();
//
//
//		totalOptTime[2][i] = defdefPD.getOptTime();
//		minOptValue[2][i] = defdefPD.getPD();
//		minOptIndex[2][i] = defdefPD.getMinOptIndex();
//
//
//		/*culling*/
//		clock_t start1 = clock();
//		rigidPD.init(rTet[0], rTet[1]);
//		rigidPD.resolveRigidPenetration();
//		for (int k = 0; k < 44; k++) {
//			sortedPD[i][k] = rigidPD.getPTetAll().optValue[k];
//			sortedPDIndex[i][k] = k;
//		}
//		quick_sort(&sortedPD[i][0], &sortedPDIndex[i][0], 0, 43);
//
//		defdefPD.init(rTet[0], rTet[1]);
//		defdefPD.resolveDefDefPenetrationWithCulling(sortedPDIndex[i], 5);
//		totalOptTime[1][i] = defdefPD.getOptTime();
//		minOptValue[1][i] = defdefPD.getPD();
//		minOptIndex[1][i] = defdefPD.getMinOptIndex();
//		clock_t end = clock();
//		meanOptTime[3] += (double)((double)end - (double)start) / (double)CLOCKS_PER_SEC;
//
//		for (int j = 0; j < 3; j++) {
//			meanOptTime[j] += totalOptTime[j][i];
//			if (totalOptTime[j][i] < minTime[j]) { minTime[j] = totalOptTime[j][i]; }
//			if (totalOptTime[j][i] > maxTime[j]) { maxTime[j] = totalOptTime[j][i]; }
//		}
//
//		if (minOptValue[0][i] > minOptValue[2][i]) {
//			metricComparison++;
//			comparePercent += (minOptValue[0][i] - minOptValue[2][i]) / minOptValue[0][i];
//		}
//
//		if (minOptIndex[0][i] == minOptIndex[2][i]) {//count if two direction is same
//			directionCheck++;
//			directionRank[i] = 0;
//			directionChecks[0]++;
//		}
//		else {
//			directionRank[i] = 1;
//
//			while (minOptIndex[2][i] != sortedPDIndex[i][directionRank[i]]) {
//				directionRank[i] += 1;;
//
//			}
//
//			averageDirectoinRank += (double)directionRank[i] + 1;
//			//for (int haha = 0; haha < directionRank[i]; haha++) {
//			directionChecks[directionRank[i]]++;
//			//}
//		}
//		/*if (sortedPDIndex[i][0] == (minOptIndex[2][i]) || sortedPDIndex[i][1] == (minOptIndex[2][i]) || sortedPDIndex[i][2] == (minOptIndex[2][i])) {
//		directionCheck3rd++;
//		}*/
//
//		defdefPD.writeCSV(filename_def);
//		rigidPD.writeCSV(filename_rigid);
//		if (i % 100 == 0) cout << i << endl;
//	}
//
//
//
//	cout << "total Rigid Time *5000= " << totalRigidTime << endl;
//	cout << "number of separated tet pairs:" << count << endl;
//	comparePercent /= numTest; //10000개로 나누고 100퍼센트로 표시
//	averageDirectoinRank /= numTest;
//	for (int i = 0; i < 4; i++) {
//		meanOptTime[i] /= numTest;
//	}
//	cout << "meanOptTime for Rigid: " << meanOptTime[0] << "seconds" << endl;
//	//cout << "totalOptTime for Def: " << meanOptTime[1] << "seconds" << endl;
//	cout << "meanOptTime for DefDef: " << meanOptTime[2] << "seconds" << endl;
//	cout << "meanOptTime for DefDefculling: w/o culling time" << meanOptTime[1] << "seconds" << endl;
//	cout << "meanOptTime for DefDefculling: w/ culling time" << meanOptTime[3] << "seconds" << endl;
//	cout << "Min,Max time for Rigid: (" << minTime[0] << "," << maxTime[0] << ") seconds" << endl;
//	cout << "Min,Max time for Def:   (" << minTime[1] << "," << maxTime[1] << ") seconds" << endl;
//	cout << "Min,Max time for DefDef:(" << minTime[2] << "," << maxTime[2] << ") seconds" << endl;
//
//
//	cout << "same direction :" << 100.0*(double)directionCheck / (double)numTest << "%" << endl;
//	double sum = 0;
//	for (int haha = 0; haha < 44; haha++)
//	{
//		double v = 100.0*(double)directionChecks[haha] / (double)numTest;
//		sum += v;
//		cout << "possiblity of within " << haha << "th direction :" << v << "%, " << sum << "%" << endl;
//
//	}
//
//	cout << "averate Rank:" << averageDirectoinRank << "th rigid PD direction is same with deformed PD direction" << endl;
//	cout << "rigid > defdef : " << 100.0*(double)metricComparison / (double)numTest << "%" << endl;
//	cout << "Mean differeces between metric: " << 100.0*comparePercent << "%" << endl;
//
//
//
//	////defdefPD->printResult(minOptIndex);
//
//	//for (int i = 0; i < numTest; i++) {
//	//	cout << "rigid PD [" << i << "]= " << minOptValue[0][i] << "\t" << "DefDef PD [" << i << "]= " << minOptValue[2][i] << endl;
//	//	cout << "rigid direction [" << i << "]= " << minOptIndex[0][i] << "\t" << "DefDef direction [" << i << "]= " << minOptIndex[2][i] << endl;
//	//	cout << directionRank[i]<<"th value:"<<"rigid direction[" << sortedPDIndex[i][directionRank[i]] << "] ="<<sortedPD[i][directionRank[i]] << endl;
//	//	cout << endl;
//	//}
//	////1. random generation : if volume==0, regenerate.
//	//// 1) Metric values for each cases --> value comparison, how much?
//	//// 2) Opt Pair indices for each cases --> calculate percentage of accuracy if not 100%
//	//// 3) Total time for each cases --> speed comparison
//	//filename1 = "test" + fileMadeTime + "Total Results";
//
//	//ofstream output(filename1 + ".txt", ios::app);
//
//	//output << "totalOptTime for Rigid: " << meanOptTime[0] << "seconds" << endl;
//	//output << "totalOptTime for Def: " << meanOptTime[1] << "seconds" << endl;
//	//output << "totalOptTime for DefDef: " << meanOptTime[2] << "seconds" << endl;
//
//	//output << "Min,Max time for Rigid: (" << minTime[0] << "," << maxTime[0] << ") seconds" << endl;
//	//output << "Min,Max time for Def:   (" << minTime[1] << "," << maxTime[1] << ") seconds" << endl;
//	//output << "Min,Max time for DefDef:(" << minTime[2] << "," << maxTime[2] << ") seconds" << endl;
//
//
//	//output << "same direction :" << 100.0*(double)directionCheck / (double)numTest << "%" << endl;
//	//sum = 0;
//	//for (int haha = 0; haha < 44; haha++)
//	//{
//	//	double v = 100.0*(double)directionChecks[haha] / (double)numTest;
//	//	sum += v;
//	//	output << "possiblity of within " << haha << "th direction :" << v << "%, " << sum << "%" << endl;
//
//	//}
//
//	//output << "averate Rank:" << averageDirectoinRank << "th rigid PD direction is same with deformed PD direction" << endl;
//	//output << "rigid > defdef : " << 100.0*(double)metricComparison / (double)numTest << "%" << endl;
//	//output << "Mean differeces between metric: " << 100.0*comparePercent << "%" << endl;
//
//
//
//	////defdefPD->printResult(minOptIndex);
//
//	//for (int i = 0; i < numTest; i++) {
//	//	output << "rigid PD [" << i << "]= " << minOptValue[0][i] << "\t" << "DefDef PD [" << i << "]= " << minOptValue[2][i] << endl;
//	//	output << "rigid direction [" << i << "]= " << minOptIndex[0][i] << "\t" << "DefDef direction [" << i << "]= " << minOptIndex[2][i] << endl;
//	//	output << directionRank[i] << "th value:" << "rigid direction[" << sortedPDIndex[i][directionRank[i]] << "] =" << sortedPD[i][directionRank[i]] << endl;
//	//	output << endl;
//	//}
//
//
//
//	return 0;
//}
//
