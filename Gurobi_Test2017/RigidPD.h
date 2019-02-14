#pragma once
#include <iostream>
#include <iomanip>
#include <fstream>

#include"DataStructure.h"

using namespace std;

class RigidPD
{
public:
	//constructor
	RigidPD(); 
	~RigidPD();

	void initDefault();
	void init(tet t1, tet t2);

	//solve Penetration 
	bool resolveRigidPenetration();

	//getters
	tet getRTet(int i) { return rTet[i]; };// .toTet(); };
	tet getPTet(int i) { return pTet[i]; }

	optResults2 getPTetAll() {	return allResults;	};
	int getMinOptIndex() { return minOptIndex; };
	float getPD() { return rigidPD_Value; };
	float getOptTime() { return totalOptTime; };

	void printV3(vec3 v);
	void printResult();

	void writeCSV(string fileName);
	void writeCSVHead(string fileName);

protected:
	plane calculateSeparatingPlane(vec3 faceVrtx[3]); //separating plane of face
	plane calculateSeparatingPlane(vec3 edge[2][2], vec3 vrtx); //separating plane of edge/edge pair.

	bool overlapTest(tet t[2], vec3 o, vec3 n, float& PD_Value, vec3& PD_n);

	float coordOnAxis(vec3 p, vec3 o, vec3 n);

	// setters for tetrahedra
	void setRTet(int i, tet t) { initTet(rTet[i], t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };
	void setPTet(int i, tet t) { initTet(pTet[i], t.vertex[0], t.vertex[1], t.vertex[2], t.vertex[3]); };

	void setRTet(int i, vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(rTet[i], v0, v1, v2, v3); };
	void setPTet(int i, vec3 v0, vec3 v1, vec3 v2, vec3 v3) { initTet(pTet[i], v0, v1, v2, v3); };
	void updateResult(int index, float time,float pd, plane pl);

	// utility functions
	void initTet(tet &T, vec3 v0, vec3 v1, vec3 v2, vec3 v3);	
	float calculateTetVolume(tet t);

	void fprintV3(ofstream &fp, vec3 v);

private:

	//input
	tet rTet[2];
	float rVolume[2];

	//output
	tet pTet[2];
	optResults2 allResults;

	int minOptIndex = -1;
	float rigidPD_Value = 1000.0f; //smallest distances
	vec3 rigidPD_Direction;
	float totalOptTime = 0.f;
	int numOpt = 0;

};
// 1. 모든 separating axis 에 대하여 normal을 구한다.
// 2. 모든 separating axis(plane)에 대하여 plane의 반대 방향에 있는 상대 tet의 vertex를 구하고, 그 vertex에서 plane까지의 거리를 구한 후 가장 작은 값을 구한다.
//