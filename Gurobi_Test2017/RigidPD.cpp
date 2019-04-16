#include "RigidPD.h"
#include <limits>


RigidPD::RigidPD() {
	numOpt = 0;
}
RigidPD::~RigidPD() {

}
//=============
//   public
//=============
void RigidPD::initDefault() {
	//static tetrahedron position
	setRTet(0,
		vec3(0.9, 3.4, 4.7),
		vec3(5.2, 0.1, 5.4),
		vec3(8.5, 3.5, 9.3),
		vec3(9.6, 7.8, 5.5));
		

	//rest pose tetrahedron position
	setRTet(1,
		vec3(4.4, 4.1, 7),
		vec3(5.5, 0.6, 1.3),
		vec3(9, 0.6, 5.10),
		vec3(5.5, 8.9, 9.8));

	////static tetrahedron position
	//setRTet(0, vec3(0.0, 0.0, 0.0),
	//	vec3(1.0, 0.0, 0.0),
	//	vec3(0.0, 1.0, 0.0),
	//	vec3(0.0, 0.0, 1.0));

	////rest pose tetrahedron position
	//setRTet(1, vec3(0.2, 0.2, 0.2),
	//	vec3(-1.2, 0.2, 0.2),
	//	vec3(0.2, 1.2, 0.2),
	//	vec3(0.7, 0.7, 1.2));



	rigidPD_Value = INFINITY;
	minOptIndex = -1;
	totalOptTime = 0.f;
	rVolume[0] = calculateTetVolume(rTet[0]);
	rVolume[1] = calculateTetVolume(rTet[1]);

	return;
}
void RigidPD::init(tet tet1, tet tet2) {
	//1. Initialize each tetrahedon
	setRTet(0, tet1);
	setRTet(1, tet2);

	rigidPD_Value = INFINITY;
	minOptIndex = -1;

	totalOptTime = 0.f;
	rVolume[0] = calculateTetVolume(rTet[0]);
	rVolume[1] = calculateTetVolume(rTet[1]);

}

bool RigidPD::resolveRigidPenetration() {
	int index = 0;
	//for the tet1 Face
	vec3 o;
	plane pl;
	float pd;
	clock_t start = clock();

	for (int i = 0; i < 4; i++) {
		index = i;
		pl = calculateSeparatingPlane(rTet[0].face[i]); //get normal;
		o = rTet[0].face[i][0];
		if (!overlapTest(rTet, o, pl.n, pd, pl.n)) {
			//cout << "Found Separating Axis on tet1 Face " << i << endl;
			return false;
		}
		updateResult(index, -1, pd, pl);
		
	}
	for (int i = 0; i < 4; i++) {
		index = i + 4;
		pl = calculateSeparatingPlane(rTet[1].face[i]);
		o = rTet[1].face[i][0];
		if (!overlapTest(rTet, o, pl.n, pd, pl.n)) {
			//cout << "Found Separating Axis on tet2 Face " << i << endl;
			return false;
		}
		updateResult(index, -1, pd, pl);
	}
	for (int s = 0; s < 6; s++) {
		for (int r = 0; r < 6; r++) {
			index = s * 6 + r + 8;
			vec3 edges[2][2] = { { rTet[0].edge[s][0],rTet[0].edge[s][1] },{ rTet[1].edge[r][0],rTet[1].edge[r][1] } };
			pl = calculateSeparatingPlane(edges, rTet[0].edge[5 - s][0]);
			o = rTet[0].edge[s][0];
			if (!overlapTest(rTet, o, pl.n, pd, pl.n)) {
				//cout << "Found Separating Axis on Edge " << s << " Edge " << r << endl;
				return false;
			}
			updateResult(index, -1, pd, pl);
			
		}
	}

	for (int i = 0; i < 44; i++) {
		//find min pd
		if (rigidPD_Value > allResults.optValue[i]) {
			rigidPD_Value = allResults.optValue[i];
			minOptIndex = i;
		}
	}
	rigidPD_Direction = allResults.normal[minOptIndex];
	rigidPD_Value = allResults.optValue[minOptIndex];
	clock_t end = clock();
	totalOptTime = (double)((double)end - (double)start) / (double)CLOCKS_PER_SEC;
	

	pTet[0] = allResults.pTets1[minOptIndex];
	pTet[1] = allResults.pTets2[minOptIndex];
	numOpt++;
	return true;
}

//===============
//   protected
//===============

plane RigidPD::calculateSeparatingPlane(vec3 faceVrtx[3]) {
	plane pl;	//ax+by+cz+d=0, d= -n¡¤q; where q is a vertex on the plane
				//compute the normal vector and constant of the plane equation for 3 vertices
	vec3 v01 = faceVrtx[1] - faceVrtx[0];
	vec3 v12 = faceVrtx[2] - faceVrtx[1];
	pl.n = normalize(cross(v01, v12));
	pl.d = -dot(pl.n, faceVrtx[0]);
	return pl;
}
plane RigidPD::calculateSeparatingPlane(vec3 edgeVrtx[2][2], vec3 vrtx) {
	plane pl;
	vec3 e1 = edgeVrtx[0][1] - edgeVrtx[0][0]; // contact edge from 1st tet
	vec3 e2 = edgeVrtx[1][1] - edgeVrtx[1][0]; // contact edge from 2nd tet
	vec3 n_cross = cross(e1, e2);

	if (dot(n_cross, n_cross) == 0) {
		//if two edges are parallel
		vec3 temp = cross(e1, edgeVrtx[1][0] - edgeVrtx[0][0]);
		n_cross = cross(e1, temp);
		if (dot(n_cross, n_cross) == 0) {
			//if two edges are colinear, use face plane near
			vec3 temp[3] = { edgeVrtx[0][0], edgeVrtx[0][1], vrtx };
			return calculateSeparatingPlane(temp);
		}
	}
	pl.n = normalize(n_cross);
	pl.d = -dot(pl.n, edgeVrtx[0][0]);
	return pl;
}

// for all vertices of two tetrahedron, find overlap on axis n, origin o.
bool RigidPD::overlapTest(tet t[2], vec3 o, vec3 n, float& PD_Value, vec3& PD_n) {
	//find a.min, a.max, b.min, b.max
	float tmax[2], tmin[2];
	float coord[2][4];
	PD_n = n;

	for (int i = 0; i < 2; i++) {
		tmax[i] = tmin[i] = coord[i][0] = coordOnAxis(t[i].vertex[0], o, n);
		for (int j = 0; j < 4; j++) {//for all vertices, find max and min cood;
			coord[i][j] = coordOnAxis(t[i].vertex[j], o, n);
			if (coord[i][j] < tmin[i]) { tmin[i] = coord[i][j]; }
			if (coord[i][j] > tmax[i]) { tmax[i] = coord[i][j]; }
		}
	}

	float canPD[2] = {tmax[0]-tmin[1],tmax[1]-tmin[0]};//candidate of PD
	//no penetration on this axis;
	if ((canPD[0] < 0.002) || (canPD[1]) < 0.002) {
		PD_Value = 0; 
		return false; 
	}
	else { //penetrated
		if (canPD[0] < canPD[1]) {
			PD_Value = canPD[0];
		}
		else {
			PD_Value = canPD[1];
			PD_n = -n;
		}
		return true;
	}
}

//p: point, o:origin of axis(point on plane), n: axis normal vector (plane normal vector)
float RigidPD::coordOnAxis(vec3 p, vec3 o, vec3 n) {
	return  dot(p - o, n); // |n|==1
}
float RigidPD::calculateTetVolume(tet t) {
	float volume = abs(dot((t.vertex[0] - t.vertex[3]), cross(t.vertex[1] - t.vertex[3], t.vertex[2] - t.vertex[3])) / 6.0f);
	//cout << "tet's volume: " << volume << endl;
	if (volume == 0) {
		cout << "Volume is Zero!" << endl;
		(volume = 0.0000000000000000001f);
	}
	return volume;
}

void RigidPD::updateResult(int index, float time,float pd, plane pl) {
	vec3 halfPD = 0.5f*pd * pl.n;

	allResults.index[index] = index;
	allResults.optTime[index] = time;
	allResults.optValue[index]=pd;	

	initTet(allResults.pTets1[index], rTet[0].vertex[0] - halfPD, rTet[0].vertex[1] - halfPD, rTet[0].vertex[2] - halfPD, rTet[0].vertex[3] - halfPD);
	initTet(allResults.pTets2[index], rTet[1].vertex[0] + halfPD, rTet[1].vertex[1] + halfPD, rTet[1].vertex[2] + halfPD, rTet[1].vertex[3] + halfPD);

	if (index < 4) { allResults.planePoint[index] = allResults.pTets1[index].face[index][0]; }
	else if(index<8){ allResults.planePoint[index] = allResults.pTets2[index].face[index - 4][0]; }
	else { allResults.planePoint[index] = allResults.pTets2[index].edge[index/6][0]; }

	allResults.normal[index] = pl.n;
	//allResults.pl[index].d = -dot(pl.n, allResults.planePoint[index]);

	//if (rigidPD_Value > allResults.optValue[index]) {
	//	//rigidPD_Value = allResults.optValue[index];
	//	minOptIndex = index;
	//}
}


void RigidPD::printV3(vec3 v) {
	cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
}
void RigidPD::printResult() {
	cout << "[Optimization Result]" << endl;
	cout << "1. Minimum metric value: " << rigidPD_Value << endl;
	cout << "2. Total Optimization time:" << totalOptTime << endl;
	cout << "3. Separating plane from: " << minOptIndex << endl;

	for (int i = 0; i < 44; i++) {
		cout << "pl["<<i<<"].normal=";
		printV3(allResults.normal[i]);
		cout << "pd[" << i << "]= " << allResults.optValue[i] << endl;
		cout << endl;
	}
}

void RigidPD::initTet(tet &T, vec3 v0, vec3 v1, vec3 v2, vec3 v3) {
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
void RigidPD::fprintV3(ofstream &fp, vec3 v) {
	fp << v.x << "," << v.y << "," << v.z;

}


void RigidPD::writeCSVHead(string fileName) {
	
	ofstream output(fileName + ".csv",ios::app);

	output << " , INPUT,,,,,,,,OUTPUT,,,,,,,," << endl;
	output << "No. , Tet1,,,,Tet2,,,,DeformedTet1,,,,DeformedTet2,,,,PD(obj. norm),Opt. Time,Sep. Index,Sep. Pair,Sep. Normal,,, Volume Before1, Volme After1, Volume Before1, Volme After1" << endl;

	output.close();
}

void RigidPD::writeCSV(string fileName) {
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

	output << rigidPD_Value << ",";
	output << totalOptTime << ",";
	output << minOptIndex << ",";
	output << pair << ",";


	fprintV3(output, allResults.normal[minOptIndex]);
	output << "," << rVolume[0] <<
		"," << calculateTetVolume(pTet[0]);
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
