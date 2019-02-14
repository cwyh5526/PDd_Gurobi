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
