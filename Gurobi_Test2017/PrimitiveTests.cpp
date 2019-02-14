//#include "DataStructure.h"
//
//vec3 ClosetPtPointPlane(vec3 q, plane pl) {
//	float t = dot(pl.n, q) + pl.d; // |n|=1
//	return q - t*pl.n;
//}
//float signedDistPointPlane(vec3 q, plane pl) {
//	return dot(pl.n, q) + pl.d; // |n|=1
//}
//typedef struct {
//	float min;
//	float max;
//}interval;
//
//vec3 projectOnAxs(vec3 o, vec3 n, float coord) {
//	return o + coord*n;
//}
//vec3 projectOnAxs(vec3 p, vec3 o, vec3 n) {
//	return o + dot(p - o, n)*n;
//}
////ax+by+cz+d=0 --> d = -n¡¤q
////p:point, n: plane normal vector, d: constant of plane eq.
//float signedDistancePointPlane(vec3 p, plane pl) {
//	return (dot(p, pl.n) + pl.d);
//}
//
////p: point, n: plane normal vector, q: point on plane
//float signedDistancePointPlane(vec3 p, vec3 n, vec3 q) {
//	return (dot(p - q, n));
//}