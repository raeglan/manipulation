#pragma once

//https://stackoverflow.com/questions/6978643/cuda-and-classes 10.08.2017 19 uhr
#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER __host__ __device__
#else
#define CUDA_CALLABLE_MEMBER
#endif 

#include <math.h>
#include <iostream>

struct Vector3f {
	CUDA_CALLABLE_MEMBER Vector3f(float x, float y, float z) : x(x), y(y), z(z) {};
	CUDA_CALLABLE_MEMBER Vector3f() : x(0), y(0), z(0) {};
	CUDA_CALLABLE_MEMBER Vector3f cross(Vector3f v) const;
	CUDA_CALLABLE_MEMBER float norm() const;
	float x;
	float y;
	float z;
};


struct Point3f {
	CUDA_CALLABLE_MEMBER Point3f(Vector3f position) : position(position) {};
	CUDA_CALLABLE_MEMBER Point3f(float x, float y, float z) : position(Vector3f(x, y, z)) {};
	CUDA_CALLABLE_MEMBER Point3f() : position(Vector3f()) {};
	Vector3f position;
};

struct Point3fn {
	CUDA_CALLABLE_MEMBER Point3fn(Vector3f position, Vector3f normal) : position(position), normal(normal) {};
	CUDA_CALLABLE_MEMBER Point3fn() : position(Vector3f()), normal(Vector3f(1, 0, 0)) {};
	Vector3f position;
	Vector3f normal;
};

//a*x1 + b*x2 + c*x3 + d=0
struct Plane {
	CUDA_CALLABLE_MEMBER Plane(float a, float b, float c, float d) : a(a), b(b), c(c), d(d) {};
	CUDA_CALLABLE_MEMBER Plane() : a(1), b(0), c(0),d(0) {};
	float a;
	float b;
	float c;
	float d;
};



inline CUDA_CALLABLE_MEMBER Vector3f Vector3f::cross(Vector3f v) const {
	return Vector3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

inline CUDA_CALLABLE_MEMBER float Vector3f::norm() const {
	return sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
}

inline CUDA_CALLABLE_MEMBER Vector3f operator+(const Vector3f& v1, const Vector3f& v2) {
	return Vector3f(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}


inline CUDA_CALLABLE_MEMBER Vector3f operator-(const Vector3f& v1, const Vector3f& v2) {
	return Vector3f(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

inline CUDA_CALLABLE_MEMBER Vector3f operator*(const float& f, const Vector3f& v) {
	return Vector3f(v.x * f, v.y * f, v.z * f);
}

inline CUDA_CALLABLE_MEMBER Vector3f operator/(const Vector3f& v, const float& f) {
	return Vector3f(v.x / f, v.y / f, v.z / f);
}

inline CUDA_CALLABLE_MEMBER Vector3f operator-(const Point3f& p1, const Point3f& p2) {
	return p1.position - p2.position;
}

inline CUDA_CALLABLE_MEMBER Point3f operator-(const Point3f& p, const Vector3f& v) {
	return Point3f(p.position - v);
}

inline CUDA_CALLABLE_MEMBER Point3f operator+(const Point3f& p, const Vector3f& v) {
	return Point3f(p.position + v);
}

inline CUDA_CALLABLE_MEMBER float operator*(const Vector3f& v1, const Vector3f& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}


inline CUDA_CALLABLE_MEMBER Point3f operator/(const Point3f& p, const float& f) {
	return Point3f(p.position.x / f, p.position.y / f, p.position.z / f);
}


inline std::ostream& operator<<(std::ostream& os, Point3f& p1) {
	os << "x: " << p1.position.x << " y: " << p1.position.y << " z: " << p1.position.z;
	return os;
}

inline std::ostream& operator<<(std::ostream& os, Plane& p) {
	os << "a: " <<p.a << " b: " <<p.b << " c: " << p.c << " d: " << p.d;
	return os;
}

inline std::ostream& operator<<(std::ostream& os, Vector3f& v) {
	os << "x: " <<v.x << " y: " << v.y << " z: " << v.z;
	return os;
}


