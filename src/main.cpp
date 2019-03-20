#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_access.hpp>

using namespace std;
const float eps = 1e-6;

// a ray is described by its origin and normalized direction
class Ray {
public:
	glm::vec3 origin, direction;

	Ray(glm::vec3 _o, glm::vec3 _d) : origin(_o), direction(_d) {};
};

// a sphere is described by its center and radius
class Sphere {
public:
	glm::vec3 center;
	float radius;

	Sphere() {};
	Sphere(glm::vec3 _c, float _r) : center(_c), radius(_r) {};

	// simple ray sphere intersection that returns distance to the closest intersection
	float intersect(Ray ray) {
		float a = glm::dot(ray.direction, ray.direction);
		glm::vec3 s0_r0 = ray.origin - center;
		float b = 2.0 * glm::dot(ray.direction, s0_r0);
		float c = glm::dot(s0_r0, s0_r0) - (radius * radius);
		if (b*b - 4.0*a*c < 0.0) {
			return -1.0;
		}
		float t = (-b - sqrt((b*b) - 4.0*a*c)) / (2.0*a);
		if (t <= 0.0) t = (-b + sqrt((b*b) - 4.0*a*c)) / (2.0*a);
		return t;
	}

	// a matrix is applied to the sphere and a new sphere is returned
	Sphere applyTransformation(glm::mat4 m) {
		glm::vec3 translation = glm::row(m, 3) * radius;
		glm::vec3 scale;
		scale.x = glm::length(glm::vec3(m[0][0], m[0][1], m[0][2]));
		scale.y = glm::length(glm::vec3(m[1][0], m[1][1], m[1][2]));
		scale.z = glm::length(glm::vec3(m[2][0], m[2][1], m[2][2]));
		float largestScale = glm::max(scale.x, glm::max(scale.y, scale.z));
		glm::vec3 c = translation + center;
		return Sphere(c, radius * largestScale);
	}
	glm::vec3 normal(glm::vec3 hitpoint) {
		return hitpoint - center;
	}
};

// scene settings
int maxDepth, width, height, numberOfNormals;
vector<glm::mat4> IFS;
glm::vec3 lightPosition, lightA, lightD, lightS;
float shadowFactor;
glm::vec3 matA, matD, matS;
float matShininess;
glm::vec3 cameraPosition;
Sphere initialBS;

// calculate the ray direction by the pixel positions on the camera plane
glm::vec3 getCameraRayDirection(int width, int height, int x, int y) {
	int w = width - 1; int h = height - 1;
	glm::vec2 imagePlaneLength(2.0, 2.0 * ((float)h / (float)w));
	float dirX = (float)x / (float)w * imagePlaneLength.x - imagePlaneLength.x / 2.0;
	float dirY = (float)y / (float)h * imagePlaneLength.y - imagePlaneLength.y / 2.0;
	return glm::vec3(dirX, dirY, 1);
}

// trace the shadow ray through the scene
bool traceShadow(Ray ray, Sphere BS, int depth, string IDoriginal, string ID) {
	float t = BS.intersect(ray);
	// prevent self intersection
	if (t > eps) {
		if (depth >= maxDepth && IDoriginal != ID) {
			// at the maximum depth
			// return true only if the current sphere is occluded by another sphere than itself
			return true;
		} else {
			// traverse hierarchy
			// create a new sphere with the applied matrix and trace further
			for (int i = 0; i < IFS.size(); i++) {
				Sphere s = BS.applyTransformation(IFS.at(i));
				if (traceShadow(ray, s, depth + 1, IDoriginal, ID + to_string(i))) return true;
			}
		}
	}
	return false;
}

// lighting calculations with ambient, diffuse and specular term
glm::vec3 calculateLighting(glm::vec3 hitpoint, glm::vec3 N, glm::vec3 L, glm::vec3 V, string ID) {
	glm::vec3 ambient = matA * lightA;
	glm::vec3 diffuse = matD * lightD * max(0.0f, min(glm::dot(N, L), 1.0f));
	float specularTerm = dot(N, L) > 0 ? specularTerm = pow(dot(N, normalize(L + V)), matShininess) : 0;
	glm::vec3 specular = matS * lightS * specularTerm;
	glm::vec3 color = ambient + diffuse + specular;

	// cast a shadow ray and change the color accordingly
	Ray r(hitpoint, L);
	if (traceShadow(r, initialBS, 0, ID, "")) color = glm::vec3(shadowFactor) * color;
	return color;
}

void trace(Ray ray, Sphere BS, vector<glm::vec3> normals, int depth, string ID, glm::vec4 &color) {
	float t = BS.intersect(ray);
	// progress only if no self intersection and no intersection that is already closer
	if (t > eps && (t < color.a || color.a == -1)) {
		if (depth >= maxDepth) {
			// at the maximum depth
			// lighting calculation only if the current depth is smaller than previously stored depth 
			if ((t < color.a || color.a == -1)) {
				// calculate all necessary values for lighting calculations
				glm::vec3 hitpoint = ray.origin + ray.direction * t;
				glm::vec3 N = glm::normalize(BS.normal(hitpoint));
				for (int i = 0; i < normals.size(); i++) N += glm::normalize(normals.at(i));
				N /= (normals.size() + 1);
				glm::vec3 L = glm::normalize(lightPosition - hitpoint);
				glm::vec3 V = glm::normalize(cameraPosition - hitpoint);
				color = glm::vec4(calculateLighting(hitpoint, N, L, V, ID), t);
			}
		} else {
			// store the current normal (not normalized for performance reasons)
			glm::vec3 normal = BS.normal(ray.origin + ray.direction * t);
			if (normals.size() < numberOfNormals - 1) {
				normals.push_back(normal);
			} else {
				normals.erase(normals.begin());
				normals.push_back(normal);
			}
			// traverse hierarchy
			// create a new sphere with the applied matrix and trace further
			for (int i = 0; i < IFS.size(); i++) {
				Sphere s = BS.applyTransformation(IFS.at(i));
				trace(ray, s, normals, depth + 1, ID + to_string(i), color);
			}
		}
	}
}

// remove comment from scene file
void removeComment (ifstream *sceneFile) {
	char c = ' ';
	while (c != '\n' && &sceneFile) sceneFile->get(c);
}

// read a number from scene file
float readNumber (ifstream *sceneFile) {
	float value;
	*sceneFile >> value;
	removeComment(sceneFile);
	return value;
}

// read a vector from scene file
glm::vec3 readVector (ifstream *sceneFile) {
	glm::vec3 value;
	for (int i = 0; i < 3; i++) *sceneFile >> value[i];
	removeComment(sceneFile);
	return value;
}

// read a scene file
bool readSceneFile(std::string &name) {
	// wait for an input
	string fileName;
	cout << "Input scene file: ";
	getline(cin, fileName);

	// check if the file does exist
	ifstream sceneFile;
	sceneFile.open(fileName);
	if (!sceneFile) {
		cerr << "Unable to open scene file \"" << fileName << "\".\n";
		return false;
	}

	// assign name
	name = fileName.substr(0, fileName.size() - 4);

	// assign all values from the scene file
	width = readNumber(&sceneFile);
	height = readNumber(&sceneFile);
	maxDepth = readNumber(&sceneFile);
	numberOfNormals = readNumber(&sceneFile);
	initialBS.center = readVector(&sceneFile);
	initialBS.radius = readNumber(&sceneFile);
	cameraPosition = readVector(&sceneFile);
	lightPosition = readVector(&sceneFile);
	lightA = readVector(&sceneFile);
	lightD = readVector(&sceneFile);
	lightS = readVector(&sceneFile);
	shadowFactor = readNumber(&sceneFile);
	matA = readVector(&sceneFile);
	matD = readVector(&sceneFile);
	matS = readVector(&sceneFile);
	matShininess = readNumber(&sceneFile);
	
	// read in number of matrices
	int numberOfMatrices = readNumber(&sceneFile);

	// read in matrices
	float m[16];
	for (int i = 0; i < numberOfMatrices; i++) {
		for (int j = 0; j < 16; j++) {
			sceneFile >> m[j];
		}
		IFS.push_back(glm::make_mat4(m));
	}

	sceneFile.close();
	return true;
}

void main() {

	// wait for scene
	std::string name;
	while (!readSceneFile(name));

	// initialize pixel
	glm::vec3 **pixel = new glm::vec3 *[width];
	for (int i = 0; i < width; i++) {
		pixel[i] = new glm::vec3[height];
	}

	// render loop, runs on multiple processors
	#pragma omp parallel for schedule(dynamic)
	for (int j = 0; j < height; j++) {
		fprintf(stdout, "%f\r", (j / (float)width));
		for (int i = 0; i < width; i++) {
			// get starting ray
			Ray ray(cameraPosition, glm::normalize(getCameraRayDirection(width, height, i, j)));
			glm::vec4 color(0, 0, 0, -1);
			vector<glm::vec3> normals;

			// trace and store color
			trace(ray, initialBS, normals, 0, "", color);
			pixel[i][j] = color;
		}
	}

	// save to ppm image
	FILE *f = fopen(string(name + ".ppm").c_str(), "w");
	fprintf(f, "P3\n%d %d\n%d\n ", width, height, 255);
	for (int j = height - 1; j > 0; j--) {
		for (int i = 0; i < width; i++) {
			fprintf(f, "%d %d %d ", min((int)(pixel[i][j].x*255.0), 255), min((int)(pixel[i][j].y*255.0), 255), min((int)(pixel[i][j].z*255.0), 255));
		}
		fprintf(f, "\n");
	}
	fclose(f);
}