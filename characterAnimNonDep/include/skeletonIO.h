#ifndef _SKELETON_IO_H_
#define _SKELETON_IO_H

#include "skeleton.h"

Skeleton* readHierarchy(std::ifstream &inputfile);
Skeleton* interpolate(std::ifstream &inputfile1, std::ifstream &inputfile2 );
Skeleton* transition(std::ifstream &inputfile1, std::ifstream &inputfile2);

void readJoint(std::ifstream &inputfile, Skeleton* parent);
void interpolateJoint(std::ifstream &inputfile, std::ifstream &inputfile2, Skeleton* parent);

void readMotion(std::ifstream &inputfile, Skeleton* root);
void interpolateMotion(std::ifstream &inputfile1, std::ifstream &inputfile2, Skeleton* root);
std::vector<double> transitionMotion(std::ifstream &inputfile1, Skeleton* root, double lastX, double lastY, double lastZ);


void readKeyFrame(std::ifstream &inputfile, Skeleton* skel, double Xinit = 0, double Yinit =0, double Zinit = 0);
void interpolateKeyFrame(std::ifstream &inputfile1, std::ifstream &inputfile2, Skeleton* skel);

void defineRotateOrder(Skeleton *skel);

void skipTransitionFrames(int nbTransitionFrames, Skeleton* root);
void updateTransitionFrames(int nbFrames, int nbTransitionFrames, Skeleton* root);
#endif
