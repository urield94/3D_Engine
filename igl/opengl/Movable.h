#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	void MyTranslate(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void MyScale(Eigen::Vector3f amt);
	void Reset();
	Eigen::Matrix3f GetRotationMatrix();
private:
	Eigen::Transform<float,3,Eigen::Affine> T;

};

