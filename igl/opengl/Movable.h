#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4f  MakeConnectedTrans();
	void MyTranslate(Eigen::Vector3f amt);
	void MyTranslate(Eigen::Vector3f amt, bool preRotation);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void MyScale(Eigen::Vector3f amt);
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Vector3f GetCenterOfRotation();
	Eigen::Transform<float,3,Eigen::Affine> Tin; //Parent
    void TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt, bool preRotation);
    void RotateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f rotAxis, float angle);
	void ScaleAndTranslate(Eigen::Vector3f amt, Movable* scene);
private:
	Eigen::Transform<float,3,Eigen::Affine> T; //Self
};

