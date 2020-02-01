#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4f MakeConnectedTrans();
	Eigen::Matrix4f GetConnectedTransIfNeeded(int index, int object_index){
		return ((index >= object_index) ? MakeTrans() : MakeConnectedTrans());
	};

	void MyPreTranslate(Eigen::Vector3f amt);
	void MyTranslate(Eigen::Vector3f amt);
	void MyTranslate(Eigen::Vector3f amt, bool prerotation);
	void MyTranslate(Eigen::Vector3f amt, bool prerotation, Movable* scn);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	Eigen::Matrix3f GetRotationMatrix();
	void MyScale(Eigen::Vector3f amt);
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Vector3f GetCenterOfRotation();
	Eigen::Transform<float,3,Eigen::Affine> Tin; //Parent
    void TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt);
    void RotateInSystem(Eigen::Vector3f rotAxis, float angle);
private:
	Eigen::Transform<float,3,Eigen::Affine> T; //Self
};

