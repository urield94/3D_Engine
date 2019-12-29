#include <GL/gl.h>
#include <iostream>
#include "Movable.h"

Movable::Movable()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return T.matrix();
}

void Movable::MyTranslate(Eigen::Vector3f amt, bool preRotation) {
    if (preRotation) {
        T.pretranslate(amt);
    } else {
        T.translate(amt);
    }
}

//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
//    SetCenterOfRotation(GetCenterOfRotation());
//    T.translate(GetCenterOfRotation());
	T.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
//    Tin.translate(GetCenterOfRotation());
//    SetCenterOfRotation(-GetCenterOfRotation());
}

void Movable::MyScale(Eigen::Vector3f amt)
{
    Tin.scale(amt);
	T.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt)
{
    Tin.translate(-amt);
    T.translate(amt);
}

Eigen::Vector3f Movable::GetCenterOfRotation()
{
    return -Tin.translation();
}

void Movable::TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt, bool preRotation)
{
    MyTranslate(mat.block<3, 3>(0, 0).transpose() * amt, preRotation);
}

void Movable::RotateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f rotAxis, float angle)
{
    MyRotate(mat.block<3, 3>(0, 0).transpose() * rotAxis, angle);
}
