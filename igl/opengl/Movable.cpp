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

Eigen::Matrix4f Movable::MakeConnectedTrans()
{
    return (T*Tin).matrix();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
    T.pretranslate(amt);
}

void Movable::MyTranslate(Eigen::Vector3f amt, bool preRotation)
{
    if(preRotation)
        T.pretranslate(amt);
    else
        T.translate(amt);
}

//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	T.rotate(Eigen::AngleAxisf(angle, rotAxis));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	T.scale(amt);
}

void Movable::ScaleAndTranslate(Eigen::Vector3f amt, Movable* scene)
{
    Eigen::Matrix3f mat = scene->T.rotation().matrix().inverse();
    MyTranslate(mat*amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt)
{
    Tin.translate(-(amt).cast<float>());
    T.translate(amt.cast<float>());
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
    MyRotate(T.rotation().inverse() * rotAxis, angle);
//    Eigen::Matrix3f inverse_tout = T.rotation().inverse();
//    T.rotate(Eigen::AngleAxisf(angle, inverse_tout*rotAxis));
}
