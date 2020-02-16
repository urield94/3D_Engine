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

void Movable::ResetTrans()
{
    T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Eigen::Matrix4f Movable::MakeConnectedTrans()
{
    return (T*Tin).matrix();
}

void Movable::MyPreTranslate(Eigen::Vector3f amt)
{
    T.pretranslate(amt);
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
        T.translate(amt);
}

void Movable::MyTranslate(Eigen::Vector3f amt, bool prerotation){
    if(prerotation)
        MyPreTranslate(amt);
    else
        MyTranslate(amt);
}

void Movable::MyTranslate(Eigen::Vector3f amt, bool prerotation, Movable* scn)
{
    MyTranslate(scn->T.rotation().matrix().inverse() * amt, prerotation);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	T.rotate(Eigen::AngleAxisf(angle, rotAxis));
}

Eigen::Matrix3f Movable::GetRotationMatrix(){
    return  T.rotation();
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	T.scale(amt);
	Tin.scale(amt);
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

void Movable::TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt)
{
    MyTranslate(T.rotation().inverse() * amt);
}

void Movable::RotateInSystem(Eigen::Vector3f rotAxis, float angle)
{
    MyRotate(T.rotation().inverse() * rotAxis, angle);
}

void Movable::SetVelocity(Eigen::Vector3f v) {
    velocity = v;
}


Eigen::Vector3f Movable::GetVelocity() {
    return velocity;
}
