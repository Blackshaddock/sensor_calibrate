#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0


template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>&& ang);

template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt);

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3);


template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rot);


template<typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R);




#endif
