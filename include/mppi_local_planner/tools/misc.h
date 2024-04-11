#ifndef MPPI_LOCAL_PLANNER_TOOLS_MISC_H
#define MPPI_LOCAL_PLANNER_TOOLS_MISC_H

#include <Eigen/Core>
#include <boost/utility.hpp>
#include <boost/type_traits.hpp>


namespace mppi
{

#define SMALL_NUM 0.00000001
#define CONST_PI 3.14159265358979323846

//! Symbols for left/none/right rotations
enum class RotType { left, none, right };

/**
 * @brief Check whether two variables (double) are close to each other
 * @param a the first value to compare
 * @param b the second value to compare
 * @param epsilon precision threshold
 * @return \c true if |a-b| < epsilon, false otherwise
 */
inline bool is_close(double a, double b, double epsilon = 1e-4)
{
  return std::fabs(a - b) < epsilon;
}

/**
 * @brief Return the average angle of an arbitrary number of given angles [rad]
 * @param angles vector containing all angles
 * @return average / mean angle, that is normalized to [-pi, pi]
 */
inline double average_angles(const std::vector<double>& angles)
{
    double x=0, y=0;
    for (std::vector<double>::const_iterator it = angles.begin(); it!=angles.end(); ++it)
    {
        x += cos(*it);
        y += sin(*it);
    }
    if(x == 0 && y == 0)
        return 0;
    else
        return std::atan2(y, x);
}

/** @brief Small helper function: check if |a|<|b| */
inline bool smaller_than_abs(double i, double j) {return std::fabs(i)<std::fabs(j);}


/**
 * @brief Calculate a fast approximation of a sigmoid function
 * @details The following function is implemented: \f$ x / (1 + |x|) \f$
 * @param x the argument of the function
*/
inline double fast_sigmoid(double x)
{
    return x / (1 + fabs(x));
}

/**
 * @brief normalize_theta
 * @param theta
 * @return normalized theta
 */
inline double normalize_theta(double theta)
{
    if (theta >= -CONST_PI && theta < CONST_PI)
        return theta;

    double multiplier = std::floor(theta / (2*CONST_PI));
    theta = theta - multiplier*2*CONST_PI;
    if (theta >= CONST_PI)
        theta -= 2*CONST_PI;
    if (theta < -CONST_PI)
        theta += 2*CONST_PI;

    return theta;
}

/**
 * @brief Calculate Euclidean distance between two 2D point datatypes
 * @param point1 object containing fields x and y
 * @param point2 object containing fields x and y
 * @return Euclidean distance: ||point2-point1||
*/
template <typename P1, typename P2>
inline double distance_points2d(const P1& point1, const P2& point2)
{
    return std::sqrt( std::pow(point2.x-point1.x,2) + std::pow(point2.y-point1.y,2) );
}


/**
 * @brief Calculate the 2d cross product (returns length of the resulting vector along the z-axis in 3d)
 * @param v1 object containing public methods x() and y()
 * @param v2 object containing fields x() and y()
 * @return magnitude that would result in the 3D case (along the z-axis)
*/
template <typename V1, typename V2>
inline double cross2d(const V1& v1, const V2& v2)
{
    return v1.x()*v2.y() - v2.x()*v1.y();
}

/**
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 *
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T& val).
 * @param ptr pointer of type T
 * @tparam T arbitrary type
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T* ptr) {return *ptr;}

/**
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 *
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T* val).
 * @param val
 * @param dummy SFINAE helper variable
 * @tparam T arbitrary type
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T& val, typename boost::disable_if<boost::is_pointer<T> >::type* dummy = 0) {return val;}

} // namespace mppi

#endif /* MPPI_LOCAL_PLANNER_TOOLS_MISC_H */

