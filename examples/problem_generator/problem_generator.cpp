
#include <Eigen/Geometry>

#include <problem_generator/random.h>
#include <problem_generator/problem_generator.h>

#include <iostream>

namespace problem_generator
{
    static Eigen::Matrix3d calculateHomography( const Eigen::Vector3d &X, const Eigen::Vector3d &n, const Eigen::Matrix3d &R, const Eigen::Vector3d &t )
    {
        double d = -n.dot(X);
        
        Eigen::Vector3d v = -n/d;
        Eigen::Matrix3d H = R + t * v.transpose();
        
        return H;
    }

    static Eigen::Matrix2d affineFromHomography( const Eigen::Matrix3d &H, const Eigen::Vector2d &x, const Eigen::Vector2d &y )
    {
        double s = H(2,0)*x(0) + H(2,1)*x(1) + H(2,2);
        
        Eigen::Matrix2d A;
        A << (H(0,0)-H(2,0)*y(0))/s, (H(0,1)-H(2,1)*y(0))/s,
             (H(1,0)-H(2,0)*y(1))/s, (H(1,1)-H(2,1)*y(1))/s;
        
        return A;
    }

    void ProblemGenerator::make_random_problem(
        Eigen::Vector2d &x,
        double &d,
        Eigen::Vector3d &n,
        Eigen::Vector2d &y,
        Eigen::Matrix2d &A,
        Eigen::Matrix3d &R,
        Eigen::Vector3d &t
    )
    {
        // generate random rotation and translation for query camera
        R = Eigen::Quaterniond::UnitRandom().toRotationMatrix();
        t = generator.rand_unit_vector() * 0.1;

        // iterate until valid configuration found
        while ( true ) {
            // make 3D point
            Eigen::Vector3d X = generator.randn3();
                
            // compute depth in reference camera
            d = X(2);
            
            // check if behind reference camera
            if ( d < 0 ) continue;
            
            // project to reference camera
            x = X.head(2)/X(2);
                
            // sample random normal vector
            n = generator.rand_unit_vector();
                
            // project to query camera
            Eigen::Vector3d PX = R * X + t;
            y = PX.head(2)/PX(2);

            // check if behind query camera
            if ( PX(2) < 0 ) continue;
                
            // calculate homography
            Eigen::Matrix3d H = calculateHomography(X,n,R,t);
            
            // calculate affine from homography
            A = affineFromHomography(H,x,y);
            
            // check if determinant is positive
            if ( A.determinant() < 0 ) continue;
            
            break;
        }
    }

}
