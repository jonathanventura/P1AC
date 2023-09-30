#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

#include <p1ac/p1ac.h>
#include <problem_generator/problem_generator.h>
using namespace problem_generator;

int main( int argc, char **argv )
{
    srand(1234);

    ProblemGenerator generator;
    
    const int nprob = 10000;

    double mean_rot_err = 0;
    double mean_pos_err = 0;
    
    for ( int i = 0; i < nprob; i++ )
    {
        Eigen::Vector2d x;
        double d;
        Eigen::Vector3d n;
        Eigen::Vector2d y;
        Eigen::Matrix2d A;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        generator.make_random_problem(x,d,n,y,A,R,t);
        
        Eigen::Quaterniond q(R);
        Eigen::Vector3d c(-R.transpose()*t);
        
        std::vector<Eigen::Matrix3d> Rsolns;
        std::vector<Eigen::Vector3d> tsolns;
        p1ac::solve(x,d,n,y,A,Rsolns,tsolns);
        
        double min_rot_err = INFINITY;
        double min_pos_err = INFINITY;
        for ( int i = 0; i < Rsolns.size(); i++ )
        {
            Eigen::Quaterniond qsoln(Rsolns[i]);
            Eigen::Vector3d csoln(-Rsolns[i].transpose()*tsolns[i]);
            double my_rot_err = q.angularDistance(qsoln);
            double my_pos_err = (c-csoln).norm();
            if ( my_rot_err < min_rot_err ) min_rot_err = my_rot_err;
            if ( my_pos_err < min_pos_err ) min_pos_err = my_pos_err;
        }
        
        mean_rot_err += min_rot_err;
        mean_pos_err += min_pos_err;
    }

    mean_rot_err /= nprob;
    mean_pos_err /= nprob;

    std::cout << "tested " << nprob << " problems.\n";
    std::cout << "average P1AC rotation error: " << mean_rot_err << "\n";
    std::cout << "average P1AC position error: " << mean_pos_err << "\n";

    return 0;
}
