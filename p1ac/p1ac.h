
#include <Eigen/Dense>
#include <vector>

namespace p1ac {

    /**
     * P1AC minimal solver.
     * Computes absolute pose solutions for the query image given
     * an affine correspondence between the reference and query image,
     * the depth of the point in the reference image,
     * and the surface normal at the 3D point.
     *
     * The solver assumes that the reference image has identity pose [I | 0]
     * and that the surface normal is given in the reference image's coordinate system.
     *
     * The rotation and translation solutions represent world-to-camera transformations.
     *
     * Reference: Ventura, J., Kukelova, Z., Sattler, T., and Barath, D. "P1AC: Revisiting absolute pose
     * from a single affine correspondence." International Conference on Computer Vision, 2023.
     *
     * \param x[in] point observation in reference image
     * \param d[in] depth of point in reference image
     * \param n[in] surface normal in reference camera's coordinate system
     * \param y[in] point observation in query image
     * \param A[in] affine transformation between reference image and query image
     * \param Rsolns[out] 3x3 rotation matrix solutions for query image (world-to-camera)
     * \param tsolns[out] 3x1 translation vector solutions for query image (world-to-camera)
     */
    void solve(
        const Eigen::Vector2d &x,
        const double d,
        const Eigen::Vector3d &n,
        const Eigen::Vector2d &y,
        const Eigen::Matrix2d &A,
        std::vector<Eigen::Matrix3d> &Rsolns,
        std::vector<Eigen::Vector3d> &tsolns
    );

}
