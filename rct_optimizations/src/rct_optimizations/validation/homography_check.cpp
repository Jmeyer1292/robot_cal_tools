#include <rct_optimizations/validation/homography_check.h>
#include <Eigen/Dense>

bool checkObservationSequence(const int& rows,
                              const int& cols,
                              rct_optimizations::Observation2D3D& ob,
                              const double& max_residual)
{

  /*points used for homography; cannot represent more than 1/2
  * target points (to keep an adequate validaiton set) and cannot
  * be less than 4
  */
  int sampled_points = 4;

  //Ensure that there are enough points for testing outside of the sampled set
  assert(rows * cols > 2*sampled_points);

  //Alpha is used to find the unique elements of the homography matrix
  Eigen::MatrixXd Alpha(8,1);

  /*The reduced form of the homography relations of a point x,y and its image
   * representation U,V. Used to find the Homography matrix components.
   */
  Eigen::MatrixXd A(2*sampled_points,8);

  //U,V coordinates of sampled points
  Eigen::MatrixXd B(1,2 * sampled_points);
  std::vector<int> selected_point_index;
  int points_selected = 0;
  while (points_selected < sampled_points)
  {
    //add one to prevent divide by zero
    int corner = points_selected + 1 % 4;

    //Nominally 4 points will be used (minumum requirement) but this will allow as many points as are in the target to be used.
    ///Does not currently prevent duplicate points
    switch(corner)
    {
      case 1: selected_point_index.push_back(floor(points_selected/4)); //points in the upper left corner; the 1st, 5th, 9th point included
              break;

      case 2: selected_point_index.push_back(cols - points_selected/4); //Upper right points; 2, 6, 10, etc.
              break;

      case 3: selected_point_index.push_back(rows*cols - points_selected/4); //lower right points selected; 3, 7, 11
              break;

      case 4: selected_point_index.push_back((rows-1)*cols + points_selected-1/4); //lower left corner points; 4, 8, 12, etc.
              break;
    }

    //switch statement for handling spiral case
    //select points in spiral from corners
    points_selected ++;
  }

  // build matrix of type Ax=b where A x is the unknown elements of the homography matrix "alpha"
  for(std::size_t i=0; i<selected_point_index.size(); i+=2)
  {

    //pt of interest index
    const rct_optimizations::Correspondence2D3D& pti = ob.correspondence_set[selected_point_index[i]];

    double xi = pti.in_target(0);
    double yi = pti.in_target(1);
    //spatial location of target; may be better to pull data from the correspondances

    double Ui = pti.in_target(0);
    double Vi = pti.in_target(1);
    //U,V position of the target from the correspondance

    //assign A row-th row:
    A.row(i) << -xi, -yi, -1.0, 0.0, 0.0, 0.0, Ui*xi, Ui*yi;
    B.row(i) << -Ui;

    A.row(i+1) << 0.0, 0.0, 0.0, -xi, -yi, -1.0,  Vi*xi, Vi*yi;
    B.row(i+1) << -Vi;
  }
  //solve with Eigen

  //This Svd methiod can be negetively impacted by using gcc's -ffast-math flag
  Alpha = A.jacobiSvd().solve(B);

  // construct the Homograpahy matrix from Alpha
  Eigen::Matrix3d P;
  P << Alpha;
  P(2,2) = 1.0;

  //Use the mapping constructed in Alpha to verify the remaining points
  bool rtn = true;

  for(int i=0; i<(int) ob.correspondence_set.size(); i++)
  {
    const rct_optimizations::Correspondence2D3D& pti = ob.correspondence_set[i];

    Eigen::Vector2d spatial_coord(pti.in_target(0), pti.in_target(1));
    Eigen::Vector2d image_coord(pti.in_image(0), pti.in_image(1));


    double ki = 1.0/(Alpha(6)*spatial_coord(0) + Alpha(7)*spatial_coord(1) + 1.0);
    //Vector of calculated UV positions;
    Eigen::Vector3d UV;
    Eigen::Vector3d X(spatial_coord(0), spatial_coord(1), 1);

    UV = ki*P*X;

    Eigen::Vector2d homography_error;
    homography_error = image_coord - UV.head(2);

    if(fabs(homography_error(0))>max_residual || fabs(homography_error(1))>max_residual)
    {
      rtn = false;
    }
  }

  //ave_error = ave_error/(int) ob.correspondence_set.size();
    return(rtn);
};
