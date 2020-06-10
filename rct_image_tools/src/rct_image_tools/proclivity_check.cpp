#include <rct_image_tools/proclivity_check.h>
#include <Eigen/Dense>

bool checkObservationProclivity(const rct_image_tools::ProclivityParams& params)
{

  Eigen::MatrixXd Alpha(8,1); //Homography matrix components
  Eigen::MatrixXd A(18,8);
  Eigen::MatrixXd B(1,18); //U,V coordinates of points

  // determine rows and cols of target
  int rows = params.target.rows;
  int cols = params.target.cols;

    //we want *9* points, spiraling in clockwise: UL, UR, LL, LR, UL +1, UR -1. etc.
  std::vector<int> selected_point_index{0, rows-1, rows*cols -1, rows*cols - cols-1,
                                        1, rows-2, rows*cols -2,  rows*cols - cols-2,
                                        rows*cols/2};

  // build matrix of type Ax=b where A x is the unknown elements of the proclivity matrix "alpha"
  int row = 0;
  for(int i=0; i<(int) selected_point_index.size(); i++){

    //pt of interest index
    rct_optimizations::Correspondence2D3D pti = params.ob.correspondence_set[selected_point_index[i]]; // index of selected point in correspondence

    double xi = pti.in_target(0);
    double yi = pti.in_target(1);
    //spatial location of target; may be better to pull data from the correspondances

    double Ui = pti.in_target(0);
    double Vi = pti.in_target(1);
    //U,V position of the target from the correspondance

    //assign A row-th row:
    A.row(row) << -xi, -yi, -1.0, 0.0, 0.0, 0.0, Ui*xi, Ui*yi;
    B.row(row) << -Ui;
    row++;

    A.row(row) << 0.0, 0.0, 0.0, -xi, -yi, -1.0,  Vi*xi, Vi*yi;
    B.row(row) << -Vi;
    row++;
  }
  //solve with Eigen
  //beware gcc's -ffast-math, hurts accuracy of JacobiSvd
  Alpha = A.jacobiSvd().solve(B);

  // construct the Proclivity matrix from Alpha
  Eigen::Matrix3d P;
  P << Alpha;
  P(2,2) = 1.0;

  // check the proclivity of every observation
  bool rtn = true;
  double ave_error = 0.0;
  std::vector<double> errorU, errorV;
  //Eigen::MatrixXd errors(params.ob.correspondence_set.size(), 2);
  for(int i=0; i<(int) params.ob.correspondence_set.size(); i++)
  {
    //int pi = CO[i].point_id;
    rct_optimizations::Correspondence2D3D pti = params.ob.correspondence_set[i];
    double xi = pti.in_target(0);
    double yi = pti.in_target(1);

    double Ui = pti.in_target(0);
    double Vi = pti.in_target(1);

    double ki = 1.0/(Alpha(6)*xi + Alpha(7)*yi + 1.0);
    Eigen::Vector3d UV;
    Eigen::Vector3d X(xi,yi,1);

    UV = ki*P*X;


    double EU = Ui - UV(0);
    double EV = Vi - UV(1);
    ave_error += sqrt(EU*EU + EV*EV);
    if (params.max_residual > 0) //pass negative residual_threshold to do outlier detection
    {
      if(fabs(EU)>params.max_residual || fabs(EV)>params.max_residual)
      {
        rtn = false;
      }
    }
    else
    {
      errorU.push_back(EU);
      errorV.push_back(EV);
    }
  }
  //use check for outlier with interquartile range if no threshhold provided
  if (params.max_residual<= 0)
  {
    //calculate interquartile range, check that top value is not more than that 1.5 that value
    std::sort(errorU.start(), errorU.end());
    std::sort(errorV.start(), errorV.end());

    ASSERT(errorU.size() == params.ob.correspondence_set.size() && errorV.size() == params.ob.correspondence_set.size());
    //will have as many error vector entries as correspondences

    std::size_t Q1i = params.ob.correspondence_set.size()/4;
    std::size_t Q3i = (params.ob.correspondence_set.size() * 3) /4;

    double IQRU = errorU[Q3i] - errorU[Q1i];
    double IQRV = errorV[Q3i] - errorV[Q1i];

    //if the max value is larger than 1.5 x the IQR, it is likely an outlier
    if (errorU.back() > IQRU * 1.5 || errorV.back() > IQRV * 1.5)
    {
      rtn = false;
    }
  }
  ave_error = ave_error/(int) params.ob.correspondence_set.size();
    return(rtn);
};
