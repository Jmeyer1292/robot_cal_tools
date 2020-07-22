#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace rct_optimizations
{
/** @brief A double value identified by one or two name strings. */
struct NamedParam
{
  /** @brief Pair of names identifying this parameter. For types with just one name (e.g. standard deviation), only names.first is used. */
  std::pair<std::string, std::string> names;
  /** @brief Value of this parameter. */
  std::double_t value;

  /**
   * @brief Format the NamedParam as a string.
   * @return
   */
  std::string toString() const
  {
    std::stringbuf buf;
    std::ostream os(&buf);
    os << names.first.c_str() << " " << names.second.c_str() << " " << value;;
    return buf.str();
  }
};

/**
 * @brief Covariance results for optimization parameters.
 * Contains standard deviations, covariances, and correlation coefficients, as well as the original covariance and correlation matrices.
 */
struct CovarianceResult
{
  /** @brief standard deviations */
  std::vector<NamedParam> standard_deviations;
  /** @brief correlation_coeffs */
  std::vector<NamedParam> correlation_coeffs;
  /** @brief covariances */
  std::vector<NamedParam> covariances;
  /** @brief Covariance matrix output from Ceres */
  Eigen::MatrixXd covariance_matrix;
  /** @brief Correlation matrix */
  Eigen::MatrixXd correlation_matrix;

  /**
   * @brief Returns named correlation coefficients that exceed @ref threshold.
   * @param Magnitude of a correlation coefficient that will result in it being returned.
   * @return Vector of NamedParams for correlation coefficients above @ref threshold.
   */
  std::vector<NamedParam> getCorrelationCoeffOutsideThreshold(const std::double_t& threshold) const
  {
    std::vector<NamedParam> out;
    for (auto corr : correlation_coeffs)
    {
      if (std::abs(corr.value) > threshold)
        out.push_back(corr);
    }
    return out;
  }

  /**
   * @brief Format NamedParam contents as a string.
   * @return
   */
  std::string toString() const
  {
    std::string out;
    out.append("Std. Devs.\n");
    for (auto std_dev : standard_deviations)
    {
      out.append(std_dev.toString() + "\n");
    }

    out.append("\nCovariance\n");
    for (auto cov : covariances)
    {
      out.append(cov.toString() + "\n");
    }

    out.append("\nCorrelation Coeffs.\n");
    for (auto corr : correlation_coeffs)
    {
      out.append(corr.toString() + "\n");
    }
    return out;
  }

  /**
   * @brief Compose a string with a list of NamedParams for correlation coefficeints above @ref threshold.
   * @param threshold
   * @return
   */
  std::string printCorrelationCoeffAboveThreshold(const std::double_t& threshold) const
  {
    auto above_thresh = getCorrelationCoeffOutsideThreshold(threshold);

    if (above_thresh.size() == 0)
    {
      return std::string("No correlation coefficients with magnitude > " + std::to_string(threshold) + "\n");
    }

    std::string out("\nCorrelation Coeff. > " + std::to_string(threshold) + ":\n");
    for (auto corr : above_thresh)
    {
      out.append(corr.toString() + "\n");
    }
    return out;
  }
};
}  // namespace rct_optimizations
