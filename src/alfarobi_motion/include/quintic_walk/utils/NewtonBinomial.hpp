#ifndef LEPH_NEWTONBINOMIAL_HPP
#define LEPH_NEWTONBINOMIAL_HPP

#include "Combination.hpp"
#include "Polynom.hpp"

namespace robotis_op
{
/**
 * NewtonBinomial
 *
 * Implement Newton binomial
 * simple formulae and binding
 * with polynom structure
 */
class NewtonBinomial
{
public:
  /**
   * Expand the given formula (x + y)^degree
   * and return the polynom in x whose coefficient
   * are computed using binomial coefficient
   */
  static Polynom expandPolynom(double y, unsigned int degree);

private:
};

}  // namespace Leph

#endif
