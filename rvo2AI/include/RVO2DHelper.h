#include "SteerLib.h"
#include "RVO2D_Parameters.h"
#include "Obstacle.h"

#define USE_ACCLMESH 1



/**
 * \relates    Agent
 * \brief      Solves a one-dimensional linear program on a specified line
 *             subject to linear constraints defined by lines and a circular
 *             constraint.
 * \param      lines         Lines defining the linear constraints.
 * \param      lineNo        The specified line constraint.
 * \param      radius        The radius of the circular constraint.
 * \param      optVelocity   The optimization velocity.
 * \param      directionOpt  True if the direction should be optimized.
 * \param      result        A reference to the result of the linear program.
 * \return     True if successful.
 */
bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
					float radius, const Util::Vector &optVelocity,
					bool directionOpt, Util::Vector &result);

/**
 * \relates    Agent
 * \brief      Solves a two-dimensional linear program subject to linear
 *             constraints defined by lines and a circular constraint.
 * \param      lines         Lines defining the linear constraints.
 * \param      radius        The radius of the circular constraint.
 * \param      optVelocity   The optimization velocity.
 * \param      directionOpt  True if the direction should be optimized.
 * \param      result        A reference to the result of the linear program.
 * \return     The number of the line it fails on, and the number of lines if successful.
 */
size_t linearProgram2(const std::vector<Line> &lines, float radius,
					  const Util::Vector &optVelocity, bool directionOpt,
					  Util::Vector &result);

/**
 * \relates    Agent
 * \brief      Solves a two-dimensional linear program subject to linear
 *             constraints defined by lines and a circular constraint.
 * \param      lines         Lines defining the linear constraints.
 * \param      numObstLines  Count of obstacle lines.
 * \param      beginLine     The line on which the 2-d linear program failed.
 * \param      radius        The radius of the circular constraint.
 * \param      result        A reference to the result of the linear program.
 */
void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
					float radius, Util::Vector &result);
