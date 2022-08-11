#include "SteerLib.h"
#include "RVO2D_Parameters.h"
#include "Obstacle.h"

#define USE_ACCLMESH 1


/// <summary>
/// Get the linear value between 2 points, clamped if above or below it
/// </summary>
/// <typeparam name="T"></typeparam>
/// <param name="x">Value to find the corresponding output value.</param>
/// <param name="lowerlimit">smaller (left) of the two points</param>
/// <param name="upperlimit">larger (right) of the two points</param>
/// <returns>corresponding map that x becomes between lowerlimit and upperlimit</returns>
template<typename T>
T clamp(T x, T x1, T x2, T y1, T y2);

/**
 * \relates 	Agent
 * \brief		Compare the distance between two points
 */
bool compareDist(std::pair<float, const SteerLib::AgentInterface *> a1,
			std::pair<float, const SteerLib::AgentInterface *> a2 );

/**
 * \relates		Agent
 * \brief		Interpolates between two x points and maps between y points
 */
float interpolation(float y_max, float y_min, float x_max, float x_min, float x);


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
