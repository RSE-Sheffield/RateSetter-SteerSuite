#include "RVO2DHelper.h"
#include "RVO2DAIModule.h"
#include "SteerLib.h"
#include "Definitions.h"
#include "RVO2D_Parameters.h"

template<typename T>
T clamp(T x, T x1, T x2, T y1, T y2) {
	T y;
	if (x < x1)
		y = y1;
	else if (x > x2)
		y = y2;
	else {
		y = (x - x1) / (x2 - x1) * (y2 - y1) + y1;
	}
	return y;
}


bool compareDist(std::pair<float, const SteerLib::AgentInterface *> a1,
			std::pair<float, const SteerLib::AgentInterface *> a2 )
{
    return a1.first < a2.first;
}

//interpolated the value x to a vlaue betweeen y_max and y_min depending on the value of x compared to x_max and x_min 
float interpolation(float y_max, float y_min, float x_max, float x_min, float x)
{
	if (x_max == x_min) {
		return y_max;
	}
	if (x > x_max) {
		return y_max;
	}
	else if (x < x_min) {
		return y_min;
	}
	else {
		return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min;
	}
}


bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Util::Vector &optVelocity, bool directionOpt, Util::Vector &result)
{
	const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
	const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);

	if (discriminant < 0.0f) {
		/* Max speed circle fully invalidates line lineNo. */
		return false;
	}

	const float sqrtDiscriminant = std::sqrt(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;

	for (size_t i = 0; i < lineNo; ++i) {
		const float denominator = det(lines[lineNo].direction, lines[i].direction);
		const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

		if (std::fabs(denominator) <= RVO_EPSILON) {
			/* Lines lineNo and i are (almost) parallel. */
			if (numerator < 0.0f) {
				return false;
			}
			else {
				continue;
			}
		}

		const float t = numerator / denominator;

		if (denominator >= 0.0f) {
			/* Line i bounds line lineNo on the right. */
			tRight = std::min(tRight, t);
		}
		else {
			/* Line i bounds line lineNo on the left. */
			tLeft = std::max(tLeft, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (optVelocity * lines[lineNo].direction > 0.0f) {
			/* Take right extreme. */
			result = lines[lineNo].point + tRight * lines[lineNo].direction;
		}
		else {
			/* Take left extreme. */
			result = lines[lineNo].point + tLeft * lines[lineNo].direction;
		}
	}
	else {
		/* Optimize closest point. */
		const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

		if (t < tLeft) {
			result = lines[lineNo].point + tLeft * lines[lineNo].direction;
		}
		else if (t > tRight) {
			result = lines[lineNo].point + tRight * lines[lineNo].direction;
		}
		else {
			result = lines[lineNo].point + t * lines[lineNo].direction;
		}
	}

	return true;
}

size_t linearProgram2(const std::vector<Line> &lines, float radius, const Util::Vector &optVelocity, bool directionOpt, Util::Vector &result)
{
	if (directionOpt) {
		/*
		 * Optimize direction. Note that the optimization velocity is of unit
		 * length in this case.
		 */
		result = optVelocity * radius;
	}
	else if (absSq(optVelocity) > sqr(radius)) {
		/* Optimize closest point and outside circle. */
		result = normalize(optVelocity) * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (size_t i = 0; i < lines.size(); ++i) {
		if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			const Util::Vector tempResult = result;

			if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return lines.size();
}

void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Util::Vector &result)
{
	float distance = 0.0f;

	for (size_t i = beginLine; i < lines.size(); ++i) {
		if (det(lines[i].direction, lines[i].point - result) > distance) {
			/* Result does not satisfy constraint of line i. */
			std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

			for (size_t j = numObstLines; j < i; ++j) {
				Line line;

				float determinant = det(lines[i].direction, lines[j].direction);

				if (std::fabs(determinant) <= RVO_EPSILON) {
					/* Line i and line j are parallel. */
					if (lines[i].direction * lines[j].direction > 0.0f) {
						/* Line i and line j point in the same direction. */
						continue;
					}
					else {
						/* Line i and line j point in opposite direction. */
						line.point = 0.5f * (lines[i].point + lines[j].point);
					}
				}
				else {
					line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
				}

				line.direction = normalize(lines[j].direction - lines[i].direction);
				projLines.push_back(line);
			}

			const Util::Vector tempResult = result;

			if (linearProgram2(projLines, radius, Util::Vector(-lines[i].direction.z, 0.0, lines[i].direction.x), true, result) < projLines.size()) {
				/* This should in principle not happen.  The result is by definition
				 * already in the feasible region of this linear program. If it fails,
				 * it is due to small floating point error, and the current result is
				 * kept.
				 */
				result = tempResult;
			}

			distance = det(lines[i].direction, lines[i].point - result);
		}
	}
}

