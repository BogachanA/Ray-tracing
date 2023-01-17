#pragma once

#include "glm/glm.hpp"

struct PixelBuffer
{
	const glm::ivec2 dimensions;
	const double ratio;
	glm::dvec3 * const pixels;

	PixelBuffer(const glm::ivec2& dimensions) :
		dimensions(dimensions),
		ratio(dimensions.y / dimensions.x),
		pixels(new glm::dvec3[dimensions.x * dimensions.y])
	{}

	~PixelBuffer()
	{
		delete[] pixels;
	}

	const glm::dvec3& get(unsigned i) const
	{
		return pixels[i];
	}

	const glm::dvec3& get(unsigned x, unsigned y) const
	{
		return pixels[y * dimensions.x + x];
	}

	void set(unsigned x, unsigned y, const glm::dvec3& value) const
	{
		pixels[y * dimensions.x + x] = value;
	}
};