#pragma once

#include <iostream>
#include <fstream>

#include "PixelBuffer.h"

const int samples_per_pixel = 60;

namespace IO
{
	void write_as_PPM(const PixelBuffer& pixel_buffer, std::ostream& output)
	{
		output << "P3\n" << pixel_buffer.dimensions.x << ' ' << pixel_buffer.dimensions.y << "\n255\n";

		const int total_pixels = pixel_buffer.dimensions.x * pixel_buffer.dimensions.y;
		for (int i = 0; i < total_pixels; ++i)
		{
			auto v = pixel_buffer.get(i);

            auto r = v.r;
            auto g = v.g;
            auto b = v.b;

            // Divide the color by the number of samples and gamma-correct for gamma=2.0.
            auto scale = 1.0 / samples_per_pixel;
            r = sqrt(scale * r);
            g = sqrt(scale * g);
            b = sqrt(scale * b);
			//gamma-correct for gamma = 2 (raising power by 1/gamma)

			int ir = static_cast<int>(256*glm::clamp(sqrt(v.r),0.0,0.999));
			int ig = static_cast<int>(256*glm::clamp(sqrt(v.g),0.0,0.999));
			int ib = static_cast<int>(256*glm::clamp(sqrt(v.b),0.0,0.999));

			output << ir << ' ' << ig << ' ' << ib << '\n';
		}
	}
}
