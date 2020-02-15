#pragma once

#include "MyTexture.h"
#include "mathtool/Vector.h"
#include <vector>

namespace GMUCS425
{

class MyInterpolator
{
public:
	//interpolate between p and q at s
	virtual float interpolate(float p, float q, float s)=0;
};

//creating perlin noise
class MyPerlin
{
	public:

		MyPerlin(int width, int height)
		{

			//Initialize
			using namespace mathtool;
			//srand48(time(NULL));
			mGradients=std::vector< std::vector<Vector2d> >(height, std::vector<mathtool::Vector2d>(width, Vector2d()) );
			for(auto & row : mGradients)
				for(auto& v : row)
					//v=Vector2d(drand48(),drand48()).normalize();
					v=Vector2d((drand48()>0.5)?1:-1,(drand48()>0.5)?1:-1).normalize();
			//
			mWidth = width;
			mHeight = height;
		}

		//TODO:
		//
		//generate perlin noise and store results in pixels
		//the values should be between 0 and 255
		void generate(Uint32 * pixels, int width, int height, MyInterpolator * interpolator);


		//TODO:
		//
		//generate n copies of perlin noises controlled using the given persistence
		//and store averaged results in pixels, the values should be between 0 and 255
		//Recall that: frequency = 2^i and amplitude = persistence^i
		//where i = 0 ~ n-1
		//
		void generate(Uint32 * pixels, int width, int height, MyInterpolator * interpolator, float persistence, int n);

		//Gets gradient dimensions
		int getWidth();
		int getHeight();

	private:

		float noise(int h, int w, float CW, float CH, MyInterpolator * interpolator);

		//The actual hardware texture
		std::vector< std::vector<mathtool::Vector2d> > mGradients;

		//Image dimensions
		int mWidth;
		int mHeight;

}; //end of class MyPerlin

class MyLinearInterpolator : public MyInterpolator
{
public:
	//interpolate between p and q at s using linear interpolation
	virtual float interpolate(float p, float q, float s)
	{
		//Implement This
		return p+s*(q-p);
	}
};

class MyCosineInterpolator : public MyInterpolator
{
public:
	//interpolate between p and q at s using cosine interpolation
	virtual float interpolate(float p, float q, float s)
	{
		//Implement This
		s= (1-cos(s*PI)) * 0.5;
		return p+s*(q-p);
	}
};


class MyCubicInterpolator : public MyInterpolator
{
public:
	//interpolate between p and q at s using cubic interpolation
	virtual float interpolate(float p, float q, float s)
	{
		//Implement This
		s=3*s*s-2*s*s*s;
		return p+s*(q-p);
	}
};

}//end namespace GMUCS425
