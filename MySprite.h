#pragma once

#include "MyTexture.h"

#include <vector>

namespace GMUCS425
{

	//this is a sprite (either stationary or dynamic sprite)
	class MySprite
	{
	public:

		MySprite(std::string name)
		{
			m_name=name;
			m_current_frame=0;
			m_sprite_sheet_texture=NULL;
		}

		//create sprite clips using a given Texture
		//specify the number of rows and cols of sprite clips in this texture
		bool create(MyTexture * texture, int row=1, int col=1);

		//displace this sprite at location (x,y) with orientation "angle"
		//rotated around the point "center"
		void display(int x, int y, float scale=1, float angle=0,
			           SDL_Point* center=NULL, SDL_RendererFlip flip=SDL_FLIP_NONE);

		void increment_frame();

		float getWidth(float scale=1) const { return scale * m_sprite_sheet_texture->getWidth(); }
		float getHeight(float scale=1) const { return scale * m_sprite_sheet_texture->getHeight(); }

	private:

		std::string m_name;
		int m_current_frame;
		std::vector<SDL_Rect> m_sprite_clips;
		MyTexture * m_sprite_sheet_texture;
	};

	class MySpriteManager : public MyManager<MySprite>
	{
	public:

		MySpriteManager()
		{

		}

		//create a texture from file
		bool create(std::string name, MyTexture * texture, int row=1, int col=1 )
		{
			MySprite * sprite=new MySprite(name);
			assert(sprite);
			if(!sprite->create(texture,row,col)) return false;
			this->add(name, sprite);
			return true;
		}

	};//end of class MyTextureManager

}//end namespace GMUCS425
