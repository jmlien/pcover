#include "MySprite.h"
#include "MyGame.h"

namespace MASC_PCOVER
{

	//create sprite clips using a given Texture
	//specify the number of rows and cols of sprite clips in this texture
	bool MySprite::create(MyTexture * texture, int row, int col)
	{
		if(texture==NULL) return false;
		int W=texture->getWidth();
		int H=texture->getHeight();
		int w=W/col;
		int h=H/row;
		if(W<=0 || H<=0 || w<=0 || h<=0) return false;

		this->m_sprite_clips.reserve(row*col);

		SDL_Rect clip;
		for(int r=0;r<row;r++)
		{
			for(int c=0;c<col;c++)
			{
				clip.x = c*w;
				clip.y = r*h;
				clip.w = w;
				clip.h = h;
				this->m_sprite_clips.push_back(clip);
			}
		}

		this->m_sprite_sheet_texture=texture;

		return true;
	}

	//render the sprite
	void MySprite::display(int x, int y, float scale, float angle, SDL_Point* center, SDL_RendererFlip flip)
	{
		if(m_sprite_sheet_texture==NULL)
		{
				std::cerr<<"ERROR: No sheet texture. Make sure to initialize the sprite."<<std::endl;
				return;
		}

		SDL_Rect& clip = m_sprite_clips[m_current_frame];
		m_sprite_sheet_texture->render(x, y, &clip, scale, angle, center, flip);
	}

	//increment the current frame
	void MySprite::increment_frame()
	{
		if(m_sprite_sheet_texture==NULL)
		{
				std::cerr<<"ERROR: No sheet texture. Make sure to initialize the sprite."<<std::endl;
				return;
		}

		m_current_frame++;
		if(m_current_frame>=m_sprite_clips.size()) m_current_frame-=m_sprite_clips.size();
	}

}//end namespace MASC_PCOVER
