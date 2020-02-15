#include "MyGame.h"

namespace GMUCS425
{
	//game singleton
	MyGame * gGame=NULL;
	MyGame * getMyGame(){ return gGame; }
	void setMyGame(MyGame * game){ gGame=game; }

	//define the game prototype
	bool MyGame::init(const std::string name)
	{
		using namespace std;

		//Initialization flag
		bool success = true;

		//Initialize SDL
		if( SDL_Init( SDL_INIT_EVERYTHING ) < 0 )
		{
			cerr<<"ERROR: SDL could not initialize! SDL Error: "<<SDL_GetError()<<endl;
			success = false;
		}
		else
		{
			//Set texture filtering to linear
			if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) )
			{
				cerr<<"WARNING: Linear texture filtering not enabled!" <<endl;
			}

			//Create window
			gWindow = SDL_CreateWindow( name.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
			if( gWindow == NULL )
			{
				cerr<<"ERROR: Window could not be created! SDL Error: "<<SDL_GetError()<<endl;
				success = false;
			}
			else
			{
				//Create vsynced renderer for window
				gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC );
				if( gRenderer == NULL )
				{
					cerr<<"ERROR: Renderer could not be created! SDL Error: "<< SDL_GetError() <<endl;
					success = false;
				}
				else
				{
					//Initialize renderer color
					SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );

					//Initialize PNG loading
					int imgFlags = IMG_INIT_PNG;
					if( !( IMG_Init( imgFlags ) & imgFlags ) )
					{
						cerr<<"ERROR: SDL_image could not initialize! SDL_image Error: "<<IMG_GetError()<<endl;
						success = false;
					}

					//initialize font
					if( TTF_Init() == -1 )
					{
						cerr<<"ERROR: SDL_ttf could not initialize! SDL_ttf Error:  "<<TTF_GetError()<<endl;
						success = false;
					}
				}
			}
		}

		//reset timer
		g_timer=0;

		return success;
	}

	bool MyGame::loadMedia(const std::string & config)
	{
		//create only one level, in the future, we will use config file
		//to create multiple levels
	  if( !this->m_SceneManager.create("level01", config) ) return false;
		return true;
	}

	void MyGame::resizeWidow(int w, int h)
	{
		if(w>h)
	    SCREEN_HEIGHT=(int)(SCREEN_WIDTH*h*1.0f/w);
	  else
	    SCREEN_WIDTH=(int)(SCREEN_HEIGHT*w*1.0f/h);
	  SDL_SetWindowSize(getMyGame()->getWindow(), SCREEN_WIDTH, SCREEN_HEIGHT);
	}

	void MyGame::run()
	{
		//Main loop flag
		bool quit = false;

		//Event handler
		SDL_Event e;

		//Current animation frame
		unsigned int frame = 0;

		std::list<MyScene *> allScenes;
		m_SceneManager.getAll(allScenes);

		//While application is running
		const Uint8 *keys = SDL_GetKeyboardState(NULL);
		while( !quit )
		{
			//Handle events on queue
			while( SDL_PollEvent( &e ) != 0 )
			{
				//User requests quit
				if( e.type == SDL_QUIT || keys[SDL_SCANCODE_ESCAPE])
				{
					quit = true;
				}
				else
				{
					for(MyScene * scene : allScenes)
						if(scene->is_active()) scene->handle_event(e);
				}
			}

			//std::cout<<"---------- UPDATE ---------- "<<std::endl;

			for(MyScene * scene : allScenes) if(scene->is_active()) scene->update();

			//Clear screen
			SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
			SDL_RenderClear( gRenderer );

			//Render current frame
			for(MyScene * scene : allScenes) if(scene->is_active()) scene->display();
			for(MyScene * scene : allScenes) if(scene->is_active()) scene->draw_HUD();

			//Update screen
			SDL_RenderPresent( gRenderer );

			//sleep for a short amount of time
			SDL_Delay(g_step);
			this->g_timer+=g_step;//in millisecond

			//Go to next frame
			++frame;
		}
	}


	void MyGame::close()
	{
		//Destroy window
		SDL_DestroyRenderer( gRenderer );
		SDL_DestroyWindow( gWindow );
		gWindow = NULL;
		gRenderer = NULL;

		//Quit SDL subsystems
		IMG_Quit();
		SDL_Quit();
	}

}//end namespace GMUCS425
