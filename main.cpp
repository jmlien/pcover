
#include "MyGame.h"

int main( int argc, char* argv[] )
{

	if(argc!=2){
		std::cerr<<"Usage: "<<argv[0]<<" *.txt"<<std::endl;
		return 1;
	}

	MASC_PCOVER::MyGame * game = new MASC_PCOVER::MyGame(512,512);
	assert(game);
	MASC_PCOVER::setMyGame(game);

	//Start up SDL and create window
	if( !game->init("Persisten Covering with Energy & Latency") )
	{
		std::cerr<<"ERROR: Failed to initialize!"<<std::endl;
	}
	else
	{
		//Load media
		if( !game->loadMedia(argv[1]) )
		{
			std::cerr<<"ERROR: Failed to load media!"<<std::endl;
		}
		else
		{
			game->run();
		}
	}

	game->close();

	delete game;

	return 0;
}
