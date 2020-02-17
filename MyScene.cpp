#include "MyScene.h"
#include "MyGame.h"
#include "MyPerlin.h"
#include "MyDragonAgent.h"
#include "MyChickenAgent.h"
#include "MyZombieAgent.h"
#include "MyPhysicsEngine.h"

#include <fstream>

namespace GMUCS425
{

bool MyScene::create( std::istream & inputfile)
{
  using namespace std;

  struct readEntity // need a structure for holding entities
  {
      string filename;
      float y; //layer
      float scale;
      float orient;
      bool agent;
  };

  MyTextureManager * texture_manager=getMyGame()->getTextureManager();
  MySpriteManager * sprite_manager=getMyGame()->getSpriteManager();

  //create physics engine
  this->physics=new MyPhysicsEngine();
  assert(this->physics);

  //build the level from the input stream
  int x,y;
  inputfile >> x >> y;	// read in the dimensions of the grid
  this->m_width=x;
  this->m_height=y;

  //resize window to get aspect ratio right.
  getMyGame()->resizeWidow(x,y);
  int screen_width = getMyGame()->getScreenWidth();
  int screen_height = getMyGame()->getScreenHeight();

  //start to create backdrop agent
  string backdropName;
  float cell_w=screen_width/x;
  float cell_h=screen_height/y;

  //for persisten covering parameters
  int pcover_grid_width=10, pcover_grid_height=10;
  string pcover_method;
  float battery=0, charging_time=0, latency=0;
  int seed=0;

  MyTexture * backdrop_texture=NULL;

  while(true)
  {
    inputfile >> backdropName;	// read in the backdrop image name
    if(backdropName=="Objects")
    {
      break;
    }
    else if(backdropName=="perlin")
    {
      //parameters needed for creating perlin noise
      string interpolate="linear", action="done";
      int n=1; float persistence=0.5;
      int perlin_w=2, perlin_h=2;
      inputfile >> perlin_w>> perlin_h >> interpolate >> n >> persistence ;

      //create interpolator
      MyInterpolator * interpolator = NULL;
      if(interpolate=="linear") interpolator=new MyLinearInterpolator();
      else if(interpolate=="cubic") interpolator=new MyCubicInterpolator();
      else if(interpolate=="cosine") interpolator=new MyCosineInterpolator();
      assert(interpolator);

      //create backdrop_texture from perlin noise
      Uint32 * pixels = new Uint32[sizeof(Uint32)*screen_width*screen_height];
      assert(pixels);
      MyPerlin perlin(perlin_w,perlin_h);
      if(n<2) perlin.generate(pixels, screen_width, screen_height, interpolator);
      else perlin.generate(pixels, screen_width, screen_height, interpolator, persistence, n);

      //
      inputfile >> action;
      if(action!="blend")//draw pixels directly
      {
        backdrop_texture=new MyTexture();
        assert(backdrop_texture);
        backdrop_texture->loadFromBuffer(pixels, screen_width, screen_height);
      }
      else //blend the two textures
      {
        string texture_file_1, texture_file_2;
        inputfile >> texture_file_1>>texture_file_2;
        // create backdrop texture
        if(!texture_manager->create(texture_file_1,true))
        {
          cerr<<"ERROR: Failed to create texture "<<texture_file_1<<endl;
          return false;
        }
        if(!texture_manager->create(texture_file_2,true))
        {
          cerr<<"ERROR: Failed to create texture "<<texture_file_2<<endl;
          return false;
        }
        MyTexture * t1=texture_manager->get(texture_file_1);
        MyTexture * t2=texture_manager->get(texture_file_2);
        if(t1->getWidth()!=t2->getWidth() || t1->getHeight()!=t2->getHeight())
        {
          cerr<<"ERROR: Textures "<<texture_file_2<<" and "<<texture_file_2<<" have different sizes"<<endl;
          return false;
        }
        if(t1->getWidth()>screen_width || t1->getHeight()>screen_height)
        {
          cerr<<"ERROR: Textures have larger size than the mask (perlin noise)"<<endl;
          return false;
        }

        backdrop_texture=t1->blend(t2, pixels);
      }

      this->m_terrain = pixels; //remember it as the terrain
    }
    else if(backdropName=="seed")
    {
      inputfile >>seed;
      std::srand(seed);
      srand48(seed);
    }
    else if(backdropName=="method")
    {
      inputfile >>pcover_method;
    }
    else if(backdropName=="regions")
    {
      inputfile >>pcover_grid_width>>pcover_grid_height;
    }
    else if(backdropName=="battery")
    {
      inputfile >>battery;
    }
    else if(backdropName=="charging")
    {
      inputfile >>charging_time;
    }
    else if(backdropName=="latency")
    {
      inputfile >>latency;
    }
    else //create backdrop from file
    {
      // create backdrop texture
      if(!texture_manager->create(backdropName))
      {
        cerr<<"ERROR: Failed to create texture "<<backdropName<<endl;
        return false;
      }
      backdrop_texture=texture_manager->get(backdropName);
    }//end create backdrop from file
  }

  //create backdrop sprite
  if(!sprite_manager->create("backdrop", backdrop_texture))
  {
    cerr<<"ERROR: Failed to create backdrop sprite"<<endl;
    return false;
  }

  m_backdrop=new MyAgent(false,false);
  assert(m_backdrop);
  m_backdrop->setSprite(sprite_manager->get("backdrop"));

  string buf;
  // inputfile >> buf;	// Start looking for the Objects section
  // //while  (buf != "Objects") inputfile >> buf;
  //
  // if (buf != "Objects")	// Oops, the file must not be formated correctly
  // {
  //   cerr << "ERROR: Level file error" << endl;
  //   return false;
  // }

  // read in the objects
  readEntity *rent = NULL;	// hold info for one object
  unordered_map<string,readEntity*> objs;		// hold all object and agent types;

  // read through all objects until you find the Characters section
  // these are the statinary objects
  while (!inputfile.eof() && buf != "Characters")
  {
    inputfile >> buf;			// read in the char
    if (buf != "Characters")
    {
      rent = new readEntity();	// create a new instance to store the next object

      // read the rest of the line
      inputfile >> rent->filename >> rent->orient >> rent->scale;
      rent->agent = false;		// these are objects
      objs[buf] = rent;		  	// store this object in the map
      if(!texture_manager->create(rent->filename))
      {
        cerr<<"ERROR: Failed to create "<<buf<<endl;
        return false;
      }
      //create the sprite
      if(!sprite_manager->create(buf, texture_manager->get(rent->filename)))
      {
        cerr<<"ERROR: Failed to create "<<buf<<endl;
        return false;
      }
    }
  }

  // Read in the characters (movable agents)
  while  (buf != "Characters") inputfile >> buf;	// get through any junk
  while (!inputfile.eof() && buf != "World") // Read through until the world section
  {
    inputfile >> buf;		// read in the char
    if (buf != "World")
    {
      rent = new readEntity();	// create a new instance to store the next object
      inputfile >> rent->filename >> rent->scale; // read the rest of the line
      rent->agent = true;			// this is an agent
      objs[buf] = rent;			// store the agent in the map

      if(buf[0]=='k') //for chicken only
      {
        //read chicken boid parameters
        //inputfile>>rent->battery>>rent->charging_time;
      }

      if(!texture_manager->create(rent->filename))
      {
        cerr<<"ERROR: Failed to create "<<buf<<endl;
        return false;
      }
      //create the sprite
      if(!sprite_manager->create(buf, texture_manager->get(rent->filename)))
      {
        cerr<<"ERROR: Failed to create "<<buf<<endl;
        return false;
      }
    }
  }

  // read through the placement map
  char c;
  for (int i = 0; i < y; i++)			// down (row)
  {
    for (int j = 0; j < x; j++)		// across (column)
    {
      inputfile >> c;			// read one char at a time
      buf = c + '\0';			// convert char to string
      rent = objs[buf];		// find cooresponding object or agent
      if (rent != NULL)		// it might not be an agent or object
      {
        //TODO: You will need to create your own agent here
        //based on the type of charactor
        MyAgent * agent=NULL;

        if(c=='z') agent = new MyZombieAgent(true);
        else if(c=='k'){
          agent = new MyChickenAgent(battery,charging_time);
        }
        else if(c=='d')
          agent = new MyDragonAgent(pcover_method, battery,charging_time,latency,pcover_grid_width,pcover_grid_height);
        else agent = new MyAgent(rent->agent);

        assert(agent);
        MySprite * sprite=sprite_manager->get(buf);
        assert(sprite);
        agent->setSprite(sprite);
        agent->rotateTo(rent->orient);
        agent->tranlateTo(j*cell_w,i*cell_h);
        agent->scaleTo(rent->scale);

        if(c=='z' || c=='k' || c=='d')
        {
          this->m_agents.push_front(agent); //remember
          //simulate the motion of chicken as particles
          //if(c=='k') this->physics->add_particle((MyChickenAgent*)agent);
        }
        else this->m_agents.push_back(agent); //remember
      }
      else // rent==null, not an object or agent
      {
        if(c!='o')
          cerr<<"WARNING: Unknow tag: "<<c<<". Ignore."<<endl;
      }
    } //end for j (col)
  }//end for i (row)

  // delete all of the readEntities in the objs map
  rent = objs["s"]; // just so we can see what is going on in memory (delete this later)
  for (auto obj : objs) // iterate through the objs
  {
    delete obj.second; // delete each readEntity
  }
  objs.clear(); // calls their destructors if there are any. (not good enough)

  //seed again...
  std::srand(seed);
  srand48(seed);
  
  //done!
  return true;
}


//handle a given event
void MyScene::handle_event(SDL_Event & e)
{
  for(MyAgent * agent : this->m_agents)
  {
    agent->handle_event(e);
  }
}

//update the scene
void MyScene::update()
{
  this->broad_range_collision();
  this->physics->step(0.05);

  for(MyAgent * agent : this->m_agents)
  {
    agent->update();
  }
}

//display the scene
void MyScene::display()
{
  //draw backgrop if any
  if(m_backdrop!=NULL) m_backdrop->display();
  //draw all agents
  for(MyAgent * agent : this->m_agents)
  {
    agent->display();
  }
}

//show HUD (heads-up display) or status bar
void MyScene::draw_HUD()
{
  for(MyAgent * agent : this->m_agents)
  {
    agent->draw_HUD();
  }
}

//detect collisions in board range
int MyScene::broad_range_collision()
{
  //brute force collision detection
  for(auto agent1 = m_agents.begin();agent1!=m_agents.end();agent1++)
  {
    if( (*agent1)->is_movable()==false) continue;
    auto agent2=agent1; agent2++;
    for(;agent2!=m_agents.end();agent2++)
    {
      //if( (*agent1)->is_movable()==false && (*agent2)->is_movable()==false) continue;
      if( (*agent1)->collide(*agent2))
      {
        SDL_Event event;
        event.type = SDL_USEREVENT;
        event.user.code = 1; //1 for collision
        event.user.data1 = *agent1;
        event.user.data2 = *agent2;
        SDL_PushEvent(&event);
      }
    }
  }
  return 0;
}


//create a texture from file
bool MySceneManager::create(std::string name, std::string scene_file)
{
  std::ifstream inputfile;		// Holds a pointer into the file
//   std::string path = __FILE__; //gets the current cpp file's path with the cpp file
// #if WIN32
//   path = path.substr(0,1+path.find_last_of('\\')); //removes filename to leave path
// #else
//   path = path.substr(0,1+path.find_last_of('/')); //removes filename to leave path
// #endif
//
//   path+= scene_file; //if txt file is in the same directory as cpp file
//   inputfile.open(path);
  inputfile.open(scene_file);

  if (!inputfile.is_open()) // oops. there was a problem opening the file
  {
    std::cerr << "ERROR: FILE COULD NOT BE OPENED" << std::endl;	// Hmm. No output?
    return false;
  }

  MyScene * level=new MyScene();
  assert(level);
  if(!level->create(inputfile)) return false;
  this->add(name,level);
  return true;
}

//get the first active scene
MyScene * MySceneManager::get_active_scene()
{
  for(auto & m :  this->m_Factory)
  {
    MyScene * s = (MyScene *)m.second;
    if(s->is_active()) return s;
  }

  return NULL; //nothing found, all inactive?
}

}//end namespace
