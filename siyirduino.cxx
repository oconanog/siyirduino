/* 
   SIYIRDUiNO story line
   ---------------------
      A secret laboratory in Eceabat, Canakkale... Scientists realized that the
   improved dodging skills can cause extraordinary leaps in human intellectual
   evolution. They knew that "reading" is much less than "understanding" itself
   therefore, they have developed this game instead of writing a paper.
     They welcome you to join the collective effort of contributing to this
   evolution by playing the game, getting addicted and adding new features to it.

   Artwork:
     Onur Cobanoglu, olmectheholy@gmail.com (original idea, images and sound effects)
     Tanya A. Baser, tanya.baser@gmail.com  (S.E.M. backgrounds)
     Hayko Cepkin,   www.haykocepkin.com    (game musics)
   
   C/C++ programing and arduino firmware:
     Ozgur Cobanoglu, Ozgur.Cobanoglu@cern.ch (SDL implementation)
*/

#include<SDL/SDL.h>
#include<SDL/SDL_image.h>
#include<SDL/SDL_rotozoom.h>
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<string>
#include<SDL/SDL_ttf.h>
#include<math.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>

#define SCREEN_WIDTH	800
#define SCREEN_HEIGHT	500

#define PLAYER_WIDTH	46
#define PLAYER_HEIGHT	32
#define PLAYER_MAX_VELOCITY	  (2.0)
#define PLAYER_MIN_VELOCITY	  (-PLAYER_MAX_VELOCITY)
#define PLAYER_FORWARD_THRUST (-0.1)
#define PLAYER_REVERSE_THRUST (-PLAYER_FORWARD_THRUST)

#define RAKI_MAX_VELOCITY	  (1)
#define RAKI_MIN_VELOCITY	  (-RAKI_MAX_VELOCITY)

#define BMB_MAX_VELOCITY	  (1)
#define BMB_MIN_VELOCITY	  (-BMB_MAX_VELOCITY)

#define NME_MAX_VELOCITY	 (1.5)
#define NME_MIN_VELOCITY	 (-NME_MAX_VELOCITY)
#define NME_FORWARD_THRUST (-0.05)
#define NME_REVERSE_THRUST (-NME_FORWARD_THRUST)

#define MAX_PARTICLES	10000000
#define PI (3.141592654F)
#define VOLUME_PER_SOUND  SDL_MIX_MAXVOLUME
#define MAX_PLAYING_SOUND 20
#define MAX_NUMBER_OF_ENEMIES 1000
#define MAX_NUMBER_OF_BOMBS 500
#define MAX_NUMBER_OF_RAKI 10
#define VELOCITY_INCREMENT 0.05
#define DEATH_FIELD 150

bool paused, helped, isGameOver, isWon, contClear, controlsInverted;
bool isAccel = false;
int geriSayac, scaler = 6;
int fd = 0;
char arduinoOutput[500];

/*
    SOUND Related Definitions ______________________________________
*/

typedef struct sound_s {
  Uint8 *samples; // raw PCM sample data
  Uint32 length;  // sound data size in bytes                     
}sound_t, *sound_p;

typedef struct playing_s {
  int active;      // 1 if this sound should be played
  sound_p sound;   // sound data to play
  Uint32 position; // current position in the sound buffer
} playing_t, *playing_p;

void AudioCallback(void *user_data, Uint8 *audio, int length);
int LoadAndConvertSound(char*filename, SDL_AudioSpec *spec, sound_p sound);
void ClearPlayingSounds();
int PlaySound(sound_p sound);
static void Check4Death();

static int counter = 0;
SDL_mutex *counter_mutex;
playing_t playing[MAX_PLAYING_SOUND];

/*
    PARTICLE Related Definitions ___________________________________
*/

typedef struct particle_s {
    double x,y;         /* coordinates of the particle */
    double energy;      /* velocity of the particle */
    double angle;       /* angle of the particle */
    int r, g, b;        /* color */
} particle_t, *particle_p;

particle_t particles[MAX_PARTICLES];
int active_particles = 0;

void DrawParticles(SDL_Surface *dest, 
                   int camera_x, 
                   int camera_y);
void UpdateParticles(void);
void CreateParticleExplosion(int x, int y, 
                             int r, int g, int b,
                             int energy, int density);
static void AddParticle(particle_p particle);
static void DeleteParticle(int index);
static Uint16 CreateHicolorPixel(SDL_PixelFormat * fmt, 
                                 Uint8 red,
				                         Uint8 green, 
                                 Uint8 blue);

// takes the string name of the serial port (e.g. "/dev/ttyACM0","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
//________________________________________________________________
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
    //        serialport,baud);

    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

//________________________________________________________________
int serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 10 * 1000 ); // wait 10 msec try again
            continue;
        }
        buf[i] = b[0]; i++;
    } while( b[0] != until );

    buf[i] = 0;  // null terminate the string
    return 0;
}

//________________________________________________________
void AudioCallback(void *user_data, Uint8 *audio, int length)
{
  memset(audio, 0, length);
  for (int i=0 ; i<MAX_PLAYING_SOUND ; i++) {
    if (playing[i].active) {
      Uint8 *sound_buf;
      Uint32 sound_len;
      sound_buf = playing[i].sound->samples;
      sound_buf += playing[i].position;
      if ((playing[i].position+length) > playing[i].sound->length) {
	      sound_len = playing[i].sound->length - playing[i].position;
      } else {
	      sound_len = length;
      }
      SDL_MixAudio(audio, sound_buf, sound_len, VOLUME_PER_SOUND);
      playing[i].position += length;
      if (playing[i].position >= playing[i].sound->length) {
	      playing[i].active = 0; // mark it inactive
      }
    }
  }
}

//________________________________________________________
int LoadAndConvertSound(char*filename, SDL_AudioSpec *spec, sound_p sound)
{
  SDL_AudioCVT cvt;
  SDL_AudioSpec loaded;
  Uint8 *new_buf;
  if (SDL_LoadWAV(filename, &loaded, &sound->samples, &sound->length ) == NULL) {
    printf("Unable to load sound : %s\n", SDL_GetError());
    return 1;
  }
  if (SDL_BuildAudioCVT(&cvt, loaded.format, loaded.channels, loaded.freq,
			spec->format, spec->channels, spec->freq) < 0) {
    printf("Unable to load sound : %s\n", SDL_GetError());
    return 1;
  }
  cvt.len = sound->length;
  new_buf = (Uint8*)malloc(cvt.len * cvt.len_mult);
  if (new_buf == NULL){
    printf("Memory allocation failed.\n");
    SDL_FreeWAV(sound->samples);
    return 1;
  }
  memcpy(new_buf, sound->samples, sound->length);
  cvt.buf = new_buf;
  if (SDL_ConvertAudio(&cvt)<0){
    printf("Audio conversion error : %s\n", SDL_GetError());
    free(new_buf);
    SDL_FreeWAV(sound->samples);
    return 1;
  }
  SDL_FreeWAV(sound->samples);
  sound->samples = new_buf;
  sound->length = sound->length * cvt.len_mult;
  return 0;
}

//________________________________________________________
void ClearPlayingSounds()
{
  for (int i=0 ; i<MAX_PLAYING_SOUND ; i++){
    playing[i].active = 0;
  }
}

//________________________________________________________
int PlaySound(sound_p sound)
{
    int i;
  for (i=0 ; i<MAX_PLAYING_SOUND ; i++ ){ // find free slot for sound
    if (playing[i].active == 0) break;
  }
  if (i == MAX_PLAYING_SOUND) return 1; // no free slot for this spound
  SDL_LockAudio();
  playing[i].active = 1;
  playing[i].sound = sound;
  playing[i].position = 0;
  SDL_UnlockAudio();
  return 0;
}

//________________________________________________________________
typedef struct player_s {
    int x, y;              // coordinates
    double velocity_x;     // velocity in pixels per frame
    double velocity_y;    
    double accel_x;        // acceleration in pixels/frame^2
    double accel_y;
    bool movable;
    bool dead;
    int angle;             // Current direction
} player_t, *player_p;

//________________________________________________________________
typedef struct nme_s {
    int x, y;              // coordinates
    double maxVel;         // maximum velocity
    double velocity_x;     // velocity in pixels per frame
    double velocity_y;    
    double accel_x;		     // acceleration in pixels/frame^2
    double accel_y;
    bool movable;
    bool dead;
    int angle;
} nme_t, *nme_p;

//________________________________________________________________
typedef struct bmb_s {
    int x, y;              // coordinates
    double velocity_x;     // velocity in pixels per frame
    double velocity_y;    
    double accel_x;		     // acceleration in pixels/frame^2
    double accel_y;
    bool movable;
    bool dead;
    int angle;
} bmb_t, *bmb_p;

//________________________________________________________________
typedef struct rak_s {
    int x, y;              // coordinates
    double velocity_x;     // velocity in pixels per frame
    double velocity_y;    
    double accel_x;		     // acceleration in pixels/frame^2
    double accel_y;
    bool movable;
    bool dead;
    int angle;
    bool rotDir;
    void toggleRotDir() {
      if (rotDir) 
        rotDir = false;
      else 
        rotDir = true;
    };
} rak_t, *rak_p;

static int exit_flag = 0;
double time_scale = 1.0;

player_t player;
nme_t nme[MAX_NUMBER_OF_ENEMIES];
bmb_t bmb[MAX_NUMBER_OF_BOMBS];
rak_t rak[MAX_NUMBER_OF_RAKI];
SDL_Surface *screen, *rock, *bomba, *raki, *background, *backgroundInv, *temp, *msg;
SDL_Surface *pmsg, *vmsg, *credits, *range, *help, *bir, *iki, *uc;
SDL_Surface *rotoTmp, *roto[360], *rotonme[360], *rotobomb[360], *rotoraki[360];
SDL_Rect src, dest, dest2;
SDL_AudioSpec desired, obtained;
sound_t explosion, yakin, pith, gameover, dur, kapat, oyunMuzigi, basariMuzigi;
TTF_Font *font = NULL;
TTF_Font *smallfont = NULL;
SDL_Color textColor = { 255, 255, 255 };
SDL_Color PtextColor = { 255, 255, 255 };
SDL_Event event;
int dusmanSayisi, bombaSayisi, oluSayisi, rakiSayisi;

static void DrawPlayer(player_p p);
static void UpdatePlayer(player_p p);
static void InitPlayer(player_p p);

static void DrawNme(nme_p p);
static void UpdateNme(nme_p p);
static void InitNme(nme_p p);

static void DrawBmb(nme_p p);
static void UpdateBmb(bmb_p p);
static void InitBmb(nme_p p);

static void DrawRak(rak_p p);
static void UpdateRak(rak_p p);
static void InitRak(rak_p p);

static void updateDisplay(int noBmb, int noNme);
static void freezeAll();
static int returnAngle(player_p p);

//________________________________________________________________
static void AddParticle(particle_p particle)
{
    /* If there are already too many particles, forget it. */
    
  if (active_particles >= MAX_PARTICLES) return;
  particles[active_particles] = *particle;
  active_particles++;
}

/* Removes a particle from the system (by index). */
//________________________________________________________________
static void DeleteParticle(int index)
{
    /* Replace the particle with the one at the end
       of the list, and shorten the list. */
    particles[index] = particles[active_particles - 1];
    active_particles--;
}

//________________________________________________________________
void DrawParticles(SDL_Surface * dest, int camera_x, int camera_y)
{
/* Draws all active particles on the screen. */

    int i;
    Uint16 *pixels;
    
      /* Lock the target surface for direct access. */
    
    if (SDL_LockSurface(dest) != 0) return;
    pixels = (Uint16 *) dest->pixels;
    
    for (i = 0; i < active_particles; i++) {
      int x, y;
	    Uint16 color;
	      /* Convert world coords to screen coords. */
	    x = (int)(particles[i].x - camera_x);
	    y = (int)(particles[i].y - camera_y);
	    if ((x < 0) || (x >= SCREEN_WIDTH)) continue;
	    if ((y < 0) || (y >= SCREEN_HEIGHT)) continue;
	      /* Find the color of this particle. */
	    color = CreateHicolorPixel( dest->format, particles[i].r, particles[i].g, particles[i].b);
	      /* Draw the particle. */
	    pixels[(dest->pitch / 2) * y + x] = color;
    }
    
    /* Release the screen. */
    
    SDL_UnlockSurface(dest);
}

//________________________________________________________________
void UpdateParticles(void)
{
/* Updates the position of each particle. Kills particles with
   zero energy. */
   
    int i;

    for (i = 0; i < active_particles; i++) {
	    particles[i].x += particles[i].energy *
	                      cos(particles[i].angle * PI / 180.0) * time_scale;
	                      particles[i].y += particles[i].energy *
	                      -sin(particles[i].angle * PI / 180.0) * time_scale;
        /* Fade the particle's color. */
	    particles[i].r-=3;
	    particles[i].g-=3;
    	particles[i].b-=3;
      if (particles[i].r < 0) particles[i].r = 0;
	    if (particles[i].g < 0) particles[i].g = 0;
    	if (particles[i].b < 0) particles[i].b = 0;
      	/* If the particle has faded to black, delete it. */
	    if ((particles[i].r + particles[i].g + particles[i].b) == 0) {
	      DeleteParticle(i);
	      /* DeleteParticle replaces the current particle with the one
	         at the end of the list, so we'll need to take a step back. */
	      i--;
	    }
    }
}

/* Creates a particle explosion of the given relative size and position. */
//________________________________________________________________
void CreateParticleExplosion(int x, int y, int r, int g, int b, int energy, int density)
{
    int i;
    particle_t particle;

    /* Create a number of particles proportional to the size of the explosion. */
    for (i = 0; i < density; i++) {
      particle.x = x;
	    particle.y = y;
	    particle.angle = (rand() % 360);
	    particle.energy = (double) (rand() % (energy * 1000)) / 1000.0;
	      /* Set the particle's color. */
	    particle.r = r;
	    particle.g = g;
	    particle.b = b;
	      /* Add the particle to the particle system. */
	    AddParticle(&particle);
    }
}

//________________________________________________________________
static Uint16 CreateHicolorPixel(SDL_PixelFormat * fmt, Uint8 red, Uint8 green, Uint8 blue)
{
/* This is directly from another code listing. It creates a 16-bit pixel. */

    Uint16 value;

    /* This series of bit shifts uses the information from the SDL_Format
       structure to correctly compose a 16-bit pixel value from 8-bit red,
       green, and blue data. */
    value = (((red >> fmt->Rloss) << fmt->Rshift) +
	          ((green >> fmt->Gloss) << fmt->Gshift) +
	          ((blue >> fmt->Bloss) << fmt->Bshift));
    return value;
}

//________________________________________________________________
static int returnAngle(player_p p)
{
    // Calculates the player direction out of velocity components
  int angle;
  double Vx = p->velocity_x;
  double Vy = p->velocity_y;  
  
  if (Vy == 0 && Vx == 0) angle = p->angle; // No velocity no change in angle
  
  if (Vx == 0) {  // Less calculation for obvious cases
    if (Vy<0) 
      return 270;
    else 
      return 90;
  } else if (Vy == 0) {
    if (Vx < 0)
      return 180;
    else 
      return 0;
  }
  
    // Angle is calculated out of velocnnity components
  angle = (int) ((360.0/(2.0*M_PI)) * asin( Vy / (sqrt(Vy*Vy+Vx*Vx)) ));
  
  if      (Vx>0 && Vy<0) angle  = 359+angle;  // This was a bit tricky :)
  else if (Vx<0)         angle  = 180-angle;
  
  if (angle < 360 && angle >= 0)  // Return only valid angles (always)
    return angle;
  else
    return -1;
}

//________________________________________________________________
static void InitPlayer(player_p p)
{
    p->x = (int) ((SCREEN_WIDTH) * (rand() / (RAND_MAX + 1.0)));
    p->y = (int) ((SCREEN_HEIGHT) * (rand() / (RAND_MAX + 1.0)));
    p->velocity_x = 0;
    p->velocity_y = 0;
    p->accel_x    = 0;
    p->accel_y    = 0;
    p->movable    = true;
    p->angle      = 0;
    UpdatePlayer(p);
}

//________________________________________________________________
static void InitNme(nme_p p)
{
    do {
      p->x = (int) ((SCREEN_WIDTH) * (rand() / (RAND_MAX + 1.0)));
      p->y = (int) ((SCREEN_HEIGHT) * (rand() / (RAND_MAX + 1.0)));
    } while (1.5*DEATH_FIELD >= sqrt(pow((player.x-p->x),2) + pow((player.y-p->y),2)));
    
    int minVel = NME_MAX_VELOCITY * 0.6;
    
    p->velocity_x = 0;
    p->velocity_y = 0;
    p->maxVel     = ((NME_MAX_VELOCITY-minVel) * (rand() / (RAND_MAX + 1.0))) + minVel;
    p->accel_x    = 0;
    p->accel_y    = 0;
    p->movable    = true;
    p->dead       = false;
    p->angle      = (int)(360 * (rand() / (RAND_MAX + 1.0)));
    dusmanSayisi++;
    UpdateNme(p);
}

//________________________________________________________________
static void InitBmb(bmb_p p)
{
    do {
      p->x = (int) ((SCREEN_WIDTH) * (rand() / (RAND_MAX + 1.0)));
      p->y = (int) ((SCREEN_HEIGHT) * (rand() / (RAND_MAX + 1.0)));
    } while (1.5*DEATH_FIELD >= sqrt(pow((player.x-p->x),2) + pow((player.y-p->y),2)));
    
    int minVel = BMB_MAX_VELOCITY * 0.8;
    
    p->velocity_x = ((BMB_MAX_VELOCITY-minVel) * (rand() / (RAND_MAX + 1.0))) + minVel;
    p->velocity_y = ((BMB_MAX_VELOCITY-minVel) * (rand() / (RAND_MAX + 1.0))) + minVel;
    p->accel_x    = 0;
    p->accel_y    = 0;
    p->movable    = true;
    p->dead       = false;
    p->angle      = (int)(360 * (rand() / (RAND_MAX + 1.0)));
    bombaSayisi++;
    UpdateBmb(p);
}

//________________________________________________________________
static void InitRak(rak_p p)
{
    do {
      p->x = (int) ((SCREEN_WIDTH) * (rand() / (RAND_MAX + 1.0)));
      p->y = (int) ((SCREEN_HEIGHT) * (rand() / (RAND_MAX + 1.0)));
    } while (1.5*DEATH_FIELD >= sqrt(pow((player.x-p->x),2) + pow((player.y-p->y),2)));
    
    int minVel = RAKI_MAX_VELOCITY * 0.8;
    
    p->velocity_x = ((RAKI_MAX_VELOCITY-minVel) * (rand() / (RAND_MAX + 1.0))) + minVel;
    p->velocity_y = ((RAKI_MAX_VELOCITY-minVel) * (rand() / (RAND_MAX + 1.0))) + minVel;
    p->accel_x    = 0;
    p->accel_y    = 0;
    p->movable    = true;
    p->dead       = false;
    p->angle      = 0;
    p->rotDir     = true; // clockwise
    rakiSayisi++;
    UpdateRak(p);
}

//________________________________________________________________
static void UpdatePlayer(player_p p)
{
    if (p->movable == false || p->dead == true) return;
    
    p->velocity_x += p->accel_x * time_scale;
    p->velocity_y += p->accel_y * time_scale;
    
    if (p->velocity_x > PLAYER_MAX_VELOCITY) p->velocity_x = PLAYER_MAX_VELOCITY;
    if (p->velocity_x < PLAYER_MIN_VELOCITY) p->velocity_x = PLAYER_MIN_VELOCITY;
    if (p->velocity_y > PLAYER_MAX_VELOCITY) p->velocity_y = PLAYER_MAX_VELOCITY;
    if (p->velocity_y < PLAYER_MIN_VELOCITY) p->velocity_y = PLAYER_MIN_VELOCITY;
	
    int angle = returnAngle(p);
    if (-1 != angle) p->angle = returnAngle(p);
    else printf("-1 returned\n");
  
    p->x += (int)(p->velocity_x * time_scale);
    p->y += (int)(p->velocity_y * time_scale);
				    
      // Reflect the player off
    if (p->x < 0 || p->x >= SCREEN_WIDTH) p->velocity_x = -p->velocity_x;
    if (p->y < 0 || p->y >= SCREEN_HEIGHT) p->velocity_y = -p->velocity_y;
    
      // Keep player inside the playground
    if (p->x < 0) p->x = 0;
    if (p->x >= SCREEN_WIDTH) p->x = SCREEN_WIDTH-1;
    if (p->y < 0) p->y = 0;
    if (p->y >= SCREEN_HEIGHT) p->y = SCREEN_HEIGHT-1;
}

//________________________________________________________________
static void UpdateNme(nme_p p)
{
    if (p->movable == false || p->dead == true) return;
    
    if (player.x < p->x) p->accel_x = NME_FORWARD_THRUST;
    else p->accel_x = NME_REVERSE_THRUST;
    if (player.y < p->y) p->accel_y = NME_FORWARD_THRUST;
    else p->accel_y = NME_REVERSE_THRUST;
    
    p->velocity_x += p->accel_x * time_scale;
    p->velocity_y += p->accel_y * time_scale;
    
    if (p->velocity_x >  p->maxVel) p->velocity_x =  p->maxVel;
    if (p->velocity_x < -p->maxVel) p->velocity_x = -p->maxVel;
    if (p->velocity_y >  p->maxVel) p->velocity_y =  p->maxVel;
    if (p->velocity_y < -p->maxVel) p->velocity_y = -p->maxVel;
    
    p->x += (int)(p->velocity_x * time_scale);
    p->y += (int)(p->velocity_y * time_scale);
    
    if (p->x < 0) p->x = 0;
    if (p->x >= SCREEN_WIDTH) p->x = SCREEN_WIDTH-1;
    if (p->y < 0) p->y = 0;
    if (p->y >= SCREEN_HEIGHT) p->y = SCREEN_HEIGHT-1;
}

//________________________________________________________________
static void UpdateBmb(bmb_p p)
{
    if (p->movable == false || p->dead == true) return;
    
    p->velocity_x += p->accel_x * time_scale;
    p->velocity_y += p->accel_y * time_scale;
    
    if (p->velocity_x > PLAYER_MAX_VELOCITY) p->velocity_x = PLAYER_MAX_VELOCITY;
    if (p->velocity_x < PLAYER_MIN_VELOCITY) p->velocity_x = PLAYER_MIN_VELOCITY;
    if (p->velocity_y > PLAYER_MAX_VELOCITY) p->velocity_y = PLAYER_MAX_VELOCITY;
    if (p->velocity_y < PLAYER_MIN_VELOCITY) p->velocity_y = PLAYER_MIN_VELOCITY;
	
    p->x += (int)(p->velocity_x * time_scale);
    p->y += (int)(p->velocity_y * time_scale);
    
      // Reflect the bombs
    if (p->x < 0 || p->x >= SCREEN_WIDTH) p->velocity_x = -p->velocity_x;
    if (p->y < 0 || p->y >= SCREEN_HEIGHT) p->velocity_y = -p->velocity_y;
}

//________________________________________________________________
static void UpdateRak(rak_p p)
{
    if (p->movable == false || p->dead == true) return;
    
    p->velocity_x += p->accel_x * time_scale;
    p->velocity_y += p->accel_y * time_scale;
    
    if (p->velocity_x > RAKI_MAX_VELOCITY) p->velocity_x = RAKI_MAX_VELOCITY;
    if (p->velocity_x < RAKI_MIN_VELOCITY) p->velocity_x = RAKI_MIN_VELOCITY;
    if (p->velocity_y > RAKI_MAX_VELOCITY) p->velocity_y = RAKI_MAX_VELOCITY;
    if (p->velocity_y < RAKI_MIN_VELOCITY) p->velocity_y = RAKI_MIN_VELOCITY;
	
    p->x += (int)(p->velocity_x * time_scale);
    p->y += (int)(p->velocity_y * time_scale);
    
      // Reflect raki bottles
    if (p->x < 0 || p->x >= SCREEN_WIDTH) p->velocity_x = -p->velocity_x;
    if (p->y < 0 || p->y >= SCREEN_HEIGHT) p->velocity_y = -p->velocity_y;
}

//________________________________________________________________
static void DrawPlayer(player_p p)
{
    SDL_Rect src, dest;
    src.x = 0;
    src.y = 0;
    src.w = roto[p->angle]->w;
    src.h = roto[p->angle]->h;
    dest.x = p->x - roto[p->angle]->w/2;
    dest.y = p->y - roto[p->angle]->h/2;
    dest.w = roto[p->angle]->w;
    dest.h = roto[p->angle]->h;
    SDL_BlitSurface(roto[p->angle], &src, screen, &dest);
}

//________________________________________________________________
static void DrawNme(nme_p p)
{
    SDL_Rect src, dest;
    src.x = 0;
    src.y = 0;
    src.w = rotonme[p->angle]->w;
    src.h = rotonme[p->angle]->h;
    dest.x = p->x - rotonme[p->angle]->w/2;
    dest.y = p->y - rotonme[p->angle]->h/2;
    dest.w = rotonme[p->angle]->w;
    dest.h = rotonme[p->angle]->h;
    if (p->angle >= 359) p->angle = 0;
    SDL_BlitSurface(rotonme[p->angle++], &src, screen, &dest);
}

//________________________________________________________________
static void DrawBmb(bmb_p p)
{
    SDL_Rect src, dest;
    src.x = 0;
    src.y = 0;
    src.w = rotobomb[p->angle]->w;
    src.h = rotobomb[p->angle]->h;
    dest.x = p->x - rotobomb[p->angle]->w/2;
    dest.y = p->y - rotobomb[p->angle]->h/2;
    dest.w = rotobomb[p->angle]->w;
    dest.h = rotobomb[p->angle]->h;
    if (p->angle <= 0) p->angle = 359;
    SDL_BlitSurface(rotobomb[p->angle--], &src, screen, &dest);
}

//________________________________________________________________
static void DrawRak(rak_p p)
{
    SDL_Rect src, dest;
    src.x = 0;
    src.y = 0;
    src.w = rotoraki[p->angle]->w;
    src.h = rotoraki[p->angle]->h;
    dest.x = p->x - rotoraki[p->angle]->w/2;
    dest.y = p->y - rotoraki[p->angle]->h/2;
    dest.w = rotoraki[p->angle]->w;
    dest.h = rotoraki[p->angle]->h;
    if (p->angle == 345 || p->angle == 15) p->toggleRotDir();
    if (p->rotDir) {
      SDL_BlitSurface(rotoraki[p->angle++], &src, screen, &dest);
      if (p->angle >= 359) p->angle = 0;
    } else {
      SDL_BlitSurface(rotoraki[p->angle--], &src, screen, &dest);
      if (p->angle <= 0) p->angle = 359;
    }
}

//________________________________________________________________
static void geriSay(int rakam)
{
  if (1 == rakam) {
    dest.x = SCREEN_WIDTH/2  - bir->w/2;
    dest.y = SCREEN_HEIGHT/2 - bir->h/2;   
    SDL_BlitSurface(bir, &src, screen, &dest); 
  }
  if (2 == rakam) {
    dest.x = SCREEN_WIDTH/2  - iki->w/2;
    dest.y = SCREEN_HEIGHT/2 - iki->h/2;
    SDL_BlitSurface(iki, &src, screen, &dest);
  }
  if (3 == rakam) {
    dest.x = SCREEN_WIDTH/2  - uc->w/2;
    dest.y = SCREEN_HEIGHT/2 - uc->h/2;
    SDL_BlitSurface(uc,  &src, screen, &dest); 
  }
}

//________________________________________________________________
static void Baslat(int noNme, int noBmb, int noRak) 
{
    paused = false;
    helped = false;
    isGameOver = false;
    isWon = false;
    contClear = true;
    controlsInverted = false;
    dusmanSayisi = 0;
    bombaSayisi = 0;
    rakiSayisi = 0;
    oluSayisi = 0;
    ClearPlayingSounds();
    PlaySound(&oyunMuzigi);
    InitPlayer(&player);
    if (noNme > MAX_NUMBER_OF_ENEMIES) noNme = MAX_NUMBER_OF_ENEMIES; 
    for (int i=0 ; i<noNme ; i++) {
      InitNme(&nme[i]);
    }
    if (noBmb > MAX_NUMBER_OF_BOMBS) noBmb = MAX_NUMBER_OF_BOMBS; 
    for (int j=0 ; j<noBmb ; j++) {
      InitBmb(&bmb[j]);
    }
    if (noRak > MAX_NUMBER_OF_RAKI) noRak = MAX_NUMBER_OF_RAKI; 
    for (int j=0 ; j<noRak ; j++) {
      InitRak(&rak[j]);
    }
    geriSayac = 0;
}

//________________________________________________________________
static void freezeAll()
{
  for (int i=0 ; i<dusmanSayisi ; i++) if (nme[i].dead == false) nme[i].movable = false;
  for (int j=0 ; j<bombaSayisi  ; j++) if (bmb[j].dead == false) bmb[j].movable = false;
  for (int j=0 ; j<rakiSayisi   ; j++) if (rak[j].dead == false) rak[j].movable = false;
  player.movable = false;
}

//________________________________________________________________
static void releaseAll()
{
  for (int i=0 ; i<dusmanSayisi ; i++) if (nme[i].dead == false) nme[i].movable = true;
  for (int j=0 ; j<bombaSayisi  ; j++) if (bmb[j].dead == false) bmb[j].movable = true;
  for (int j=0 ; j<rakiSayisi   ; j++) if (rak[j].dead == false) rak[j].movable = true;
  player.movable = true;
}

//________________________________________________________________
static void Check4Death()
{
  // Checks for gaming events
  
    // Do I touch an nme ?
  for (int i=0 ; i<dusmanSayisi ; i++) {
      if (true == nme[i].dead) continue;
      if (PLAYER_HEIGHT/2 >= sqrt(pow((nme[i].x-player.x),2) + pow((nme[i].y-player.y),2))) {
          // Player dies
        PlaySound(&explosion);
        PlaySound(&kapat);
		    CreateParticleExplosion(player.x, player.y, 255, 255, 255, 5, 1000);
        CreateParticleExplosion(player.x, player.y, 255, 255, 255, 5, 1000);
        CreateParticleExplosion(player.x, player.y, 255, 0, 0, 5, 1000);
        player.movable = false;
        freezeAll();
        isGameOver = true;
        return;
      }  
  }

    // Did I just have raki with white cheese and melon ?
  for (int j=0 ; j<rakiSayisi ; j++) {
     if (true == rak[j].dead) continue;
          // Bottle of raki disappears and...
     if (PLAYER_HEIGHT >= sqrt(pow((rak[j].x-player.x),2) + pow((rak[j].y-player.y),2))) {
        rak[j].dead = true;
        PlaySound(&pith);
        CreateParticleExplosion(rak[j].x, rak[j].y, 255, 255, 255, 5, 100);
          // Toggle controls
        if (controlsInverted) controlsInverted = false;
        else controlsInverted = true;
      }
  }
  
    // Do I touch a bomb ?
  for (int j=0 ; j<bombaSayisi ; j++) {
     if (true == bmb[j].dead) continue;
          // Bomb disappears and...
     if (PLAYER_HEIGHT >= sqrt(pow((bmb[j].x-player.x),2) + pow((bmb[j].y-player.y),2))) {
        bmb[j].dead = true;
        PlaySound(&explosion);
		    CreateParticleExplosion(bmb[j].x, bmb[j].y, 255, 255, 0, 5, 1000);
        CreateParticleExplosion(bmb[j].x, bmb[j].y, 255, 255, 255, 5, 1000);
        CreateParticleExplosion(bmb[j].x, bmb[j].y, 255, 0, 0, 5, 1000);
          // Does any nme explode ?
        for (int k=0 ; k<dusmanSayisi ; k++) {
          if (true == nme[k].dead) continue;
            // ...within-circle-NMEs die
          if (DEATH_FIELD >= sqrt(pow((bmb[j].x-nme[k].x),2) + pow((bmb[j].y-nme[k].y),2))) {
            nme[k].dead = true;
            oluSayisi++;
            PlaySound(&explosion);
		        CreateParticleExplosion(nme[k].x, nme[k].y, 255, 255, 0, 5, 1000);
            CreateParticleExplosion(nme[k].x, nme[k].y, 255, 255, 255, 5, 1000);
            CreateParticleExplosion(nme[k].x, nme[k].y, 255, 0, 0, 5, 1000);
          }
        }
      } else if (DEATH_FIELD >= sqrt(pow((bmb[j].x-player.x),2) + pow((bmb[j].y-player.y),2))) {
          // Am I close to a bomb so that effective area becomes visible
        dest.x = bmb[j].x-range->w/2;
        dest.y = bmb[j].y-range->h/2;
        SDL_BlitSurface(range, &src, screen, &dest);
      }
  }
  
    // There must be at least one fast nme
  float totalSpeed = 0.0;
  float threshold  = sqrt(2* pow(NME_MAX_VELOCITY, 2)); 
  for (int i=0 ; i<dusmanSayisi ; i++) {
    if (nme[i].dead == false) {
      totalSpeed += sqrt(pow(nme[i].velocity_x, 2) + pow(nme[i].velocity_y, 2));
    }
  }
    // As the NMEs disappear, last ones go crazy in speed
  if (totalSpeed/dusmanSayisi <= 0.1*threshold) {
    for (int i=0 ; i<dusmanSayisi ; i++) {
      if (nme[i].dead == false) {
        if (nme[i].maxVel < PLAYER_MAX_VELOCITY) {
          nme[i].maxVel += 0.1 * NME_MAX_VELOCITY;
        }
      }
    }
  }
  
    // Dead end ?
  int number_of_bombs = 0, number_of_enemies = 0;
  for (int i=0 ; i<dusmanSayisi ; i++) if (false == nme[i].dead) number_of_enemies++;
  for (int i=0 ; i<bombaSayisi  ; i++) if (false == bmb[i].dead) number_of_bombs++;
  if (number_of_enemies != 0 && number_of_bombs == 0) { // Infinite escape is not allowed
    player.movable = false;
    freezeAll();
    isGameOver = true;
  }
  
    // Did I win ?
  if (0 == number_of_enemies) {
    isGameOver = true;
    isWon = true;
    freezeAll();
  }
  
    // Update the score table
  updateDisplay(number_of_bombs, number_of_enemies);
} 

//________________________________________________________________
static void updateDisplay(int noBmb, int noNme)
{
    // Updates the display
    
  char str[200];
  dest.x = SCREEN_WIDTH/100;
  
  SDL_Surface *display;
  
  sprintf(str, "Score...: %d", oluSayisi);
  display = TTF_RenderText_Blended(smallfont, str, PtextColor);
  dest.y = 7.0*SCREEN_HEIGHT/10;
  SDL_BlitSurface(display, &src, screen, &dest);
  SDL_FreeSurface(display);
  
  sprintf(str, "Current.: %d", noNme);
  display = TTF_RenderText_Blended(smallfont, str, PtextColor);
  dest.y = 7.5*SCREEN_HEIGHT/10;
  SDL_BlitSurface(display, &src, screen, &dest);
  SDL_FreeSurface(display);
  
  sprintf(str, "Bombs...: %d", noBmb);
  display = TTF_RenderText_Blended(smallfont, str, PtextColor);
  dest.y = 8.0*SCREEN_HEIGHT/10;
  SDL_BlitSurface(display, &src, screen, &dest);
  SDL_FreeSurface(display);
  
  if (isAccel) 
    sprintf(str, "[C]ontrol: ARDUINO+ACCELEROMETER / keyboard");
  else 
    sprintf(str, "[C]ontrol: arduino+accelerometer / KEYBOARD");
  display = TTF_RenderText_Blended(smallfont, str, PtextColor);
  dest.y = 8.5*SCREEN_HEIGHT/10;
  SDL_BlitSurface(display, &src, screen, &dest);
  SDL_FreeSurface(display);
  
  sprintf(str, "[H]elp, Level select:[N, 1..9, 0], [Q]uit");
  display = TTF_RenderText_Blended(smallfont, str, PtextColor);
  dest.y = 9.0*SCREEN_HEIGHT/10;
  SDL_BlitSurface(display, &src, screen, &dest);
  SDL_FreeSurface(display);
  
  display = TTF_RenderText_Blended(smallfont, arduinoOutput, PtextColor);
  dest.y = 9.5*SCREEN_HEIGHT/10;
  SDL_BlitSurface(display, &src, screen, &dest);
  SDL_FreeSurface(display);
}

//________________________________________________________________
static void PlayGame() 
{
    int quit = 0;
    int cur_ticks = 0;
    int prev_ticks = SDL_GetTicks();
    int start_time = time(NULL);
    char inStr[50], ilk[50], ikinci[50], ucuncu[50], dorduncu[50], besinci[50], altinci[50], yedinci[50];
    char tmpStr[100];
    int accelX, accelY, accelZ;
    Uint8 *keystate;
    Baslat(10, 5, 3);
    while (quit == 0) {
      prev_ticks = cur_ticks;
      cur_ticks = SDL_GetTicks();
		  time_scale = (double)(cur_ticks-prev_ticks)/scaler;
		  SDL_PumpEvents();
		  keystate = SDL_GetKeyState(NULL); // grab keyboard snapshot
		  if (keystate[SDLK_q] || keystate[SDLK_ESCAPE]) quit = 1;
      src.x = 0;
      src.y = 0;
      src.w = background->w;
      src.h = background->h;
      dest = src;
      if (!controlsInverted) SDL_BlitSurface(background, &src, screen, &dest);
      else SDL_BlitSurface(backgroundInv, &src, screen, &dest);
      player.accel_x = 0;
      player.accel_y = 0;
      while( SDL_PollEvent( &event ) ) {  // tek tus ve pencere olaylarini gozle
        if( event.type == SDL_QUIT ) quit = true; // pencereyi x lediginde kapatir
          if( event.type == SDL_KEYDOWN ) {
            switch( event.key.keysym.sym ) {
              case SDLK_h: {                          // Helped
                if (geriSayac < 200) break;
                if (paused) break;
                if (!isGameOver) {
                if (!helped) {                      // durdur herseyi
                    PlaySound(&dur);
                    player.movable = false;
                    for (int i=0 ; i<dusmanSayisi ; i++) nme[i].movable = false;
                    for (int j=0 ; j<bombaSayisi  ; j++) bmb[j].movable = false;
                    for (int k=0 ; k<rakiSayisi   ; k++) rak[k].movable = false;
                    helped = true;
                  } else {                            // devam ettir herseyi
                    PlaySound(&dur);
                    player.movable = true;
                    for (int i=0 ; i<dusmanSayisi ; i++) {
                      if (nme[i].dead == false) nme[i].movable = true;
                    }
                    for (int j=0 ; j<bombaSayisi ; j++) {
                      if (bmb[j].dead == false) bmb[j].movable = true;
                    }
                    for (int k=0 ; k<rakiSayisi ; k++) {
                      if (rak[k].dead == false) rak[k].movable = true;
                    }
                    helped = false;
                  }}
                break;
              }
              case SDLK_p: {                          // Paused
                if (geriSayac < 200) break;
                if (helped) break;
                if (!isGameOver) {
                  if (!paused) {                      // durdur herseyi
                    PlaySound(&dur);
                    player.movable = false;
                    for (int i=0 ; i<dusmanSayisi ; i++) nme[i].movable = false;
                    for (int j=0 ; j<bombaSayisi  ; j++) bmb[j].movable = false;
                    for (int k=0 ; k<rakiSayisi   ; k++) rak[k].movable = false;
                    paused = true;
                  } else {                            // devam ettir herseyi
                    PlaySound(&dur);
                    player.movable = true;
                    for (int i=0 ; i<dusmanSayisi ; i++) {
                      if (nme[i].dead == false) nme[i].movable = true;
                    }
                    for (int j=0 ; j<bombaSayisi ; j++) {
                      if (bmb[j].dead == false) bmb[j].movable = true;
                    }
                    for (int k=0 ; k<rakiSayisi ; k++) {
                      if (rak[k].dead == false) rak[k].movable = true;
                    }
                    paused = false;
                    break;
                  }
                }
              break;
              }
            break;
            case SDLK_n:
              Baslat(10, 5,   2); // New game with 10 enemies & 5 bombs & 3 raki bottles
              break;
            case SDLK_1:
              Baslat(10, MAX_NUMBER_OF_BOMBS,   2);
              break;
            case SDLK_2:
              Baslat(MAX_NUMBER_OF_ENEMIES, MAX_NUMBER_OF_BOMBS,   3);
              break;
            case SDLK_3:
              Baslat(40, 6,   3);
              break;
            case SDLK_4:
              Baslat(50, 7,   4); 
              break;
            case SDLK_5:
              Baslat(60, 8,   4);
              break;
            case SDLK_6:
              Baslat(70, 9,   4);
              break;
            case SDLK_7:
              Baslat(80, 10,  4);
              break;
            case SDLK_8:
              Baslat(90, 11,  4); 
              break;
            case SDLK_9:
              Baslat(100, 12, 4);
              break;
            case SDLK_0:
              Baslat(150, 13, 5);
              break;  
            case SDLK_c:
              if (isAccel) isAccel = false;
              else isAccel = true;
              break; 
            }
          }
        }
          // Select the control type: keyboard or arduino/gadget
        if (!isAccel) {
          if (keystate[SDLK_UP] || keystate[SDLK_w]) {
              // yukari (go up)
            if (!controlsInverted) player.accel_y = PLAYER_FORWARD_THRUST;
            else player.accel_y = PLAYER_REVERSE_THRUST;
            if (player.velocity_x > 0) player.velocity_x -= VELOCITY_INCREMENT;
            if (player.velocity_x < 0) player.velocity_x += VELOCITY_INCREMENT;
          } else if (keystate[SDLK_DOWN] || keystate[SDLK_s]) {  
              // asagi (go down)
            if (!controlsInverted) player.accel_y = PLAYER_REVERSE_THRUST;
            else player.accel_y = PLAYER_FORWARD_THRUST;
            if (player.velocity_x > 0) player.velocity_x -= VELOCITY_INCREMENT;
            if (player.velocity_x < 0) player.velocity_x += VELOCITY_INCREMENT;
          } else if (keystate[SDLK_LEFT] || keystate[SDLK_a]) {  
              // sol (go to left)
            if (!controlsInverted) player.accel_x = PLAYER_FORWARD_THRUST;
            else player.accel_x = PLAYER_REVERSE_THRUST;
            if (player.velocity_y > 0) player.velocity_y -= VELOCITY_INCREMENT;
            if (player.velocity_y < 0) player.velocity_y += VELOCITY_INCREMENT;
          } else if (keystate[SDLK_RIGHT] || keystate[SDLK_d]) { 
              // sag (go to right)
            if (!controlsInverted) player.accel_x = PLAYER_REVERSE_THRUST;
            else player.accel_x = PLAYER_FORWARD_THRUST;
            if (player.velocity_y > 0) player.velocity_y -= VELOCITY_INCREMENT;
            if (player.velocity_y < 0) player.velocity_y += VELOCITY_INCREMENT;
          }
        } else { // Arduino delivers: "xxx.87.yyy.45.zzz.67.xyz\n", where 87, 45 
                 // and 67 are the accelerometer values read-out from the arduino.
                 // Edit this part: use accelerometer values to control the ship.
          /*
             Ivmeolcer ayari (Accelerometer calibration):
             ----------------------x---y----z------------
             duz iken (streight):  79, 86,  110
             en sag (right-most):  79, 50,  86
             en sol (left-most) :  79, 126, 82
             en ileri (forward) : 114, 88,  85
             en geri (backward) :  42, 88,  85
    
             Yonelim (allignment): Arduino/Gadget'i duz tuttugumda (holding streight):
             masa duzlemi (table surface)          -> soldan saga (left to right)    , y ekseni (y-axis)
                                                      asagidan yukari (bottom to top), x ekseni (x-axis)
             masaya dik (perpendicular to desktop) -> asagidan yukari (bottom to top), z ekseni (z-axis)
          */
          
          serialport_read_until(fd, inStr, '\n');                // Read serial port until line end ("\n")
          sprintf(arduinoOutput, "Arduino delivers: %s", inStr); // Create the string to be displayed
          printf("%s \n", arduinoOutput);                        // Print the created string
          
          // Your optional homework: parse this line and use the (x,y,z) values to control the ship.
          // Use the keyboard controls above as your starting point and send your solutions to:
          //
          //   Ozgur.Cobanoglu@cern.ch
          //
          // Have fun !!
        }

        UpdatePlayer(&player);
        DrawPlayer(&player);

        for (int j=0 ; j<bombaSayisi ; j++) {
          if (bmb[j].dead == true) continue;
          UpdateBmb(&bmb[j]);
          DrawBmb(&bmb[j]);
        }

        for (int i=0 ; i<dusmanSayisi ; i++) {
          if (nme[i].dead == true) continue;
          UpdateNme(&nme[i]);
          DrawNme(&nme[i]);
        }
        
        for (int i=0 ; i<rakiSayisi ; i++) {
          if (rak[i].dead == true) continue;
          UpdateRak(&rak[i]);
          DrawRak(&rak[i]);
        }
        
          // Gerisayac
        if (geriSayac<60) {
          freezeAll();
          geriSay(3); 
          if (geriSayac == 1) PlaySound(&dur);
        } else if (geriSayac<120 && geriSayac>60) {
          geriSay(2); 
          if (geriSayac == 61) PlaySound(&dur);
        } else if (geriSayac<180 && geriSayac>120) {
          geriSay(1);
          if (geriSayac == 121) PlaySound(&dur);
        }
        if (geriSayac == 180) releaseAll();
        if (geriSayac < 200) geriSayac++;
      
        if (isGameOver) { // Game is over
          if (contClear) {
            ClearPlayingSounds();
            if (isWon) PlaySound(&basariMuzigi); 
            else PlaySound(&gameover);
          }
            // Display an appropriate message
          if (isWon) SDL_BlitSurface(vmsg, &src, screen, &dest);
          else SDL_BlitSurface(msg, &src, screen, &dest);
            // Update the display properly
          int number_of_bombs = 0, number_of_enemies = 0;
          for (int i=0 ; i<dusmanSayisi ; i++) if (false == nme[i].dead) number_of_enemies++;
          for (int i=0 ; i<bombaSayisi  ; i++) if (false == bmb[i].dead) number_of_bombs++;
          updateDisplay(number_of_bombs, number_of_enemies);
          DrawParticles(screen, 0, 0);
          contClear = false;
        } else { // Game continues
          DrawParticles(screen, 0, 0);
          Check4Death();
        }
        
        dest.x = SCREEN_WIDTH/8;
        dest.y = SCREEN_HEIGHT/16;
        
        if (paused) SDL_BlitSurface(credits, &src, screen, &dest);
        if (helped) SDL_BlitSurface(help, &src, screen, &dest);
           
        UpdateParticles(); 
        SDL_Flip(screen);
    }
}

//________________________________________________________________
int main() {

      // Recently introduced arduino part starts here
    
    int baudrate = B9600;
    char port[50] = "/dev/ttyACM0";

    fd = serialport_init(port, baudrate);
    if (fd == -1)
      isAccel = false;
    else
      isAccel = true;

      // Historically "dodger" part starts here
      
    dusmanSayisi = 13;
    rakiSayisi = 5;
    
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        printf("can not initialize SDL : %s\n", SDL_GetError());
        return 1;
    }
    if( TTF_Init() == -1 ) return false;    
    atexit(SDL_Quit);
    atexit(SDL_CloseAudio);
    screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 16, SDL_HWSURFACE);
    if (screen == NULL) {
        printf("error setting video mode :%s\n", SDL_GetError());
        return 1;
    }
    
      // Background normal
    temp = IMG_Load("resources/ardalan3.png"); 
    background = SDL_DisplayFormat(temp);
    SDL_FreeSurface(temp);
    if (NULL == background) {
        printf("ERROR: Can not load image: ardalan.\n");
        return 1;
    }
    
      // Background inverted
    temp = IMG_Load("resources/ardalan.png"); 
    backgroundInv = SDL_DisplayFormat(temp);
    SDL_FreeSurface(temp);
    if (NULL == backgroundInv) {
        printf("ERROR: Can not load image: ardalan inverted.\n");
        return 1;
    }
    
      // Player
    temp = IMG_Load("resources/ship5.png"); 
    if (NULL == temp) {
        printf("ERROR: Can not load image: ship.\n");
        return 1;
    } else 
      temp = SDL_DisplayFormatAlpha(temp);
  
      // Create rotated images of player
    for (int i=0 ; i<360 ; i++) {
      rotoTmp = rotozoomSurface(temp, 360-i, 1, 1);
      roto[i] = SDL_DisplayFormatAlpha(rotoTmp);
      if (NULL == roto[i]) {
        printf("ERROR: Can not load roto[0] image.\n");
        return 1;
      }
    }
    SDL_FreeSurface(temp);
    
      // Enemy
    temp = IMG_Load("resources/nme1.png");  
    if (NULL == temp) {
      printf("ERROR: Can not load image: nme.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    rock = SDL_DisplayFormatAlpha(temp);
    if (NULL == rock) {
      printf("ERROR: Can not load image: rock.\n");
      return 1;
    }  
    rock = SDL_DisplayFormatAlpha(rock);
    
          // Create rotated images of enemy
    for (int i=0 ; i<360 ; i++) {
      rotoTmp = rotozoomSurface(temp, 360-i, 1, 1);
      rotonme[i] = SDL_DisplayFormatAlpha(rotoTmp);
      if (NULL == rotonme[i]) {
        printf("ERROR: Can not load rotonme[0] image.\n");
        return 1;
      }
    }
    SDL_FreeSurface(temp);
    
      // Credits
    temp = IMG_Load("resources/credits.png");  
    if (NULL == temp) {
      printf("ERROR: Can not load image: credits.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    credits = SDL_DisplayFormatAlpha(temp);
    if (NULL == credits) {
      printf("ERROR: Can not load image: credits.\n");
      return 1;
    }  
    credits = SDL_DisplayFormatAlpha(credits);
    SDL_FreeSurface(temp); 
    
      // Help
    temp = IMG_Load("resources/help.png");  
    if (NULL == temp) {
      printf("ERROR: Can not load image: help.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    help = SDL_DisplayFormatAlpha(temp);
    if (NULL == help) {
      printf("ERROR: Can not load image: help.\n");
      return 1;
    }  
    help = SDL_DisplayFormatAlpha(help);
    SDL_FreeSurface(temp);      
    
      // Range
    temp = IMG_Load("resources/range.png");  
    if (NULL == temp) {
      printf("ERROR: Can not load image: range.\n");
      return 1;
    }
    temp = SDL_DisplayFormatAlpha(temp);
    range = SDL_DisplayFormatAlpha(temp);
    if (NULL == credits) {
      printf("ERROR: Can not load image: range.\n");
      return 1;
    }  
    range = SDL_DisplayFormatAlpha(range);
    SDL_FreeSurface(temp);  

      // Raki sisesi
    temp = IMG_Load("resources/raki2.png"); 
    if (NULL == temp) {
      printf("ERROR: Can not load image: raki.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    raki = SDL_DisplayFormatAlpha(temp);
    if (NULL == raki) {
      printf("ERROR: Can not load image: raki.\n");
      return 1;
    }
    raki = SDL_DisplayFormatAlpha(raki);
        
          // Create rotated images of raki bottle
    for (int i=0 ; i<360 ; i++) {
      rotoTmp = rotozoomSurface(temp, 360-i, 1, 1);
      rotoraki[i] = SDL_DisplayFormatAlpha(rotoTmp);
      if (NULL == rotoraki[i]) {
        printf("ERROR: Can not load rotoraki[0] image.\n");
        return 1;
      }
    }
    SDL_FreeSurface(temp);
    
      // Bomb
    temp = IMG_Load("resources/bomba.png"); 
    if (NULL == temp) {
      printf("ERROR: Can not load image: bomba.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    bomba = SDL_DisplayFormatAlpha(temp);
    if (NULL == bomba) {
      printf("ERROR: Can not load image: bomba.\n");
      return 1;
    } 
    bomba = SDL_DisplayFormatAlpha(bomba);
        
      // Create rotated images of bomb
    for (int i=0 ; i<360 ; i++) {
      rotoTmp = rotozoomSurface(temp, 360-i, 1, 1);
      rotobomb[i] = SDL_DisplayFormatAlpha(rotoTmp);
      if (NULL == rotobomb[i]) {
        printf("ERROR: Can not load rotobomba[0] image.\n");
        return 1;
      }
    }
    SDL_FreeSurface(temp);
    
      // Sayac rakamlari - 1
    temp = IMG_Load("resources/1.png"); 
    if (NULL == temp) {
      printf("ERROR: Can not load image: 1.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    bir  = SDL_DisplayFormatAlpha(temp);
    if (NULL == bir) {
      printf("ERROR: Can not load image: bir.\n");
      return 1;
    } 
    bir = SDL_DisplayFormatAlpha(bir);
    
          // Sayac rakamlari - 2
    temp = IMG_Load("resources/2.png"); 
    if (NULL == temp) {
      printf("ERROR: Can not load image: 2.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    iki  = SDL_DisplayFormatAlpha(temp);
    if (NULL == iki) {
      printf("ERROR: Can not load image: iki.\n");
      return 1;
    } 
    iki = SDL_DisplayFormatAlpha(iki);
    
          // Sayac rakamlari - 3
    temp = IMG_Load("resources/3.png"); 
    if (NULL == temp) {
      printf("ERROR: Can not load image: 3.\n");
      return 1;
    } 
    temp = SDL_DisplayFormatAlpha(temp);
    uc  = SDL_DisplayFormatAlpha(temp);
    if (NULL == uc) {
      printf("ERROR: Can not load image: uc.\n");
      return 1;
    } 
    uc = SDL_DisplayFormatAlpha(uc);
    
      // Game font
    font = TTF_OpenFont( "resources/Orbitron-Regular.ttf", 80 );
    if( font == NULL ) return false;
    smallfont = TTF_OpenFont( "resources/Orbitron-Regular.ttf", 20 );
    if( smallfont == NULL ) return false;
    
    msg = TTF_RenderText_Solid(font, "You failed :(", textColor);
    vmsg = TTF_RenderText_Solid(font,"You won :D", textColor);
    pmsg = TTF_RenderText_Solid(font,"Paused :|", PtextColor);
    
      // Update the screen
    if( SDL_Flip( screen ) == -1 ) return 1; 
    
    desired.freq = 44100;
    desired.format = AUDIO_S16; //signed 16bit sample
    desired.samples = 4096;
    desired.channels = 2; // stereo
    desired.callback = AudioCallback;
    desired.userdata = NULL;
    char str[200];
    if (SDL_OpenAudio(&desired, &obtained) < 0) {
        printf("Can not open audio device : %s\n", SDL_GetError());
        return 1;
    }
    sprintf(str, "resources/expl.wav");
    if (LoadAndConvertSound(str, &obtained, &explosion) != 0) {
        printf("Can not load sound: expl.\n");
        return 1;
    }
    sprintf(str, "resources/yakin.wav");
    if (LoadAndConvertSound(str, &obtained, &yakin) != 0) {
        printf("Can not load sound: yakin.\n");
        return 1;
    }
    sprintf(str, "resources/pith.wav");
    if (LoadAndConvertSound(str, &obtained, &pith) != 0) {
        printf("Can not load sound: pith.\n");
        return 1;
    }
    sprintf(str, "resources/dur.wav");
    if (LoadAndConvertSound(str, &obtained, &dur) != 0) {
        printf("Can not load sound: dur.\n");
        return 1;
    }
    sprintf(str, "resources/gameover3.wav");
    if (LoadAndConvertSound(str, &obtained, &gameover) != 0) {
        printf("Can not load sound: gameover.\n");
        return 1;
    }
    sprintf(str, "resources/kapat.wav");
    if (LoadAndConvertSound(str, &obtained, &kapat) != 0) {
        printf("Can not load sound: kapat.\n");
        return 1;
    }
    sprintf(str, "resources/Hayko_Cepkin_Son_Veda_Editted.wav");
    if (LoadAndConvertSound(str, &obtained, &oyunMuzigi) != 0) {
        printf("Can not load sound: hayko.\n");
        return 1;
    }
    sprintf(str, "resources/won.wav");
    if (LoadAndConvertSound(str, &obtained, &basariMuzigi) != 0) {
        printf("Can not load sound: won.\n");
        return 1;
    }
    ClearPlayingSounds();
    SDL_PauseAudio(0);
    
    SDL_WM_SetCaption("SIYIRDUiNO", "SIYIRDUiNO");
    SDL_ShowCursor(1);
    PlayGame();
    
    for (int i=0 ; i<360 ; i++) SDL_FreeSurface(roto[i]);
    SDL_FreeSurface(rock);
    SDL_FreeSurface(bomba);
    SDL_FreeSurface(raki);
    SDL_FreeSurface(range);
    SDL_FreeSurface(background);
    SDL_FreeSurface(backgroundInv);
    SDL_FreeSurface(bir);
    SDL_FreeSurface(iki);
    SDL_FreeSurface(uc);
    
    TTF_CloseFont(font);
    TTF_CloseFont(smallfont);
    TTF_Quit();
    SDL_PauseAudio(1);
    SDL_LockAudio();
    free(explosion.samples);
    SDL_UnlockAudio();
    return 0;
}
