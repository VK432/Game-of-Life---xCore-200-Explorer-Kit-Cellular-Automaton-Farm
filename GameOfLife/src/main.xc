// COMS20001 - Cellular Automaton Farm - Final Version Varadhan Kalidasan (vk14831), Jonathon Wingrove(jw14897).

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 128                //image height
#define  IMWD 128               //image width
#define  BITWD IMWD/16           //width in bytes
#define  HT8   IMHT/8            //offset between each workers start point

typedef unsigned char uchar;      //using uchar as shorthand
typedef unsigned short ushor;     //using ushor as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to accelerometer
on tile[0] : port p_sda = XS1_PORT_1F;

on tile[0] : in port buttons = XS1_PORT_4E; //port to access xCore-200 buttons
on tile[0] : out port leds = XS1_PORT_4F;   //port to access xCore-200 LEDs

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for accelerometer
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(chanend c_out)
{
  char infname[] = "128x128.pgm";//put your input image path here
  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

  //Open PGM file
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s\n.", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
    }
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream:Done...\n" );
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Calculates the number of living neighbours a square has, given a grid, coordinates
// and an 'a' offset, where 'a' is used to locate the specific bit
//
/////////////////////////////////////////////////////////////////////////////////////////
int livingNeighbours(ushor grid[(BITWD)+2][3], int x, int y, int a)
{
    int total = 0;
    ushor val;
    int bit;
    if(a ==0){
        val = grid[(x-1)][(y-1)];
        bit = (val >> (0)) & 1;
        if (bit == 1) total++;
        val = grid[(x-1)][y];
        bit = (val >> (0)) & 1;
        if (bit == 1) total++;
        val = grid[(x-1)][(y+1)];
        bit = (val >> (0)) & 1;
        if (bit == 1) total++;


        val = grid[(x)][(y-1)];
        bit = (val >> (14)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][y];
        bit = (val >> (14)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][(y+1)];
        bit = (val >> (14)) & 1;
        if (bit == 1) total++;

        }
    else if(a ==15){
        val = grid[(x)][(y-1)];
        bit = (val >> (1)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][y];
        bit = (val >> (1)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][(y+1)];
        bit = (val >> (1)) & 1;
        if (bit == 1) total++;


        val = grid[(x+1)][(y-1)];
        bit = (val >> (15)) & 1;
        if (bit == 1) total++;
        val = grid[(x+1)][y];
        bit = (val >> (15)) & 1;
        if (bit == 1) total++;
        val = grid[(x+1)][(y+1)];
        bit = (val >> (15)) & 1;
        if (bit == 1) total++;

        }
    else{
        val = grid[(x)][(y-1)];
        bit = (val >> ((15-a)+1)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][y];
        bit = (val >> ((15-a)+1)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][(y+1)];
        bit = (val >> ((15-a)+1)) & 1;
        if (bit == 1) total++;


        val = grid[(x)][(y-1)];
        bit = (val >> ((15-a)-1)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][y];
        bit = (val >> ((15-a)-1)) & 1;
        if (bit == 1) total++;
        val = grid[(x)][(y+1)];
        bit = (val >> ((15-a)-1)) & 1;
        if (bit == 1) total++;

        }

    val = grid[x][(y+1)];
    bit = (val >> (15-a)) & 1;
    if (bit == 1) total++;
    val = grid[x][(y-1)];
    bit = (val >> (15-a)) & 1;
    if (bit == 1) total++;

    return total;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Stores the image and distributes parts of the image to the workers.
// Listens for both button presses and signals from the accelerometer in order to
// trigger both pauses, and image outputs.
// Recieves new images from the harvester, and replaces old image with new image.
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend fromButtons, chanend toTimer, chanend toWorker1, chanend toWorker2, chanend toWorker3, chanend toWorker4, chanend toWorker5, chanend toWorker6, chanend toWorker7, chanend toWorker8, chanend fromHarv)
{
  ushor val;
  uchar line_number = 0;
  int harv = 0;
  int acc = 0;
  int button = 0;
  int iterations = 0;
  ushor grid[BITWD][IMHT];         //Stored image
  int living = 0;               //Living square count, used when paused

  printf( "ProcessImage:Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Button Press...\n" );

  while (button != 14)      //Waits for initial button press to start reading in file
  {
        fromButtons :> button;
  }

  leds <: 4;
  printf( "Processing...\n" );
  for( int y = 0; y < IMHT; y++ ) {   //go through all lines
    for( int x = 0; x < BITWD; x++ ) { //go through each pixel per line
      int a = 0;
      ushor line = 0;
      while(a != 16){               //Compresses each byte into a single bit of a unsigned short
          uchar in_out;             //if the square is white, bit is set to 1, if black set to 0
          c_in :> in_out;
          if(in_out != 0){
              line |= 1 << (15-a);
          }
          a++;
      }
      grid[x][y] = line;
    }
  }
  leds <: 0;
  printf( "\nFinished Reading File...\n" );
  toTimer <: 1;

  while(1)
  {
  select {
      case fromButtons :> button: // Check if button is pressed
          if(button == 13) {
              c_out <: 1;
              leds <: 2;
              for( int y = 0; y < IMHT; y++ ) {   //go through all lines
                  for( int x = 0; x < BITWD; x++ ) { //go through each pixel per line
                      val = grid[x][y];
                      int a = 0;
                      while(a != 16){
                          uchar colour = 0;
                          int bit = (val >> (15-a)) & 1;    //checks bit value, and sends uncompressed value to output
                          if(bit == 1){
                              colour = 255;
                          }
                          c_out <: colour;
                          a++;
                      }

                  }
              }
              leds <: 0;
          }
          break;
      case fromAcc :> acc:
          printf("Paused..\n");
          leds <: 8;
          living = 0;
          for (int y = 0; y < IMHT; y++) {          //iterates over every pixel of the image
              for (int x = 0; x < BITWD; x++) {
                  val = grid[x][y];
                  int a = 0;
                  while(a != 16){
                      int bit = (val >> (15-a)) & 1;        //checks bit value and counts number of living cells
                      if(bit != 0){
                          living++;
                      }
                      a++;
                  }
              }
          }
          toTimer <: 1;
          int time = 0;
          toTimer :> time;
          printf("Rounds completed: %d\n", iterations);
          printf("Live cells: %d\n", living);
          printf("Time elapsed: %d\n", time);
          fromAcc :> acc;
          printf("Unpaused..\n");
          toTimer <: 0;
          leds <: 0;
          break;
      case fromHarv :> harv:            //waits for signal from harvester
          if(harv == -1){               //one iteration is complete, so new image is received from harvester
              iterations++;
              for(int y=0; y<IMHT; y++){
                  for(int x=0; x<BITWD; x++){
                      fromHarv :> val;
                      grid[x][y] = val;
                  }
              }
              leds <: (iterations % 2);
          }else if(harv == -2){                  //sends next line to each worker + an extra row and column on all sides
              int ny,nx;                         //adds a different line offest for each worker
              for(int y = -1; y < 2; y++)
              {
                  for (int x = -1; x < BITWD+1; x++)
                  {
                      nx=x;
                      if(nx == -1){nx=BITWD-1;}         //checks edge cases
                      if(nx == BITWD) nx = 0;

                      //Worker1
                      ny=y+line_number;
                      if(ny == -1) ny=IMHT-1;           //checks edge cases
                      val = grid[nx][ny];
                      toWorker1 <: val;

                      //Worker2
                      ny=y+line_number+(HT8);
                      val = grid[nx][ny];
                      toWorker2 <: val;

                      //Worker3
                      ny=y+line_number+(HT8*2);
                      val = grid[nx][ny];
                      toWorker3 <: val;

                      //Worker4
                      ny=y+line_number+(HT8*3);
                      val = grid[nx][ny];
                      toWorker4 <: val;

                      //Worker5
                      ny=y+line_number+(HT8*4);
                      val = grid[nx][ny];
                      toWorker5 <: val;

                      //Worker6
                      ny=y+line_number+(HT8*5);
                      val = grid[nx][ny];
                      toWorker6 <: val;

                      //Worker7
                      ny=y+line_number+(HT8*6);
                      val = grid[nx][ny];
                      toWorker7 <: val;

                      //Worker8
                      ny=y+line_number+(HT8*7);
                      if(ny == IMHT) ny = 0;             //checks edge cases
                      val = grid[nx][ny];
                      toWorker8 <: val;
                  }
              }
              line_number ++;
              if(line_number == HT8){                   //resets line_number counter after one full iteration
                  line_number = 0;
              }

          }
          break;
          }
      }
 }

/////////////////////////////////////////////////////////////////////////////////////////
//
// Receives processed lines from workers and stores them in new image, after a complete
// iteration, the new image is sent back to the distributor.
//
/////////////////////////////////////////////////////////////////////////////////////////
void harvester(chanend fromWorker1, chanend fromWorker2, chanend fromWorker3, chanend fromWorker4, chanend fromWorker5, chanend fromWorker6, chanend fromWorker7, chanend fromWorker8, chanend toDist){
    uchar line_number = 0;
    ushor newgrid[BITWD][IMHT];         //new image
    int sig;                            //used to tell the distributor to either send more lines to the workers, receive new image from here
    ushor val;
    int signal1;                        //signals used to synchronise workers
    int signal2;
    int signal3;
    int signal4;
    int signal5;
    int signal6;
    int signal7;
    int signal8;
    while(1){
        select{
            case fromWorker1 :> signal1:
                fromWorker2 :> signal2;
                fromWorker3 :> signal3;
                fromWorker4 :> signal4;
                fromWorker5 :> signal5;
                fromWorker6 :> signal6;
                fromWorker7 :> signal7;
                fromWorker8 :> signal8;
                if(signal1 == -2 && signal2 == -2 && signal3 == -2 && signal4 == -2 && signal5 == -2 && signal6 == -2 && signal7 == -2 && signal8 == -2)
                {
                    sig = -2;
                    toDist <: sig;
                }
                if(signal1 == -1 && signal2 == -1 && signal3 == -1 && signal4 == -1 && signal5 == -1 && signal6 == -1 && signal7 == -1 && signal8 == -1)
                {
                    for (int x = 0; x < BITWD; x++)             //copies processed lines to new image
                    {
                        fromWorker1 :> val;
                        newgrid[x][line_number] = val;
                        fromWorker2 :> val;
                        newgrid[x][line_number+(HT8)] = val;
                        fromWorker3 :> val;
                        newgrid[x][line_number+(HT8*2)] = val;
                        fromWorker4 :> val;
                        newgrid[x][line_number+(HT8*3)] = val;

                        fromWorker5 :> val;
                        newgrid[x][line_number+(HT8*4)] = val;
                        fromWorker6 :> val;
                        newgrid[x][line_number+(HT8*5)] = val;
                        fromWorker7 :> val;
                        newgrid[x][line_number+(HT8*6)] = val;
                        fromWorker8 :> val;
                        newgrid[x][line_number+(HT8*7)] = val;
                    }
                    line_number++;
                    if(line_number == HT8){                 //sends new image to distributor if one iteration is complete
                        sig = -1;
                        toDist <: sig;
                        line_number = 0;
                        for(int y=0; y<IMHT; y++){
                            for(int x=0; x<BITWD; x++){
                                val = newgrid[x][y];
                                toDist <: val;
                            }
                        }
                    }
                }
                break;
            }
        }
}

void worker(chanend fromDist, chanend toHarv)
{
    int y_dimension = 3;                //y dimension of grid section
    int x_dimension = BITWD+2;          //x dimension of grid section
    int signal;
    ushor grid[BITWD+2][3];             //unprocessed grid
    ushor newGrid[(BITWD)][1];          //processed grid
    ushor val;
    while(1)
    {
        signal = -2;
        toHarv <: signal;
        for(int y = 0; y < y_dimension; y++)        //receives image from distributor
        {
            for(int x = 0; x < x_dimension; x++)
            {
                fromDist :> val;
                grid[x][y] = val;
            }
        }
        for (int x = 1; x < x_dimension-1; x++)         //calculates whether each square will live or die, based on number of neighbours
        {
            val = grid[x][1];
            int a = 0;
            ushor line = 0;
            while(a != 16){
                int bit = (val >> (15-a)) & 1;
                int lNeighbours = livingNeighbours(grid, x, 1, a);
                if(bit == 0){
                    if(lNeighbours == 3)
                    {
                        line |= 1 << (15-a);
                    }
                }
                else{
                    if(lNeighbours == 2 || lNeighbours == 3){
                        line |= 1 << (15-a);
                    }
                }
                a++;
             }
             newGrid[x-1][0] = line;                     //copies processed coordinates into new grid
        }
        signal = -1;
        toHarv <: signal;
        for(int x = 0; x < x_dimension-2; x++)          //sends processed line to the harvester
        {
            val = newGrid[x][0];
            toHarv <: val;
        }
    }

}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(chanend c_in)
{
  char outfname[] = "testout.pgm";
  int res;
  uchar line[ IMWD ];

  while(1)
  {
  int start;
  c_in :> start;
  if(start == 1)
  {
      //Open PGM file
      printf( "DataOutStream:Start...\n" );
      res = _openoutpgm( outfname, IMWD, IMHT );
      if( res ) {
          printf( "DataOutStream:Error opening %s\n.", outfname );
          return;
      }

      //Compile each line of the image and write the image line-by-line
      for( int y = 0; y < IMHT; y++ ) {
          for( int x = 0; x < IMWD; x++ ) {
              c_in :> line[ x ];
            }
            _writeoutline( line, IMWD );
          }

          //Close the PGM image
          _closeoutpgm();
          printf( "DataOutStream:Done...\n" );
      }
  }
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read accelerometer, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void accelerometer(client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;
  int tilted = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }
  
  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the accelerometer x-axis forever
  while (1) {

    //check until new accelerometer data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to distributor after first tilt
    if (!tilted) {
      if (x>30) {
        tilted = 1 - tilted;
        toDist <: 1;
      }
    } else {
        if (x<5)
        {
            tilted = 1 - tilted;
            toDist <: 0;
        }
    }
  }
}

// Check for button presses
void buttonListener(in port b, chanend toDist) {
  int r;
  while (1) {
    b when pinseq(15)  :> r;    // check that no button is pressed
    b when pinsneq(15) :> r;    // check if some buttons are pressed
    if ((r==13) || (r==14))     // if either button is pressed
    toDist <: r;             // send button pattern to distributor
  }
}

void runningTimer(chanend toDist) {
    timer t;
    unsigned int time;
    const unsigned int period = 100000000;
    t :> time;
    int secsElapsed = 0;
    toDist :> int func;
    while (1)
    {
        select {
            case t when timerafter (time) :> void:
                secsElapsed++;
                time+=period;
                break;
            case toDist :> int func:
                //int pausedSecs = secsElapsed;
                if(func == 1)
                {
                   toDist <: secsElapsed;
                }
                toDist :> func;
                t :> time;
                //secsElapsed = pausedSecs;
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

  i2c_master_if i2c[1];               //interface to accelerometer


  chan c_inIO, c_outIO, c_control, buttonToDist, timerToDist, distToWorker1, distToWorker2, distToWorker3, distToWorker4, distToWorker5, distToWorker6, distToWorker7, distToWorker8, harvToWorker1, harvToWorker2, harvToWorker3, harvToWorker4, harvToWorker5, harvToWorker6, harvToWorker7, harvToWorker8, harvToDist;    //extend your channel definitions here

  par {
    on tile[0] : runningTimer(timerToDist);
    on tile[0] : buttonListener(buttons, buttonToDist);  //thread to check for button presses
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing accelerometer data
    on tile[0] : accelerometer(i2c[0],c_control);        //client thread readitng accelerometer data
    on tile[1] : DataInStream(c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream(c_outIO);       //thread to write out a PGM image
    on tile[0] : distributor(c_inIO, c_outIO, c_control, buttonToDist, timerToDist, distToWorker1, distToWorker2, distToWorker3, distToWorker4, distToWorker5, distToWorker6, distToWorker7, distToWorker8, harvToDist);//thread to coordinate work on image
    on tile[1] : harvester(harvToWorker1, harvToWorker2, harvToWorker3, harvToWorker4, harvToWorker5, harvToWorker6, harvToWorker7, harvToWorker8, harvToDist);
    on tile[0] : worker(distToWorker1, harvToWorker1);
    on tile[0] : worker(distToWorker2, harvToWorker2);
    on tile[0] : worker(distToWorker3, harvToWorker3);
    on tile[1] : worker(distToWorker4, harvToWorker4);
    on tile[1] : worker(distToWorker5, harvToWorker5);
    on tile[1] : worker(distToWorker6, harvToWorker6);
    on tile[1] : worker(distToWorker7, harvToWorker7);
    on tile[1] : worker(distToWorker8, harvToWorker8);
  }

  return 0;
}
