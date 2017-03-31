/*
 * Copyright (C) paki
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/manan_test/manan_test.c"
 * @author paki
 * testing if nps can compile custom modules
 */

#include "modules/manan_test/manan_test.h"
#include "stdio.h"

// think this contains state information from that file
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

// dont know where all these imports are from (most prolly for the curl)
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include <curl/curl.h>

// imports for jpeg; dont know the relevance of all of them
#include <fcntl.h> //this one used to write to ppmfile

#include <jpeglib.h>

struct MemoryStruct 
{
  char *memory;
  size_t size;
};

struct BmpStruct {
    unsigned char *bmp_buffer;
    unsigned long size;
    int width;
    int height;
    int pixel_size;
};

int heading_ind = 0;
unsigned long redsatheading[36];
unsigned long curredcount;

void my_init()
{
    printf("MY MODULE INIT \n\n");
}

//Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}

/* The following functions are to yaw the craft in different directions */
uint8_t increase_nav_heading_deg(int32_t *heading, int32_t increment)
{
  // calculate new heading in weird units
  int32_t increment_int32t = ANGLE_BFP_OF_REAL(RadOfDeg(increment));
  *heading = *heading + increment_int32t;
  // Check if your turn made it go out of bounds...
  INT32_COURSE_NORMALIZE(*heading);
  return FALSE;
}

int printheading(int32_t *heading)
{
    printf("Heading: %f \n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
    return 0;
}
/*
uint8_t yaw(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}
*/

/* The following functions are to move the waypoints in 4 directions relative
 * to the craft; these can be factored into one or two
 */
uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters)
{
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t moveWaypointBackwards(uint8_t waypoint, float distanceMeters)
{
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x - POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y - POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t moveWaypointLeftwards(uint8_t waypoint, float distanceMeters)
{
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x - POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t moveWaypointRightwards(uint8_t waypoint, float distanceMeters)
{
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.y = pos->y - POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t printme(void) 
{
    printf("THIS IS THE MESSAGE\n");
    return FALSE;
}

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  size_t realsize = size * nmemb;
  struct MemoryStruct *mem = (struct MemoryStruct *)userp;

  mem->memory = realloc(mem->memory, mem->size + realsize + 1);
  if(mem->memory == NULL) {
    /* out of memory! */
    printf("not enough memory (realloc returned NULL)\n");
    return 0;
  }

  memcpy(&(mem->memory[mem->size]), contents, realsize);
  mem->size += realsize;
  mem->memory[mem->size] = 0;

  return realsize;
}

int curl2mem(struct MemoryStruct* chunk)
{
  CURL *curl_handle;
  CURLcode res;

  // struct MemoryStruct chunk;

  // chunk.memory = malloc(1);  /* will be grown as needed by the realloc above */
  // chunk.size = 0;    /* no data at this point */

  curl_global_init(CURL_GLOBAL_ALL);

  /* init the curl session */
  curl_handle = curl_easy_init();

  /* specify URL to get */
  // curl_easy_setopt(curl_handle, CURLOPT_URL, "http://www.example.com/");
  // curl_easy_setopt(curl_handle, CURLOPT_URL, "http://blogs.smh.com.au/entertainment/getflickd/44655_native.jpeg.jpg");
  curl_easy_setopt(curl_handle, CURLOPT_URL, "http://localhost:1234/screenshot");
  // printf("Downloaded \n");

  /* send all data to this function  */
  curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);

  // printf("Written \n");
  /* we pass our 'chunk' struct to the callback function */
  // curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&chunk);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)chunk);

  /* some servers don't like requests that are made without a user-agent
     field, so we provide one */
  curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");

  /* get it! */
  res = curl_easy_perform(curl_handle);

  /* check for errors */
  if(res != CURLE_OK) {
    fprintf(stderr, "curl_easy_perform() failed: %s\n",
            curl_easy_strerror(res));
  }
//   else {
//     /*
//      * Now, our chunk.memory points to a memory block that is chunk.size
//      * bytes big and contains the remote file.
//      *
//      * Do something nice with it!
//      */
// 
//     printf("%lu bytes retrieved\n", (long)chunk->size);
//   }

  /* cleanup curl stuff */
  curl_easy_cleanup(curl_handle);

  // this may beed to be removed
  // free(chunk->memory);

  /* we're done with libcurl, so clean it up */
  curl_global_cleanup();

  return 0;
}

unsigned char* get_pointertopix(unsigned char *startpointer , int row, int col, int rowwidth)
{
    // row and col counters begin at 0 and end at n-1
    // startpointer -- pointerto startig to memory chunk
    // row -- row of pixel begining from 0
    // col -- col of pixel begining from 0
    // rowwidth -- width of a row in pixels

    int rowwidth_bytes = rowwidth*3;
    unsigned char *pointertopix = startpointer + row*rowwidth_bytes + col*3;
    return pointertopix;

}

long sumpixels_box(unsigned char *startpointer , int row1, int col1, int row2, int col2, int rowwidth)
{
    long sumred = 0;
    // pointers to pixels in row
    unsigned char *curpx;
    // pointer to the bute in the pixels
    unsigned char *curby;
    // unsigned char *curloc = locinbmpbuffer;
    // printf("Sumpixels init \n");

    for(int rowi = row1; rowi < row2; rowi++) {
        for(int coli = col1; coli < col2; coli++) {
            // printf("In for loop 2 \n");
            curpx = get_pointertopix(startpointer, rowi, coli, rowwidth);
            for(int bytei = 0; bytei < 3; bytei++) {
                curby = curpx + bytei;
                // make it red!
                if (bytei == 2) {
                    sumred = sumred + (long)*curby;
                }
                else {
                    // *curby = 0;
                }
            }
        }
    }


    // printf("Sumpixels init \n");
    return sumred;
}

unsigned long countredpixels(struct BmpStruct *bmpstructPtr, unsigned char redthresh, \
        unsigned char bluethresh, unsigned char greenthresh)
{
    // printf("Counting reds; R > %u, G < %u, B < %u \n", redthresh, bluethresh, greenthresh);

    unsigned long redcounter = 0;
    unsigned char *curpx, *pxr, *pxg, *pxb;
//     unsigned char redthresh = 245;
//     unsigned char greenthresh = 175;
//     unsigned char bluethresh = 175;
    // int height = (*bmp).height;
    int height = bmpstructPtr->height;
    int width = bmpstructPtr->width;
    unsigned char *bmp_buffer = bmpstructPtr->bmp_buffer;
    // printf("Height %d \n", height);

    for(int rowi = 0; rowi < height; rowi++) {
        // printf("Row: %d \n", rowi);
        for(int coli = 0; coli < width; coli++) {
            // curpx = locinbmpbuffer + 3*pxi; // RGB stored consequtively
            curpx = get_pointertopix(bmp_buffer, rowi, coli, width);
            pxr = curpx;
            pxg = curpx + 1;
            pxb = curpx + 2;

            if(*pxr > redthresh && *pxg < greenthresh && *pxb < bluethresh) {
                // printf("(x,,y):%3d %3d. RGB: %3u, %3u, %3u \n", rowi, coli, *pxr, *pxg, *pxb);
                redcounter++;
            }
        }
    }
    printf("Total reds above %u: %ld \n", redthresh, redcounter);
    curredcount = redcounter;
    return redcounter;

}

int do_cv(unsigned char *jpg_buffer, unsigned long jpg_size)
{
    // // INTIALIZE
    int rc;
	// Variables for the decompressor itself
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

	// Variables for the output buffer, and how long each row is
	unsigned long bmp_size;
	unsigned char *bmp_buffer;
	int row_stride, width, height, pixel_size;

    // printf("jpeg; Settting up \n");
    // // SETUP AND CHECK
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);

	jpeg_mem_src(&cinfo, jpg_buffer, jpg_size);

	rc = jpeg_read_header(&cinfo, TRUE);
	if (rc != 1) {
        printf("FILE IS PROBABLY NOT JPEG");
		// exit(EXIT_FAILURE);
        return 0;
	}

	jpeg_start_decompress(&cinfo);

	width = cinfo.output_width;
	height = cinfo.output_height;
	pixel_size = cinfo.output_components;

	bmp_size = width * height * pixel_size;
	bmp_buffer = (unsigned char*) malloc(bmp_size);

	// The row_stride is the total number of bytes it takes to store an
	// entire scanline (row).
	row_stride = width * pixel_size;

    // printf("jpeg; reading \n");
    // // READ THE LINES
	while (cinfo.output_scanline < cinfo.output_height) {
		unsigned char *buffer_array[1];
		buffer_array[0] = bmp_buffer + (cinfo.output_scanline) * row_stride;

		jpeg_read_scanlines(&cinfo, buffer_array, 1);

	}

    struct BmpStruct bmp;

    bmp.bmp_buffer = bmp_buffer;
    bmp.size = bmp_size;
    bmp.width = width;
    bmp.height = height;
    bmp.pixel_size = pixel_size;

    // unsigned long redcount = countredpixels(&bmp, 200,175,175);
    countredpixels(&bmp, 200,175,175);

    // printf("jpeg; destroying jpeg \n");
    // // DESTROY CREATED OBJECT
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

    // printf("jpeg; writing to file \n");
    // // WRITE TO FILE
    int output = 0;
    if (output == 1) {
        int fd;
        fd = open("output.ppm", O_CREAT | O_WRONLY, 0666);
        char buf[1024];

        rc = sprintf(buf, "P6 %d %d 255\n", width, height);
        write(fd, buf, rc); // Write the PPM image header before data
        write(fd, bmp_buffer, bmp_size); // Write out all RGB pixel data

        // printf("jpeg; closing deleting \n");
        close(fd);
    }

	return 1;
}

int runcurltomem(void)
{
  struct MemoryStruct chunk;
  chunk.memory = malloc(1);
  chunk.size = 0;

  // printf("Downloading \n");
  curl2mem(&chunk);

  // printf("Jpegging \n");
  do_cv((unsigned char*)chunk.memory, chunk.size);

  // FILE *fp;
  // fp = fopen("myout","wb");
  // fwrite((const void *)chunk.memory, sizeof(char),chunk.size,fp);

  // printf("Size of char: %lu; size of memory: %lu \n", sizeof(char),chunk.size);
  //
  free(chunk.memory);
  // printf("Done \n");

  return 0;
}

int update_redsatheading(void)
{
    struct MemoryStruct chunk;
    chunk.memory = malloc(1);
    chunk.size = 0;

    // printf("Downloading \n");
    curl2mem(&chunk);

    // printf("Jpegging \n");
    do_cv((unsigned char*)chunk.memory, chunk.size);
    redsatheading[heading_ind] = curredcount;
    heading_ind++;
    // FILE *fp;
    // fp = fopen("myout","wb");
    // fwrite((const void *)chunk.memory, sizeof(char),chunk.size,fp);

    // printf("Size of char: %lu; size of memory: %lu \n", sizeof(char),chunk.size);
    //
    free(chunk.memory);
    // printf("Done \n");

    return 0;
}

int printredsatheading(void)
{
    for(int i = 0; i < 35; i++) {
        printf("H: %3d; redcount: %6lu \n", (i)*10, redsatheading[i]);
    }
    return 0;
}
