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

#include <curl/curl.h>

// imports for jpeg; dont know the relevance of all of them
// #include <fcntl.h> //this one used to write to ppmfile

#include <jpeglib.h>


// #include "generated/flight_plan.h"

struct MemoryStruct
{
  char *memory;
  size_t size;
};

struct BmpStruct
{
    unsigned char *buffer;
    unsigned long size;
    uint16_t width;
    uint16_t height;
    uint8_t pixel_size;
    uint16_t row_stride;
};

// uint16_t heading_ind = 0;
// uint32_t redsatheading[36];
// uint32_t curredcount;
// struct BmpStruct curbmp;
uint32_t redcount_arr[3] = {0,0,0};
uint32_t bluecount_arr[3] = {0,0,0};
uint32_t greencount_arr[3] = {0,0,0};
uint32_t count_arr[3][3] = {{0}};
uint32_t sumcount_arr[3] = {0,0,0};
uint8_t domcol_arr[3] = {0,0,0};

// GList* list = NULL;
// GHashTable* qdict = g_hash_table_new(g_str_hash, g_str_equal);

void my_init()
{
    printf("MY MODULE INIT \n\n");
}

uint8_t set_nav_heading(float newheading)
{
    // printf("Oldhead: %f; NewHead: %f\n", ANGLE_FLOAT_OF_BFP(nav_heading), newheading);
    nav_heading = ANGLE_BFP_OF_REAL(newheading);
    INT32_ANGLE_NORMALIZE(nav_heading); // HEADING HAS INT32_ANGLE_FRAC....
    return FALSE;
}
//Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
uint8_t increase_nav_heading(float increment)
{
  nav_heading += ANGLE_FLOAT_OF_BFP(increment);
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(nav_heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}

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
	  // new_coor.z = waypoint->z; // Keep the height the same
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
      float wpheight = waypoint_get_alt(waypoint);
      // waypoint_set_here(waypoint);
      waypoint_set_alt(waypoint, wpheight);

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

uint8_t setHeadingNorth(void)
{
    float cur_head = GetCurHeading();
    // float cur_navhead = ANGLE_FLOAT_OF_BFP(nav_heading);
    // printf("Nav %f; Curhead %f \n", ANGLE_FLOAT_OF_BFP(nav_heading),ANGLE_FLOAT_OF_BFP(GetCurHeading()));

    if (abs(cur_head) > 1.57) {
        if (cur_head > 0) {
            nav_heading = ANGLE_BFP_OF_REAL(RadOfDeg(90));
        }
        else {
            nav_heading = ANGLE_BFP_OF_REAL(RadOfDeg(270));
        }
        INT32_COURSE_NORMALIZE(nav_heading);
        return TRUE;
    }
    else {
        nav_heading = ANGLE_BFP_OF_REAL(0);
        INT32_COURSE_NORMALIZE(nav_heading);
        return FALSE;
    }
    // if (abs(cur_head) < 0.1
}

/*
uint8_t setHeadingSmooth(float targhead)
{
    float cur_head = GetCurHeading();
    float diff_head = targhead-cur_head
    // float cur_navhead = ANGLE_FLOAT_OF_BFP(nav_heading);
    // printf("Nav %f; Curhead %f \n", ANGLE_FLOAT_OF_BFP(nav_heading),ANGLE_FLOAT_OF_BFP(GetCurHeading()));

    if (abs(diff_head) > 3.14) {
        diff_head = cur_head-targhead
        }

    if (abs(diff_head) > 1.57) {
        if (diff_head > 0) {
            nav_heading = ANGLE_BFP_OF_REAL(targhead+RadOfDeg(90));
        }
        else {
            nav_heading = ANGLE_BFP_OF_REAL(targhead+RadOfDeg(90));
        }
        INT32_COURSE_NORMALIZE(nav_heading);
        return TRUE;
    }
    else {
        nav_heading = ANGLE_BFP_OF_REAL(targhead);
        INT32_COURSE_NORMALIZE(nav_heading);
        return FALSE;
    }
    // if (abs(cur_head) < 0.1
}
*/

uint8_t setAltToWp(uint8_t waypoint_toset, uint8_t waypoint_ref)
{
      float wpheight = waypoint_get_alt(waypoint_ref);
      // waypoint_set_here(waypoint);
      waypoint_set_alt(waypoint_toset, wpheight);
      return FALSE;
}

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    /* Helper function for curl2mem. Handles the writing of the data from
     * the source HTTP into the memory. Needs to follow format provided by
     * libcurl.
     */
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

void curl2mem(struct MemoryStruct *chunk)
{
    /* Uses libcurl functions to load the screenshot from the FlightGear HTTP
     * server into the programs memory
     */
  CURL *curl_handle;
  CURLcode res;

  curl_global_init(CURL_GLOBAL_ALL);

  /* init the curl session */
  curl_handle = curl_easy_init();

  /* specify URL to get */
  curl_easy_setopt(curl_handle, CURLOPT_URL, "http://localhost:1234/screenshot");

  /* send all data to this function  */
  curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);

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

  /* cleanup curl stuff */
  curl_easy_cleanup(curl_handle);

  /* we're done with libcurl, so clean it up */
  curl_global_cleanup();
}

uint8_t get_bmp(unsigned char *jpg_buffer, unsigned long jpg_size, struct BmpStruct *bmp)
{
    // // INTIALIZE
    uint8_t rc;
	// Variables for the decompressor itself
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

	// Variables for the output buffer, and how long each row is
	unsigned long bmp_size;
	unsigned char *bmp_buffer;
	uint16_t row_stride, width, height, pixel_size;

    // holds the output bmp and its metadata

    // printf("jpeg; Settting up \n");
    // // SETUP AND CHECK
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);

	jpeg_mem_src(&cinfo, jpg_buffer, jpg_size);

    // 	skipping error check for now. file should be JPEG
	rc = jpeg_read_header(&cinfo, TRUE);
	// jpeg_read_header(&cinfo, TRUE);
	if (rc != 1) {
// 		// exit(EXIT_FAILURE);
        printf("File to get_bmp() not JPEG");
        return 1;
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

    bmp->buffer = bmp_buffer;
    bmp->size = bmp_size;
    bmp->width = width;
    bmp->height = height;
    bmp->pixel_size = pixel_size;

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

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

uint8_t colmax(uint32_t colarr[3][3],uint8_t maxcolarr[3])
{
    for (int i = 0; i < 3; i++) {
        uint32_t curcolmax = 0;
        uint8_t curcolmax_ind = 0;
        for (int j = 0; j < 3; j++) {
            if (colarr[j][i] > curcolmax) {
                curcolmax = colarr[j][i];
                curcolmax_ind = j+1;
            }
        }
        maxcolarr[i] = curcolmax_ind;
    }
    return 0;
}

uint8_t count_pixels_in_three_grids(struct BmpStruct *bmpstructPtr)
{
    unsigned char *curpx, *pxr, *pxg, *pxb;
    uint16_t height = bmpstructPtr->height;
    uint16_t width = bmpstructPtr->width;
    uint16_t width1 = width/3;
    uint16_t width2 = 2*width/3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            count_arr[i][j] = 0;
        }
        sumcount_arr[i] = 0;
        domcol_arr[i] = 0;
    }

    unsigned char *bmp_buffer = bmpstructPtr->buffer;

    int colvalsum;
    float redfrac,bluefrac,greenfrac;

    uint8_t arrtoapp = 100;
    for(int rowi = 0; rowi < height; rowi++) {
        for(int coli = 0; coli < width; coli++) {
            arrtoapp = 100;
            curpx = get_pointertopix(bmp_buffer, rowi, coli, width);
            pxr = curpx;
            pxg = curpx + 1;
            pxb = curpx + 2;
            colvalsum = *pxr + *pxg + *pxb;
            redfrac = (float) *pxr / (float) colvalsum;
            greenfrac = (float) *pxg / (float) colvalsum;
            bluefrac = (float) *pxb / (float) colvalsum;

            if (redfrac > 0.75) { arrtoapp=0; }
            else if (greenfrac > 0.75) { arrtoapp=1; }
            else if (bluefrac > 0.65) { arrtoapp=2; }

            if (arrtoapp < 3) {
                if (coli < width1) { count_arr[arrtoapp][0]++; }
                else if (coli < width2) { count_arr[arrtoapp][1]++; }
                else { count_arr[arrtoapp][2]++; }
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        uint32_t sumpix = 0;
        for (int j = 0; j < 3; j++) {
            sumpix += count_arr[i][j];
        }
        sumcount_arr[i] = sumpix;
    }

    colmax(count_arr,domcol_arr);

    return 0;
}

uint8_t cv_3grids(void)
{
    struct MemoryStruct chunk; // structure holding download data
    chunk.memory = malloc(1);
    chunk.size = 0;

    struct BmpStruct bmp; // structure holding decompressed image

    // download the jpeg into internal memory
    curl2mem(&chunk);

    // decompress the image and store it in preallocated structure
    get_bmp((unsigned char*)chunk.memory, chunk.size, &bmp);

    // free up memory of downloaded jpeg
    free(chunk.memory);

    count_pixels_in_three_grids(&bmp);

    // free up memory of the bmp
    free(bmp.buffer);

    return 0;
}
