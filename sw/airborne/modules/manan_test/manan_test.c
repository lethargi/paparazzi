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
#include <fcntl.h> //this one used to write to ppmfile

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

uint16_t heading_ind = 0;
uint32_t redsatheading[36];
uint32_t curredcount;
// struct BmpStruct curbmp;
uint32_t redcount_arr[3] = {0,0,0};
uint32_t bluecount_arr[3] = {0,0,0};
uint32_t greencount_arr[3] = {0,0,0};
uint32_t count_arr[3][3] = {{0}};
uint32_t sumcount_arr[3] = {0,0,0};
uint32_t domcol_arr[3] = {0,0,0};

// GList* list = NULL;
// GHashTable* qdict = g_hash_table_new(g_str_hash, g_str_equal);

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
    float cur_head = ANGLE_FLOAT_OF_BFP(GetCurHeading());
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

void curl2mem(struct MemoryStruct* chunk)
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

  // return 0;
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

uint32_t count_redpixels(struct BmpStruct *bmpstructPtr)
{
    uint32_t redcounter = 0;
    unsigned char *curpx, *pxr, *pxg, *pxb;
    uint16_t height = bmpstructPtr->height;
    uint16_t width = bmpstructPtr->width;
    unsigned char *bmp_buffer = bmpstructPtr->buffer;

    int colvalsum;
    float redfrac;

    for(int rowi = 0; rowi < height; rowi++) {
        for(int coli = 0; coli < width; coli++) {
            curpx = get_pointertopix(bmp_buffer, rowi, coli, width);
            pxr = curpx;
            pxg = curpx + 1;
            pxb = curpx + 2;
            colvalsum = *pxr + *pxg + *pxb;
            redfrac = (float) *pxr / (float) colvalsum;
            if (redfrac > 0.75) {
                redcounter++;
            }
        }
    }
    // printf("Redcount: %6d \n", redcounter);
    return redcounter;
}

uint8_t colmax(uint32_t colarr[3][3], uint8_t arrwidth, uint8_t arrheight)
{
    for (int i = 0; i < arrwidth; i++) {
        uint32_t curcolmax = 0;
        uint8_t curcolmax_ind = 0;
        for (int j = 0; j < arrheight; j++) {
            if (colarr[j][i] > curcolmax) {
                curcolmax = colarr[j][i];
                curcolmax_ind = j+1;
            }
        }
        domcol_arr[i] = curcolmax_ind;
    }
    return 0;
}

uint8_t count_redpixels_in_three_grids(struct BmpStruct *bmpstructPtr)
{
    unsigned char *curpx, *pxr, *pxg, *pxb;
    uint16_t height = bmpstructPtr->height;
    uint16_t width = bmpstructPtr->width;
    uint16_t width1 = width/3;
    uint16_t width2 = 2*width/3;
    // redcount_arr = {0,0,0};
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

    for(int rowi = 0; rowi < height; rowi++) {
        for(int coli = 0; coli < width; coli++) {
            uint8_t arrtoapp = 100;
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

    colmax(count_arr,3,3);

    return 0;
}

uint32_t sumpixcounts(uint32_t *colarr, uint8_t arrsize)
{
    uint32_t sumpix = 0;
    for (int i = 0; i < arrsize; i++) {
        sumpix += colarr[i];
    }
    return sumpix;
}

/* Counts the number of pixels above provided threshold values of RGB */
unsigned long threshold_pixels(struct BmpStruct *bmpstructPtr, unsigned char redthresh, \
        unsigned char bluethresh, unsigned char greenthresh)
{
    unsigned long redcounter = 0;
    unsigned char *curpx, *pxr, *pxg, *pxb;
    // int height = (*bmp).height;
    uint16_t height = bmpstructPtr->height;
    uint16_t width = bmpstructPtr->width;
    unsigned char *bmp_buffer = bmpstructPtr->buffer;
    // printf("Height %d \n", height);

    for(uint16_t rowi = 0; rowi < height; rowi++) {
        // printf("Row: %d \n", rowi);
        for(uint16_t coli = 0; coli < width; coli++) {
            // curpx = locinbmpbuffer + 3*pxi; // RGB stored consequtively
            curpx = get_pointertopix(bmp_buffer, rowi, coli, width);
            pxr = curpx;
            pxg = curpx + 1;
            pxb = curpx + 2;

            //  Implementing new logic for finding red
            if(*pxr > redthresh && *pxg > greenthresh && *pxb > bluethresh) {
                // printf("(x,,y):%3d %3d. RGB: %3u, %3u, %3u \n", rowi, coli, *pxr, *pxg, *pxb);
                redcounter++;
            }
        }
    }
    printf("Total reds above %u: %ld \n", redthresh, redcounter);
    // curredcount = redcounter;
    return redcounter;

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

uint8_t cv_task(void)
{
    struct MemoryStruct chunk;
    chunk.memory = malloc(1);
    chunk.size = 0;

    struct BmpStruct bmp;

    curl2mem(&chunk);

    get_bmp((unsigned char*)chunk.memory, chunk.size, &bmp);
    // free up memory of downloaded jpeg
    free(chunk.memory);

    curredcount = count_redpixels(&bmp);
    // free up memory of the bmp
    free(bmp.buffer);


    return 0;
}

uint8_t cv_3grids(void)
{
    struct MemoryStruct chunk;
    chunk.memory = malloc(1);
    chunk.size = 0;

    struct BmpStruct bmp;

    curl2mem(&chunk);

    get_bmp((unsigned char*)chunk.memory, chunk.size, &bmp);
    // free up memory of downloaded jpeg
    free(chunk.memory);

    count_redpixels_in_three_grids(&bmp);
//     printf("\n Redcount: ");
//     for(int i=0; i<3; i++) {
//         printf("%d ", redcount_arr[i]);
//     }
//     printf("\n");
    // free up memory of the bmp
    free(bmp.buffer);


    return 0;
}

// uint32_t cv_get_redcount
uint8_t print_cvarrs(void) {
    printf("\n Printing CV-arrs \n Dominant colors \n");
    for (int i = 0; i < 3; i++) {printf("%d ",domcol_arr[i]);}
    printf("\n Sum of colors \n");
    for (int i = 0; i < 3; i++) {printf("%d ",sumcount_arr[i]);}
    printf("\n Colors in grids ");
    char *colarr[3] = {"red","green","blue"};
    for (int i = 0; i < 3; i++) {
        printf("\n %s \n", colarr[i]);
        for (int j = 0; j < 3; j++) {
            printf("%d ", count_arr[i][j]);
        }
        printf("\n");
    }
    printf("\n Done \n");
    return 0;
}
