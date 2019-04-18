/*
 * Copyright (c) 2012-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVFLIPPER_H
#define INCLUDED_NVFLIPPER_H

enum nv_flipper_format {
    nv_flipper_format_A8R8G8B8
};

/*
 * Initializes backend functionality and allocates two frame buffers.
 *
 * \display_head    display Head to use [0, 1]
 * \display_window  display Window (hardware layer) to use [0, 2]
 * \window_depth    z-order for display window [0, 255]
 * \format          specifies format of frame buffer
 * \width           specifies width of frame buffer
 * \height          specifies height of frame buffer
 */
int nvInitFlipper(
    int display_head,
    int display_window,
    int window_depth,
    enum nv_flipper_format format,
    int width, int height);

void nvDeinitFlipper(void);

/*
 * Allocates a surface of size width*height in buffer.
 */
unsigned int allocSurf(void *buffer[1],
                       int width, int height);

/*
 * Gets important arguments from RM
 *
 * \buffer      pointer array of size two, to store the frame buffer pointers
 * \fb_*        addresses to store values describing the frame buffer dimensions
 */
void getArgs(void *buffer[2], unsigned int *fb_width,
             unsigned int *fb_height, unsigned int *fb_pitch);


/*
 * Gets flip id of last flip
 */
unsigned long getFlipId(void);


/*
 * Flips framebuffers
 * \fb_idx          index into buffer array returned by getArgs() [0, 1]
 *                  pass fb_idx as -1 to flip a NULL buffer to the display window.
 *                  nvflipper allocates two frame buffers, we need to alternate
 *                  between using the two buffers, there will be tearing if we update
 *                  the frame buffer which was last passed to nvflip().
 * \dst_x, dst_y    x and y offsets into the hardware overlay
 */
int nvflip(int fb_idx,
           unsigned int dst_x, unsigned int dst_y);

#endif  // INCLUDED_NVFLIPPER_H
