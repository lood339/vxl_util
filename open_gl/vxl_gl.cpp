#include <stdio.h>
#include <stdlib.h>
#include "vxl_gl.h"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>


void gl_check_error(void)
{
	GLenum error = glGetError();
	switch (error)
	{
	case GL_NO_ERROR:
		break;
	case GL_INVALID_ENUM:
		fprintf(stderr, "GL_INVALID_ENUM\n");
		exit(-1);
	case GL_INVALID_VALUE:
		fprintf(stderr, "GL_INVALID_VALUE\n");
		exit(-1);
	case GL_INVALID_OPERATION:
		fprintf(stderr, "GL_INVALID_OPERATION\n");
		exit(-1);
	case GL_STACK_OVERFLOW:
		fprintf(stderr, "GL_STACK_OVERFLOW\n");
		exit(-1);
	case GL_STACK_UNDERFLOW:
		fprintf(stderr, "GL_STACK_UNDERFLOW\n");
		exit(-1);
	case GL_OUT_OF_MEMORY:
		fprintf(stderr, "GL_OUT_OF_MEMORY\n");
		exit(-1);
	default:
		fprintf(stderr, "gl_check_error parameter error\n");
		exit(-1);
	}
}



void check_framebuffer_status(void)
{
	GLenum status;
	status = (GLenum) glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch(status) {
		case GL_FRAMEBUFFER_COMPLETE_EXT:
			break;
		case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
			fprintf(stderr, "Unsupported framebuffer format\n");
			exit(-1);			
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
			fprintf(stderr, "Framebuffer incomplete, missing attachment\n");
			exit(-1);
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
			fprintf(stderr, "Framebuffer incomplete, duplicate attachment\n");
			exit(-1);
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
			fprintf(stderr, "Framebuffer incomplete, attached images must have same dimensions\n");
			exit(-1);
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
			fprintf(stderr, "Framebuffer incomplete, attached images must have same format\n");
			exit(-1);
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
			fprintf(stderr, "Framebuffer incomplete, missing draw buffer\n");
			exit(-1);
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
			fprintf(stderr, "Framebuffer incomplete, missing read buffer\n");
			exit(-1);
		default:
			fprintf(stderr, "Framebuffer incomplete\n");
			exit(-1);			
	}
	
}


void VxlGL::image2Texture(const vil_image_view<vxl_byte> & image, TextureBuf & texture)
{
    const unsigned nChannel = image.nplanes();
    const int w = image.ni();
    const int h = image.nj();
    switch (nChannel) {
        case 3:
            texture._channel = 3;
            texture._width  = image.ni();
            texture._height = image.nj();
            texture._data = new unsigned char [w * h * 3];
            for (int j = 0; j<image.nj(); j++) {
                for (int i = 0; i<image.ni(); i++) {
                    int idx = j * w + i;
                    texture._data[3 * idx + 0] = image(i, j, 0);
                    texture._data[3 * idx + 1] = image(i, j, 1);
                    texture._data[3 * idx + 2] = image(i, j, 2);
                }
            }
            break;
            
        default:
            assert(0);
            break;
    }
}

bool VxlGL::genRgbTexture(TextureBuf &texture)
{
	if (texture._channel == 3) {
		gl_check_error();
		if (glIsTexture(texture._name)) {
			glDeleteTextures(1, &texture._name);
		}
		glGenTextures(1, &texture._name);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,  texture._name);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB32F_ARB, texture._width,
					 texture._height, 0, GL_RGB, GL_UNSIGNED_BYTE,  // GL_RGBA
					 texture._data);
		gl_check_error();
		return true;
	}
	else
	{
		fprintf(stderr, "genRgbTexture error\n");
		return false;
	}
}

void VxlGL::drawTex2Quad(const TextureBuf &texture, const vgl_point_2d<double> &starP)
{
    if (texture._name && (glIsTexture(texture._name))) {
        
		int width  = texture._width;
		int height = texture._height;
        
		GLfloat v0[2] = {0, 0};
		GLfloat v1[2] = {(float)width, 0};
		GLfloat v2[2] = {(float)width, (float)height};
		GLfloat v3[2] = {0, (float)height};
        
        
		GLint t0[2] = {0, 0};
		GLint t1[2] = {width, 0};
		GLint t2[2] = {width, height};
		GLint t3[2] = {0, height};
        
		glTranslatef(starP.x(), starP.y(), 0);
		gl_check_error();
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture._name);
		glBegin(GL_TRIANGLES);
		glTexCoord2iv(t0);
		glVertex2fv(v0);
		glTexCoord2iv(t1);
		glVertex2fv(v1);
		glTexCoord2iv(t2);
		glVertex2fv(v2);
        
		glTexCoord2iv(t0);
		glVertex2fv(v0);
		glTexCoord2iv(t2);
		glVertex2fv(v2);
		glTexCoord2iv(t3);
		glVertex2fv(v3);
		glEnd();
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		gl_check_error();
		glTranslatef(-starP.x(), -starP.y(), 0);
	}
	else
	{
		assert(0);
	}
}

void VxlGL::drawTexture2Rectangle(const TextureBuf & texture, const vgl_point_2d<double> & leftTop, const vgl_point_2d<double> & rightBottom)
{
    if (texture._name && (glIsTexture(texture._name))) {
        
		int width  = texture._width;
		int height = texture._height;
        
	
        
        GLfloat v0[2], v1[2], v2[2], v3[2];
        v0[0] = leftTop.x();
        v0[1] = leftTop.y();
        v1[0] = rightBottom.x();
        v1[1] = leftTop.y();
        v2[0] = rightBottom.x();
        v2[1] = rightBottom.y();
        v3[0] = leftTop.x();
        v3[1] = rightBottom.y();
        
        
		GLint t0[2] = {0, 0};
		GLint t1[2] = {width, 0};
		GLint t2[2] = {width, height};
		GLint t3[2] = {0, height};
        
	
		gl_check_error();
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture._name);
		glBegin(GL_TRIANGLES);
		glTexCoord2iv(t0);  glVertex2fv(v0);
		glTexCoord2iv(t1);  glVertex2fv(v1);
		glTexCoord2iv(t2);  glVertex2fv(v2);
       
        
		glTexCoord2iv(t0); 	glVertex2fv(v0);
		glTexCoord2iv(t2); 	glVertex2fv(v2);
		glTexCoord2iv(t3); 	glVertex2fv(v3);
		glEnd();
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		gl_check_error();
	}
	else
	{
		assert(0);
	}
}

void VxlGL::drawImageTexture(const TextureBuf & texture)
{
    if (texture._name && (glIsTexture(texture._name))) {
        
		int width  = texture._width;
		int height = texture._height;
        
        
		gl_check_error();
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture._name);
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);               glVertex3f(0, 0, 0); // top-left
        glTexCoord2f(width, 0);           glVertex3f(1, 0, 0); // bottom-left
        glTexCoord2f(width, height);      glVertex3f(1, 1, 0); // bottom-right
        glTexCoord2f(0, height);          glVertex3f(0, 1, 0); // top-right
		glEnd();
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		gl_check_error();
	}
	else
	{
		assert(0);
	}
    
}
/*
void Ipl2TextureBuf(IplImage *image, const int nChannel, TextureBuf &texture)
{
	assert(image);
	assert(image->width%2 == 0);
	if (nChannel == 1) {
		if (image->nChannels == 1 && image->depth == IPL_DEPTH_8U) {
			int width = image->width;
			int height = image->height;
			texture._width   = width;
			texture._height  = height;
			texture._channel = 1;
			texture._data = new unsigned char[width * height];
			BwImage b_img(image);
			for (int y = 0; y<height; ++y) {
				for (int x = 0; x<width; ++x) {
					int index = y * width + x;
					texture._data[index] = b_img[y][x];				
				}
			}
		}
		else
		{
			assert(0);
		}

	}
	else if (nChannel == 3) {
		if (image->nChannels == 3 && image->depth == IPL_DEPTH_8U) {
			int width = image->width;
			int height = image->height;
			texture._width = width;
			texture._height = height;
			texture._channel = 3;
			texture._data = new unsigned char[3 * width * height];
			RgbImage rgb_img(image);
			for (int y = 0; y<height; ++y) {
				for (int x = 0; x<width; ++x) {
					int index = y * width + x;
					texture._data[3 * index] = rgb_img[y][x].r;
					texture._data[3 * index + 1] = rgb_img[y][x].g;
					texture._data[3 * index + 2] = rgb_img[y][x].b;
				}
			}			
		}
		else
		{
			assert(0);
		}
	}
	else if (nChannel == 4) {
		if (image->nChannels == 3 && image->depth == IPL_DEPTH_8U) {
			int width = image->width;
			int height = image->height;
			texture._width = width;
			texture._height = height;
			texture._channel = 4;
			texture._data = new unsigned char[4 * width * height];
			RgbImage rgb_img(image);
			for (int y = 0; y<height; ++y) {
				for (int x = 0; x<width; ++x) {
					int index = y * width + x;
					texture._data[4 * index] = rgb_img[y][x].r;
					texture._data[4 * index + 1] = rgb_img[y][x].g;
					texture._data[4 * index + 2] = rgb_img[y][x].b;
					texture._data[4 * index + 3] = 255;
				}
			}
		}
		else
		{
			assert(0);
		}
	}
	else
	{
		assert(0);
	}
}

void TextureBuf2Ipl(const TextureBuf &texture, _IplImage *&image)
{
	if (glIsTexture(texture._name)) {
		int width = texture._width;
		int height = texture._height;
		unsigned char *data = new unsigned char[width * height * 3];
		memset(data, 0, sizeof(data[0]) * width * height * 3);
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture._name);
		glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		gl_check_error();

		image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		cvZero(image);
		RgbImage img(image);
		for (int y = 0; y<height; ++y) {
			for (int x = 0; x<width; ++x) {
				int index = y * width + x;
				int r_y = height - 1 - y;
				img[r_y][x].r = data[3*index];
				img[r_y][x].g = data[3*index+1];
				img[r_y][x].b = data[3*index+2];
			}
		}
		delete []data;
	}
	else assert(0);
}
_IplImage *TextureBuf2Ipl(const TextureBuf &texture, int channels, bool rev)
{
	if (glIsTexture(texture._name)) {
		int width = texture._width;
		int height = texture._height;

		unsigned char *data = new unsigned char[width * height * channels];
		memset(data, 0, sizeof(data[0]) * width * height * channels);
		
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture._name);
		if (channels == 4) {
			glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)data);
		}
		else if (channels == 3) {
			glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, GL_UNSIGNED_BYTE, (void*)data);
		}
		else assert(0);			 
		
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		gl_check_error();

		IplImage *image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		cvZero(image);
		RgbImage img(image);
		if (rev) {
			for (int y = 0; y<height; ++y) {
				for (int x = 0; x<width; ++x) {
					int index = y * width + x;
					int r_y = height - 1 - y;
					img[r_y][x].r = data[channels*index];
					img[r_y][x].g = data[channels*index+1];
					img[r_y][x].b = data[channels*index+2];
				}
			}
		}
		else
		{
			for (int y = 0; y<height; ++y) {
				for (int x = 0; x<width; ++x) {
					int index = y * width + x;					
					img[y][x].r = data[channels*index];
					img[y][x].g = data[channels*index+1];
					img[y][x].b = data[channels*index+2];
				}
			}
		}		
		delete []data;
		return image;
	}
	else assert(0);

}

void drawTex2Quad(const TextureBuf &texture, const CvPoint &starP)
{
	if (texture._name && (glIsTexture(texture._name))) {

		float width  = texture._width;
		float height = texture._height;

		GLfloat v0[2] = {0, 0};
		GLfloat v1[2] = {width, 0};
		GLfloat v2[2] = {width, height};
		GLfloat v3[2] = {0, height};


		GLint t0[2] = {0, 0};
		GLint t1[2] = {width, 0};
		GLint t2[2] = {width, height};
		GLint t3[2] = {0, height};

		glTranslatef(starP.x, starP.y, 0);
		gl_check_error();
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture._name);	
		glBegin(GL_TRIANGLES);
		glTexCoord2iv(t0);
		glVertex2fv(v0);
		glTexCoord2iv(t1);
		glVertex2fv(v1);	
		glTexCoord2iv(t2);
		glVertex2fv(v2);

		glTexCoord2iv(t0);
		glVertex2fv(v0);
		glTexCoord2iv(t2);
		glVertex2fv(v2);
		glTexCoord2iv(t3);
		glVertex2fv(v3);
		glEnd();
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		gl_check_error();
		glTranslatef(-starP.x, -starP.y, 0);
	}
	else
	{
		assert(0);
	}
}
 
 */
void calc_projectionMatrix(const float *eye, const float *center,
						   const float *up, float *matrix)
{
	gl_check_error();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(eye[0], eye[1], eye[2],
		      center[0], center[1], center[2],
			  up[0], up[1], up[2]);
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	glPopMatrix();
	gl_check_error();
}
