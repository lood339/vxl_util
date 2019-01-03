#include "vxl_glu.h"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include "vxl_gl.h"

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
