#ifndef VXL_GL_H
#define VXL_GL_H 1

#include <OpenGL/gl.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>

struct TextureBuf{
	int _width;
	int _height;
	int _channel;
	unsigned char *_data;   //0-255 ,
	GLuint _name;
	TextureBuf(){_data = 0;}
	~TextureBuf(){delete []_data, _data = 0;}
};
//mesh on the rectangle
struct Rect2DMesh
{
	int _width;
	int _height;
	float *_xy;
	float *_uv;
	Rect2DMesh(int w, int h, bool uv)
	{
		_width = w, _height = h;
		_xy = new float[_width * _height * 2];		
		if (uv) { _uv = new float[_width * _height * 2];}
		else {_uv = 0;}
	}
	~Rect2DMesh(){delete []_xy, _xy = 0; delete []_uv, _uv = 0;}
};


//opengl_error
void gl_check_error(void);
//

//
void check_framebuffer_status(void);

// vxl and opengl extrange data
class VxlGL
{
public:
    
    static void image2Texture(const vil_image_view<vxl_byte> & image, TextureBuf & texture);
    
    static bool genRgbTexture(TextureBuf &texture);
    
    static void drawTex2Quad(const TextureBuf &texture, const vgl_point_2d<double> &starP);
    
    static void drawTexture2Rectangle(const TextureBuf & texture, const vgl_point_2d<double> & leftTop,
                                      const vgl_point_2d<double> & rightBottom);
    static void drawImageTexture(const TextureBuf & texture);
};



inline void gl_red(void){glColor3f(1.0, 0.0, 0.0);}
inline void gl_green(void){glColor3f(0.0, 1.0, 0.0);}
inline void gl_blue(void){glColor3f(0.0, 0.0, 1.0);}
inline void gl_white(void){glColor3f(1.0, 1.0, 1.0);}
inline void gl_black(void){glColor3f(0.0, 0.0, 0.0);}


#endif
