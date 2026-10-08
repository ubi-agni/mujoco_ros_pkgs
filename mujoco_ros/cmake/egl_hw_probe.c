/* Exit 0 only when EGL can create a context on a non-software renderer. */
#include <EGL/egl.h>
#include <GL/gl.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

static int is_software(const char *r)
{
	static const char *names[] = { "llvmpipe", "softpipe", "swiftshader", "software" };
	char low[256];
	size_t i;
	for (i = 0; r[i] && i < sizeof(low) - 1; ++i)
		low[i] = (char)tolower((unsigned char)r[i]);
	low[i] = 0;
	for (i = 0; i < sizeof(names) / sizeof(names[0]); ++i)
		if (strstr(low, names[i]))
			return 1;
	return 0;
}

int main(void)
{
	EGLint major = 0, minor = 0, cfgn = 0;
	EGLDisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	if (dpy == EGL_NO_DISPLAY)
		return 2;
	if (!eglInitialize(dpy, &major, &minor))
		return 3;
	const EGLint attr[] = { EGL_SURFACE_TYPE,
		                     EGL_PBUFFER_BIT,
		                     EGL_RENDERABLE_TYPE,
		                     EGL_OPENGL_BIT,
		                     EGL_RED_SIZE,
		                     8,
		                     EGL_GREEN_SIZE,
		                     8,
		                     EGL_BLUE_SIZE,
		                     8,
		                     EGL_DEPTH_SIZE,
		                     24,
		                     EGL_NONE };
	EGLConfig cfg;
	if (!eglChooseConfig(dpy, attr, &cfg, 1, &cfgn) || cfgn < 1)
		return 4;
	if (!eglBindAPI(EGL_OPENGL_API))
		return 5;
	const EGLint surface_attributes[] = { EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE };
	EGLSurface surf                   = eglCreatePbufferSurface(dpy, cfg, surface_attributes);
	EGLContext ctx                    = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, NULL);
	if (surf == EGL_NO_SURFACE || ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, surf, surf, ctx))
		return 6;
	const char *r = (const char *)glGetString(GL_RENDERER);
	if (r)
		printf("renderer=%s\n", r);
	if (!r || is_software(r))
		return 7;
	return 0;
}
